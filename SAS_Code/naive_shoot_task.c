#include "cmsis_os.h"
#include "pid.h"
#include "remote_control.h"
#include "nucCommu.h"
#include "robot_total_mode.h"
#include "CAN_receive.h"
#include "main.h"
#include "detect_task.h"
#include "OLED.h"
#include "pid.h"

//注意：当遥控器右拨杆指向中央的时候，才允许鼠标操控

// #define ZERO_CURRENT_SAFE

//*************任务设置
#define SHOOT_TASK_INIT_TIME 500
#define SHOOT_CTRL_TIME 5



//**************射击电机控制常量 15 0.40 0.24 0.53   1.0 0.6 1.72 1.0 18 0.45
//#define SHOOT_SPEED_LIMIT 0.525f        //摩擦轮速度。未来可以测试摩擦轮速度和射速的关系 0.60f 
double SHOOT_SPEED_LIMIT = 0.525;
bool_t is_x_pressed = 0;
#define SHOOT_TRIGGER_SPEED_LIMIT 0.5f   //拨弹轮开启时速度
#define READY_TRIGGER_SPEED 0.625f				//拨弹轮准备速度



#define PRESS_LONG_TIME     200    

//**********单发圈数控制相关
#define TRIGGER_ROUNDS_FOR_A_BULLET_A 4   
#define TRIGGER_ROUNDS_FOR_A_BULLET_B 4     
#define TRIGGER_ROUNDS_FOR_A_BULLET_C 4
#define ECD_FULL_ROUND 8192

//************遥控器和键鼠设置
#define SHOOT_MODE_CHANNEL 1    //遥控器左拨杆控制发射
// #define MOUSE_SHOOT  鼠标左键按下抬起——单发，按下不放——连发，右键按下：允许NUC控制发射       
#define KEY_FRIC_ON     KEY_PRESSED_OFFSET_SHIFT    //按shift开启摩擦轮

// #define TEAMER_ALLOW_SHOOT  //按下鼠标右键不放，允许nuc控制发射

//*************电机速度环PID
#define M3505_MOTOR_SPEED_PID_KP 60000.0f
#define M3505_MOTOR_SPEED_PID_KI 0.0f
#define M3505_MOTOR_SPEED_PID_KD 0.0f
//老拨弹盘 20000
#define M2006_MOTOR_SPEED_PID_KP 20000.0f

#define M2006_MOTOR_SPEED_PID_KI 0.0f   //不要这个积分项，否则连续发射停止后还会转
#define M2006_MOTOR_SPEED_PID_KD 1000.0f

#define MAX_MOTOR_SHOOT_CAN_CURRENT 30000.0f
#define M3505_MOTOR_SHOOT_SPEED_PID_MAX_OUT MAX_MOTOR_SHOOT_CAN_CURRENT
#define M3505_MOTOR_SHOOT_SPEED_PID_MAX_IOUT 2000.0f
//70000
#define MAX_MOTOR_CAN_CURRENT_M2006 30000.0f
#define M2006_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT_M2006
#define M2006_MOTOR_SPEED_PID_MAX_IOUT 2000.0f


#define SHOOT_M3508_MOTOR_RPM_TO_VECTOR (0.000415809748903494517209f/5)
#define SHOOT_M2006_MOTOR_RPM_TO_VECTOR (0.000415809748903494517209f/5)

//*************************里程碑栈法的控制参数***********//
#define MILESTONE_NEAR_THRESHHOLD 2000
#define MILESTONE_NUMBER 3

//**********************卡弹相关***********************//
#define STUCK_TIME_LIMIT 1500   //当在发射一发的模式中持续一段时间不退出后，进入卡弹解决模式
#define SOLVING_TIME_LIMIT 500  //在尝试卡弹解决模式中停留的时间。



const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
const static fp32 trigger_motor_speed_pid[3] = {M2006_MOTOR_SPEED_PID_KP, M2006_MOTOR_SPEED_PID_KI, M2006_MOTOR_SPEED_PID_KD};

//***********拨弹轮电机模式枚举类型
enum TriggerMode_e{
    TriggerMode_e_Stop,         //停止发射
    TriggerMode_e_ShootOne,     //发射一个，靠拨弹轮减速前的电机转4圈。对应弹盘轮子转一格
    TriggerMode_e_ShootMulti,   //多发，其实是等待进入发射一个的状态
    TriggerMode_e_StuckSolve,   //尝试解决卡弹
};

//**********拨弹轮ECD圈数监控器
struct milestoneStack_s {
    uint8_t head;
    uint8_t stack[MILESTONE_NUMBER+1];
};

struct TriggerControl_s{
    struct milestoneStack_s mstack; //里程碑栈
    const uint16_t * ECDPoint;           //电机ECD所在位置
    uint16_t    initECD;                //初始ECD
    uint16_t    nowECD;                 //现在的ECD
    int16_t     nowRounds;              //现在转过的圈数
};


//**********缓冲指令
struct InstructBuff_s{
    uint8_t turnOnFric  ; //为1时表示指令开启摩擦轮，为0时表示指令关闭摩擦轮
    uint8_t shootOne    ;
    uint8_t shootMulti  ;
};


//*******************全局变量********//
// 摩擦轮开启
bool_t fricOn=0;
extern bool_t wantFricOn; // LYL
bool_t wantTriggerOn = 0; // LYL
bool_t mouseWantTriggerOn = 0;
bool_t keywantFricOn = 0;

// 拨弹轮开启
uint8_t triggerOn=0;
uint8_t reverse = 0;

static struct InstructBuff_s insBuff;   //全局发射机构控制指令缓冲
static enum TriggerMode_e triggerMode;  //拨弹轮电机状态
// 在云台任务中控制速度

static int zeroCurrentMark;    //当离线时，直接发送零电流的时候
const static RC_ctrl_t *rc_p;
static const enum RobotState_e *robotMode;   //机器人模式
static const toSTM32_t *nuc_p;  //NUC数据位置。未来制造全自动机器人时需要

static int highspeedshoot=0;
static int16_t giveShootCurrent[2];
int16_t giveTriggerCurrent;
static pid_type_def shootMotorPIDs[2],triggerMotorPID;
static fp32 wantedVShootMotor[2],wantedVTriggerMotor;
// int16_t motor_speed_rpm;
static fp32 presentVShootMotor[2],presentVTriggerMotor;

static struct TriggerControl_s triggerCtrl;
static enum TriggerMode_e triggerMode;
static const enum RobotState_e *robotMode;
static uint32_t pressTimeDebug;


static void initShootPIDs(void)
{
    uint8_t i;
    for(i=0;i<2;i++)
        PID_init(&shootMotorPIDs[i],PID_POSITION,motor_speed_pid,M3505_MOTOR_SHOOT_SPEED_PID_MAX_OUT,M3505_MOTOR_SHOOT_SPEED_PID_MAX_IOUT);
    PID_init(&triggerMotorPID,PID_POSITION,trigger_motor_speed_pid,M2006_MOTOR_SPEED_PID_MAX_OUT,M2006_MOTOR_SPEED_PID_MAX_IOUT);
}

/**
 * @brief 将ECD差值等化为(0,8191)范围
 * 
 * @param rawECD 
 * @return uint16_t 
 */
static uint16_t ECDFormat(int16_t rawECD)     //test done
{
    while(rawECD<0)
        rawECD+=ECD_FULL_ROUND;
    while(rawECD>=ECD_FULL_ROUND)
        rawECD-=ECD_FULL_ROUND;
    return (uint16_t)rawECD;
}

static void initTriggerECDRoundsMonitor()  //初始化拨弹轮圈数监控
{
    triggerCtrl.ECDPoint=&(get_trigger_motor_measure_point()->ecd);
    triggerCtrl.initECD=*(triggerCtrl.ECDPoint);
}   

//添加NUC控制发射
static void getInstructionAndBuff(void)
{
    static uint32_t pressTime=0;
    static uint8_t up=0,down=0;
    zeroCurrentMark=0;
            
    //通过按键开启摩擦轮（也许用处并不大，有了遥控器的就足够了，但还是用来确保开启了摩擦轮吧）
    if(rc_p->key.v & KEY_FRIC_ON)
		{
			if(insBuff.turnOnFric==1)
				insBuff.turnOnFric=0;
		}
    //当遥控器左拨杆指向中央的时候，允许鼠标操控
    if(switch_is_mid(rc_p->rc.s[SHOOT_MODE_CHANNEL]))
    {
        if(rc_p->mouse.press_l) //按下左键
        {
            if(pressTime<PRESS_LONG_TIME)
                pressTime+=SHOOT_CTRL_TIME;
            else    //已经按了足够长的时间了，可以不用再增加了，
            {
                insBuff.shootMulti=1;
            }
        }
        else    //左键抬起
        {
            if(pressTime<PRESS_LONG_TIME)   //拨下并很快抬起，发射一颗
                if(pressTime>0)
                {
                    insBuff.shootOne=1; 
                }
            pressTime=0;
            insBuff.shootMulti=0;
        }
    }

    if(switch_is_up(rc_p->rc.s[SHOOT_MODE_CHANNEL]))
    {
        up=1;
    }
    else if(switch_is_mid(rc_p->rc.s[SHOOT_MODE_CHANNEL]))
    {
        //通过上拨和下拨，实现摩擦轮状态的控制
        if(up==1)   // 从上到中 下拨
        {
            if(fricOn)
            {
                // fricOn=0;
                // triggerOn=0;
                //为了防止卡弹，使用缓冲指令设计。当拨弹轮停止时才能关闭摩擦轮。
                insBuff.shootMulti=0;
                insBuff.shootOne=0;
                insBuff.turnOnFric=0;   //希望关闭摩擦轮
            }    
            else
            {
                // fricOn=1;        
                insBuff.turnOnFric=1;   //希望开启摩擦轮
            }
            up=0;
        }
        if(down==1)  // 刚刚是下方，从下到中的上拨
        {
            insBuff.shootMulti=0;   //停止连发
            if(pressTime<PRESS_LONG_TIME)   //拨下并很快抬起，发射一颗
                if(pressTime>0)
                {
                    insBuff.shootOne=1; 
                }
            pressTime=0;
            down=0;
        }

    }
    else if(switch_is_down(rc_p->rc.s[SHOOT_MODE_CHANNEL]))
    {
        down=1;
        if(pressTime<PRESS_LONG_TIME)
            pressTime+=SHOOT_CTRL_TIME;
        else    //已经按了足够长的时间了，可以不用再增加了，
        {
            insBuff.shootMulti=1;
        }
    }
    pressTimeDebug=pressTime;

		// LYL
//		if(robotIsAuto())
//		{

//			if(nuc_p->is_fire==0x02&&(nuc_p->confidence.data>0.9845))
//				insBuff.shootMulti = 1 ;
//			else if((nuc_p->is_fire==0x01&&(nuc_p->confidence.data>0.9845)))
//				insBuff.shootOne = 1;
//			else
//			{

//        insBuff.shootOne=0;
//        insBuff.shootMulti=0;
//			}
//		}
		
		// LYL
		static bool_t is_v_pressed = 0;
		if (rc_p->key.v & KEY_PRESSED_OFFSET_V){
			osDelay(80);
			if ((rc_p->key.v & KEY_PRESSED_OFFSET_V) && !is_v_pressed) {
				if (keywantFricOn) {keywantFricOn = 0;}
				else {keywantFricOn = 1;}
				is_v_pressed = 1;
			} else {
				is_v_pressed = 0;
			}
		}
		if (rc_p->mouse.press_l)
		{
			mouseWantTriggerOn = 1;
		} else {
			mouseWantTriggerOn = 0;
		}
		
		// LYL
		if (rc_p->key.v & KEY_PRESSED_OFFSET_X){
			osDelay(80);
			if ((rc_p->key.v & KEY_PRESSED_OFFSET_X) && !is_x_pressed) {
				is_x_pressed = 1;
			} else {
				is_x_pressed = 0;
			}
		}
		
		// LYL
		if (robotIsAuto() && wantFricOn){
			if((nuc_p->confidence.data>0.9865)){
//				insBuff.shootMulti = 1 ;
				wantTriggerOn = 1;
			} else {
//        insBuff.shootOne=0;
//        insBuff.shootMulti=0;
				wantTriggerOn = 0;
			}
			//usart_printf("%d,%f\r\n",wantTriggerOn,nuc_p->confidence.data);
		}
		else 
		{
			wantTriggerOn = 0;
		}

    if(toe_is_error(DBUS_TOE))
    {
        zeroCurrentMark=1;
        triggerOn=0;
        fricOn=0;
        triggerMode=TriggerMode_e_Stop;
        insBuff.turnOnFric=0;
        insBuff.shootOne=0;
        insBuff.shootMulti=0;
    }
}


static void monitorTriggerECDRound(void)
{
    uint8_t j;
    // 更新ECD
    triggerCtrl.nowECD=*(triggerCtrl.ECDPoint);

    for(j=0;j<MILESTONE_NUMBER;j++)    //枚举每一个里程碑所在位置
    {
        fp32 relativeRealECD;
        relativeRealECD=ECDFormat((int16_t)triggerCtrl.nowECD-(int16_t)triggerCtrl.initECD);
        //失败原因是0的比较出现了问题
        
        if(ECDFormat(relativeRealECD-j*ECD_FULL_ROUND/MILESTONE_NUMBER)<MILESTONE_NEAR_THRESHHOLD)
                //当前位置落在相应里程碑点所在区域内
        {
            if(j!=(triggerCtrl.mstack.stack[triggerCtrl.mstack.head]))
               //不等说明到达了一个新位置，将此新位置加入栈中
            {
                triggerCtrl.mstack.head++;
                #ifdef WATCH_ARRAY_OUT
                if(c->mstack.head>=MILESTONE_NUMBER)
                {
                    itHappens();    //  让usb task输出此数组越界信息
                    c->mstack.head=MILESTONE_NUMBER-1;
                }
                    
                #endif
                triggerCtrl.mstack.stack[triggerCtrl.mstack.head]=j;
            }
        }
    }
    if(((triggerCtrl.mstack.head)-2)>=0)
    {
        if(triggerCtrl.mstack.stack[triggerCtrl.mstack.head]==triggerCtrl.mstack.stack[triggerCtrl.mstack.head-2])
            (triggerCtrl.mstack.head)-=2;
    }
    if(((triggerCtrl.mstack.head)-3)>=0)
    {
        if(triggerCtrl.mstack.stack[triggerCtrl.mstack.head]==triggerCtrl.mstack.stack[triggerCtrl.mstack.head-3])
        {//到达了一圈
            if(triggerCtrl.mstack.stack[1]==1)//正向旋转（逆时针）
                triggerCtrl.nowRounds +=1;
            else
                triggerCtrl.nowRounds -=1;
            triggerCtrl.mstack.head=0;     // 清空栈，回到初始为0的时候
        }
    }
}
static void initShootModes(void)
{
    //初始化指令
    insBuff.shootMulti=0;
    insBuff.shootOne=0;
    insBuff.turnOnFric=0;
    triggerMode=TriggerMode_e_Stop;
    fricOn=0;
    triggerOn=0;
}


static void fricModeChange(void)
{
    //等待拨弹轮关闭时再关闭摩擦轮
    if((!(insBuff.turnOnFric)) || (*robotMode)==RobotState_e_Powerless)
    {
        if(triggerMode==TriggerMode_e_Stop)
        {
            fricOn=0;
            insBuff.shootMulti=0;
            insBuff.shootOne=0;
            insBuff.turnOnFric=0;
        }
    }
    else
    {
        fricOn=1;
    }        
}

static void triggerModeChange(void)
{
    static uint32_t lastShootMultiTime=0;
    static uint32_t shootOneEnterTime = 0;  //进入发射一发的状态的时刻
    // static uint32_t shootOneStayTime = 0;   // 在发射一发的状态中停留的时间
    static uint32_t solveStuckEnterTime = 0;   // 在发射一发的状态中停留的时间
    // static uint32_t solveStuckNowTime = 0;   // 在尝试解决卡弹状态中的现在时间，直接使用nowTime 代替
    
    static uint32_t nowTime=0;
    nowTime=HAL_GetTick();  //每次进入此函数都会更新
    
    if(!fricOn) //摩擦轮关闭状态，始终让拨弹轮关闭
    {
        triggerMode=TriggerMode_e_Stop;
        return;
    }

    //对每个状态枚举
    if(triggerMode==TriggerMode_e_Stop)
    {
        //机器人不处于无力状态时，接收从stop状态转变为其他状态的指令
        if((*robotMode)!=RobotState_e_Powerless)
        {
            if(insBuff.shootOne)
            {
                triggerMode=TriggerMode_e_ShootOne;
                shootOneEnterTime = nowTime;
                insBuff.shootOne=0; //清除缓冲的指令
            }
            else if (insBuff.shootMulti)
            {
                triggerMode=TriggerMode_e_ShootMulti;
                insBuff.shootMulti=0;   //清除缓冲的指令
            }
                
        }
    }
    else if(triggerMode==TriggerMode_e_ShootOne)
    {
				//改进圈数+ECD控制se3333
        static uint8_t nowTimeRoundThreshold=TRIGGER_ROUNDS_FOR_A_BULLET_A;
        if(triggerCtrl.nowRounds<=(-nowTimeRoundThreshold)||triggerCtrl.nowRounds>=nowTimeRoundThreshold)
        {
            triggerMode=TriggerMode_e_Stop;
            triggerCtrl.nowRounds=0;
            if(nowTimeRoundThreshold==TRIGGER_ROUNDS_FOR_A_BULLET_A)
                nowTimeRoundThreshold=TRIGGER_ROUNDS_FOR_A_BULLET_B;
            if(nowTimeRoundThreshold==TRIGGER_ROUNDS_FOR_A_BULLET_B)
                nowTimeRoundThreshold=TRIGGER_ROUNDS_FOR_A_BULLET_C;
						if(nowTimeRoundThreshold==TRIGGER_ROUNDS_FOR_A_BULLET_C)
								nowTimeRoundThreshold=TRIGGER_ROUNDS_FOR_A_BULLET_A;
        }
        //否则留在这个状态

    }
    else if(triggerMode==TriggerMode_e_ShootMulti)
    {
				if(insBuff.shootMulti==0)
						triggerMode = TriggerMode_e_Stop;
    }

}

static void setSpeedByMode(void)
{
    if(fricOn || wantFricOn || keywantFricOn)
    {
        wantedVShootMotor[0]=-SHOOT_SPEED_LIMIT;//实验测试转动方向。反过来可以实验卡弹时的退弹
        wantedVShootMotor[1]=SHOOT_SPEED_LIMIT;
    }
    else
    {
        wantedVShootMotor[0]=0;
        wantedVShootMotor[1]=0;
    }
    if(triggerMode==TriggerMode_e_ShootOne ||  triggerMode == TriggerMode_e_ShootMulti)
        triggerOn=1;
    else 
        triggerOn=0;
    
    reverse = 0;

    if(triggerOn || wantTriggerOn || mouseWantTriggerOn)
        wantedVTriggerMotor=SHOOT_TRIGGER_SPEED_LIMIT;//若正在解决stuck(reverse)，则以逆向v/4的速度转动
		else
				wantedVTriggerMotor=0;

}

extern int is_rc_online_new;
static void calcGiveCurrent1(void)
{
    int i;
    for(i=0;i<2;i++)
	{
        PID_calc(&shootMotorPIDs[i],presentVShootMotor[i],wantedVShootMotor[i]);
//				usart_printf("%d:%d ",i,shootMotorPIDs[i].out);
	}
//	usart_printf("\n");
    for(i=0;i<2;i++)
        giveShootCurrent[i]=shootMotorPIDs[i].out;
    PID_calc(&triggerMotorPID,presentVTriggerMotor,wantedVTriggerMotor);
    giveTriggerCurrent=triggerMotorPID.out;
	
	if (!is_rc_online_new)
	{
		giveTriggerCurrent=0;
	}
	
}

static void getShootMotorSpeed(void)
{
    uint8_t i;
    for(i=0;i<2;i++)
    {
        presentVShootMotor[i]=get_shoot_motor_measure_point(i)->speed_rpm*SHOOT_M3508_MOTOR_RPM_TO_VECTOR;
				//usart_printf("%d\r\n",get_shoot_motor_measure_point(i)->speed_rpm);
    }
		
    presentVTriggerMotor=get_trigger_motor_measure_point()->speed_rpm*SHOOT_M2006_MOTOR_RPM_TO_VECTOR;
}

void shoot_task(void const *pvParameters)
{
    osDelay(SHOOT_TASK_INIT_TIME);
		
		// LYL
		if (is_x_pressed)  SHOOT_SPEED_LIMIT = 0.51;
		else SHOOT_SPEED_LIMIT = 0.525;
	
    initShootPIDs();    //初始化PID
    initTriggerECDRoundsMonitor();  //初始化拨弹轮圈数监控
    initShootModes();   //初始化发射状态
    // initWantedSpeed();  //初始化电机速度和电流 全局变量自己就是0；
    robotMode=getRobotPresentMode();
    rc_p=get_remote_control_point();    //获取遥控器数据和NUC数据指针
		nuc_p = get_nuc_control_point();
    // triggerMonitorSubOn=1;
    while(1)
    {
        getInstructionAndBuff();    //获取指令并缓冲
        fricModeChange();           //摩擦轮状态改变
        //若摩擦轮关闭状态，则清除单发和连发指令

        triggerModeChange();        //拨弹轮状态转变
        setSpeedByMode();

        // monitorTriggerECDRound();        //监控拨弹轮ECD圈数
        getShootMotorSpeed();
			
			// LYL
//			usart_printf("%f,%f\r\n",presentVShootMotor[0],presentVShootMotor[1]);
			
        calcGiveCurrent1();                   //计算PID
                                    //发送控制电流
        #ifdef ZERO_CURRENT_SAFE
        zeroCurrentMark=1;
        #endif
        
				//usart_printf("%d\r\n",is_rc_online_new);
			
			//usart_printf("%d,%d\r\n", get_shoot_motor_measure_point(0)->temperate, get_shoot_motor_measure_point(1)->temperate);
				
        if(zeroCurrentMark)
        {
            CAN_cmd_shoot(0,0);
            giveTriggerCurrent=0;
        }
        else
            CAN_cmd_shoot(giveShootCurrent[0], giveShootCurrent[1]);
				
				// LYL
				//usart_printf("%f,%f\r\n",giveShootCurrent[0],giveShootCurrent[1]);
//				usart_printf("%f,%f,%d\r\n",wantedVTriggerMotor,presentVTriggerMotor,giveTriggerCurrent);
        //usart_printf("%f,%f,%f,%f,%d,%d\r\n",presentVShootMotor[0],presentVShootMotor[1],-wantedVShootMotor[0],-wantedVShootMotor[1],giveShootCurrent[0], giveShootCurrent[1]);
				//usart_printf("%d,%d,%d,%d\r\n", get_chassis_motor_measure_point(0)->speed_rpm, get_chassis_motor_measure_point(1)->speed_rpm, get_chassis_motor_measure_point(2)->speed_rpm, get_chassis_motor_measure_point(3)->speed_rpm);
				
				osDelay(SHOOT_CTRL_TIME);
    }
}

int16_t * getTriggerCurrentP(void)
{
    return &giveTriggerCurrent;
}

bool_t get_fric(void)
{
	return fricOn;
}
//struct M2006Control_s{
//    struct milestoneStack_s mstack; //里程碑栈
//    const uint16_t * ECDPoint;           //电机ECD所在位置
//    uint16_t    initECD;                //初始ECD
//	  uint16_t    targetECD;              //目标ECD
//    uint16_t    nowECD;                 //现在的ECD
//    int16_t     nowRounds;              //现在转过的圈数
//		int16_t     targetRounds;           //目标转到的圈数
//		int16_t	    set_speed;								//目标转速
//		int16_t*    now_speed;								//当前转速
//		enum M2006Mode mode;								//当前2006的模式
//		pid_type_def       pid;                    //速度PID
//	  pid_type_def       angle_pid;               //角度PID
//};
//struct M2006Control_s M2006Ctrl[4];
//enum M2006Mode{
//	M2006_POWERLESS = 0		,
//	M2006_ROTATE_FORWARD 	,
//	M2006_ROTATE_BACKWARD ,
//	M2006_STOP
//};
//#define SPEED_M2006 1500
//#define MAX_MOTOR_ECD 8192
//uint16_t ecd_limit(uint16_t ref, uint16_t set)
//{
//	if(set > ref)
//	{
//		if(set - ref > ref - set + MAX_MOTOR_ECD)
//		{
//			return set - MAX_MOTOR_ECD;//error = set - MAX_MOTOR_ECD - ref
//		}
//		else if(set - ref < ref - set + MAX_MOTOR_ECD)
//		{
//			return set;//error = set -ref
//		}
//	}
//	else if(set < ref)
//	{
//		if(ref - set > set - ref + MAX_MOTOR_ECD)
//		{
//			return set + MAX_MOTOR_ECD;//error = 
//		}
//		else if(ref - set < set - ref + MAX_MOTOR_ECD)
//		{
//			return set;//error
//		}
//	}
//}
//fp32 PID_ECD_calc(pid_type_def *pid, fp32 ref, fp32 set)
//{
//	PID_calc(pid, ref, ecd_limit(ref, set));
//}

//void set_M2006_speed()
//{
//	for(int i=0;i<4;i++)
//	{
//		if(M2006Ctrl[i].mode==M2006_ROTATE_FORWARD)
//		{
//			M2006Ctrl[i].set_speed=SPEED_M2006;
//		}
//		else if (M2006Ctrl[i].mode==M2006_ROTATE_BACKWARD)
//		{
//			M2006Ctrl[i].set_speed=-SPEED_M2006;
//		}
//		else if (M2006Ctrl[i].mode==M2006_STOP)
//		{
//			M2006Ctrl[i].set_speed=(int)PID_ECD_calc(&M2006Ctrl[i].angle_pid,*M2006Ctrl[i].ECDPoint,M2006Ctrl[i].targetECD);
//		}
//  }
//}
////拨弹盘控制任务

//void refresh_M2006_ctrl(){
//	for(int i=0;i<4;i++)
//	{
//		if(M2006Ctrl[i].nowRounds>M2006Ctrl[i].targetRounds)
//			M2006Ctrl[i].mode=M2006_ROTATE_BACKWARD;
//		if(M2006Ctrl[i].nowRounds<M2006Ctrl[i].targetRounds)
//			M2006Ctrl[i].mode=M2006_ROTATE_FORWARD;
//		if(M2006Ctrl[i].nowRounds==M2006Ctrl[i].targetRounds)
//		{
//			M2006Ctrl[i].mode=M2006_STOP;
//			M2006Ctrl[i].targetRounds = M2006Ctrl[i].nowRounds;
//		}
//  }
//}
//void set_M2006_current(){
//	int current[4];
//	for(int i=0;i<4;i++)
//		current[i]=PID_calc(&M2006Ctrl[i].pid,*M2006Ctrl[i].now_speed,M2006Ctrl[i].set_speed);
//}
////为了减小延迟使用的trigger monitor，独立于shoot任务进行
void shootTaskTrggMonitor(void const *pvParameters)
{
    osDelay(SHOOT_TASK_INIT_TIME);
    while(1)
    {
        // if(triggerMonitorSubOn)
        monitorTriggerECDRound();
//
        osDelay(1);
    }
}

