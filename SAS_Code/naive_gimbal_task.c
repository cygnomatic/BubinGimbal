/**
 * encoding:GBK
 * @file naive_chassis_task.c
 * @author 
 * @brief 
 * @version 0.2
 * @date 2023-03-26
 * 
 * 
 * 功能定义
 * 
 */

#include "bsp_buzzer.h"
#include "cmsis_os.h"
#include "pid.h"
#include "bsp_laser.h"
#include "arm_math.h"
#include "CAN_receive.h"
#include "ins_task.h"
#include "main.h"
#include "remote_control.h"
#include "bsp_usart.h"
#include "nucCommu.h"
#include "naive_gimbal_task.h"
#include "robot_total_mode.h"
#include "RobotStructure.h"
#include "usb_task.h"
#include "OLED.h"
#include "user_lib.h"
#include "detect_task.h"
#include "usart.h"
#include "can.h"
#include <cmath>
#include "RobotStructure.h"
#include "test_task.h"
#define GIMBAL_TASK_INIT_TIME 500
#define GIMBAL_TASK_CTRL_TIME 2


#define SAFE_AUTO_SHIFT_TIME 1500   //自动模式时，当装甲板丢失时，使用手动控制，
                //但不清空操作手鼠标偏移量。当重新从手动进入自动模式模式时，操作手可能因为来不及反应
                //导致偏移量破坏。因此设计安全时间，当重新进入时，有一段时间无视操作手新增偏移量。
                //这段时间为SAFE_AUTO_SHIFT_TIME 单位ms
//操作手相关
#define CHANNEL_MAX 660
#define K_CTRL_EXP_X 1.5
#define K_CTRL_EXP_Y 2.5
#define K_CTRL_EXP_MOUSE_X 1
#define K_CTRL_EXP_MOUSE_Y 1
#define K_CTRL_PROP_X 20.0
#define K_CTRL_PROP_Y 20.0
#define K_CTRL_MOUSE_X 40
#define K_CTRL_MOUSE_Y 5


//****************************机器人测试获得值*************//
#define PITCH_TEST_INIT_ANGLE_MIDDLE_ECD 4628
#define PITCH_TEST_MAX_ANGLE_ECD 5500 // 241.73度 总：8191
#define PITCH_TEST_MIN_ANGLE_ECD 4800 // 210.96度
#define PITCH_MAX_ANGLE_ECD 0.27f //0.275763俯角
#define PITCH_MIN_ANGLE_ECD -0.28f //-0.280368仰角
// #define PITCH_TEST_ANGLE_ECD_LENGTH 1039

#define PITCH_TEST_RAD_MIN (-0.28f)
#define PITCH_TEST_RAD_MAX  (0.38f)

#define YAW_TEST_INIT_ANGLE_MIDDLE_ECD 3557
#define YAW_TEST_MAX_ANGLE_ECD 5616
#define YAW_TEST_MIN_ANGLE_ECD 1534
// #define YAW_TEST_ANGLE_ECD_LENGTH 1039

#define NO_SLIP_RING_MAX_ROUNDS 0

/*****
 * Try again at 2023/3/16 21:00
 */

// 输入角速度 rad/s 、输出电压 int16_t 的PID系数
#define PITCH_SPD_KP 10000000.0f
// #define PITCH_SPD_KP 10000000.0f

#define PITCH_SPD_KI 2000.0f
#define PITCH_SPD_KD 1000.0f
                            
#define PITCH_VOLT_MAX_OUT  30000.0f    // prev ent overflow of control control volt
                                        // because the control volt is -30000~30000
#define PITCH_VOLT_MAX_IOUT 3000.0f

//输入角度 rad ，输出角速度rad/s 的PID系数
#define PITCH_AGL_KP 0.015f
#define PITCH_AGL_KI 0.0005f
#define PITCH_AGL_KD 0.01f

#define PITCH_AGL_SPD_MAX_OUT (2.0f)
#define PITCH_AGL_SPD_MAX_IOUT (1.0f)


//yawPID锟剿诧拷
#define YAW_SPD_KP 10000000.0f //10000000.0f
#define YAW_SPD_KI 0.0f
#define YAW_SPD_KD 1000000.0f

#define YAW_VOLT_MAX_OUT  30000.0f
#define YAW_VOLT_MAX_IOUT 10000.0f

// #define YAW_AGL_KP 0.07f
#define YAW_AGL_KP 0.0083f // 0.0085
#define YAW_AGL_KI 0.000f // 0.00
#define YAW_AGL_KD 0.00000f // 0.000015

#define YAW_AGL_SPD_MAX_OUT (50.0f)
#define YAW_AGL_SPD_MAX_IOUT (11.7f)

#define YAW_SPD_FILTER_NUM 1.0f

#define GIMBAL_DEAD_BOND 0.00 // LYL:死区值
#define YAW_DEAD_BOND 0.00 // 0.0007 安全临界死区








#define ECD_FULL_ROUND 8192

#define GIMBAL_FILTER_ON

//*****************************遥控器通道选项**************//
#define GIMBAL_YAW_OR_CHASSIS_W_CHANNEL 0     //右边的左右摇杆
#define GIMBAL_PITCH_CHANNEL 1  //右边的前后摇杆
#define RC_DEADLINE 10  //低于这个值的摇杆读数一律为0
#define TO_CHASSIS_COEF 1

#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }


//****************************控制量计算系数**************//
// 使用遥控器和键盘控制云台，被控制量为速度
// 遥控器和键盘控制都是速度，控制量和速度
// NUC控制时，目标控制量为角度
#define YAW_RC_SEN      -0.000004f
#define PITCH_RC_SEN    -0.000005f //0.005
#define YAW_MOUSE_SEN   -0.000005f
#define PITCH_MOUSE_SEN -0.000005f

//估计数量级

// 遥控器最大值660
// 0.5秒(500ms)角度增量 1.7rad
//5.1515151515151515151515151515152e-6

//鼠标最大值：32767
//0.5秒(500ms)角度增量 1.7rad
// 1.0376293221839045381023590807825e-7


//*************************里程碑栈法的控制参数***********//
#define MILESTONE_NEAR_THRESHHOLD 200
#define MILESTONE_NUMBER 3

//限幅函数 //test done
#define range_limit_inside(toBeLimited, range)                      \
    {                                                               \
         if((toBeLimited)>(range))                                  \
        {                                                           \
            (toBeLimited)=(range);                                  \
        }                                                           \
        else if((toBeLimited)<-(range))                             \
        {                                                           \
            (toBeLimited)=-(range);                                 \
        }                                                           \
    }

//一阶滤波器的参数,专门给。这两个值越大，理论上越丝滑
// #define PITCH_SPD_FILTER_NUM 15.0f
// #define YAW_SPD_FILTER_NUM 15.0f

#define PITCH_SPD_FILTER_NUM 0.001f


 #define LimitMax(input, max)   \
     {                          \
         if (input > max)       \
         {                      \
             input = max;       \
         }                      \
         else if (input < -max) \
         {                      \
             input = -max;      \
         }                      \
     }


// 为了调试，需要一些数据结构,从usb输出调试
// 
// 云台在一定时间内控制无法到达指定位置，就放弃控制
struct milestoneStack_s {
    uint8_t head;
    uint8_t stack[MILESTONE_NUMBER+1];
};
struct gimbalMotorCtrl_s{
    //rads
    const fp32 * anglePoint;          //从姿态解算任务得到的角度位置
    fp32 nowAbsoluteAngle;      //当前姿态角
    uint32_t nowTime;           //当前时间
    fp32 lastAbsoluteAngle;     //上次姿态角
    uint32_t lastTime;          //上次时间
    fp32 radSpeed;              //角速度 rad/s
    fp32 wantedAbsoluteAngle;   //目标姿态角

    //ECDs
    const uint16_t * ECDPoint;        //ECD参数所在位置
    uint16_t maxECD,minECD,middleECD,nowECD;    //最大、最小、中值、当前ECD
    int16_t nowRounds;          //现在转过的圈数
    int16_t maxRounds;          //转过的最大正负圈数
    struct milestoneStack_s mstack; //和圈数记录相关变量

    //PIDs
    pid_type_def spd_pid;       //用于控制速度的PID环。输入角速度 rad/s, 输出GM6020电压 int16
    pid_type_def agl_pid;       //用于控制角度的PID环。输入角度 rad,输出角速度rad/s

    //控制量
    int16_t giveVolt;       //GM6020发送电压
    uint8_t zeroVoltMark;

    // 一阶滤波器，过滤角速度
    first_order_filter_type_t spd_filter;
};

static struct gimbalMotorCtrl_s gimbalPitchCtrl,gimbalYawCtrl;
static struct gimbalMotorCtrl_s *gimbalCtrl[2]={&gimbalPitchCtrl,&gimbalYawCtrl};
static fp32 yawSpdPIDco[3]={YAW_SPD_KP,YAW_SPD_KI,YAW_SPD_KD};
static fp32 pitchSpdPIDco[3]={PITCH_SPD_KP,PITCH_SPD_KI,PITCH_SPD_KD};
static fp32 pitchAglPIDco[3]={PITCH_AGL_KP,PITCH_AGL_KI,PITCH_AGL_KD};
static fp32 yawAglPIDco[3]={YAW_AGL_KP,YAW_AGL_KI,YAW_AGL_KD};
static const enum RobotState_e *robotMode;
static enum RobotState_e lastMode;

//遥控器数据,用于累加
static int16_t rc_mouse_x;
static int16_t rc_mouse_y;
static int16_t press_x=0;

//控制指针
static const RC_ctrl_t *rc_p;   //遥控器位置指针，需要初始化
static const toSTM32_t *nuc_p;  //NUC数据位置

//自瞄控制
double delta_pitch,delta_yaw;

static struct GimbalToChassis_s toChassis={0,0};

extern bool_t fricOn; // LYL
bool_t wantFricOn=0; // LYL，用处见naive_shoot_task.c

static void initGimbalCtrls(void)
{
    uint8_t i;
    //初始化绝对角度位置
    gimbalYawCtrl.anglePoint=&(get_INS()->Yaw);         //第一个角度
    gimbalPitchCtrl.anglePoint=&(get_INS()->Pitch); //第三个角度
    // 初始化ECD
    gimbalYawCtrl.ECDPoint=&(get_yaw_gimbal_motor_measure_point()->ecd);//云台读不到也无所谓
    gimbalPitchCtrl.ECDPoint=&(get_pitch_gimbal_motor_measure_point()->ecd);

    gimbalPitchCtrl.maxECD=PITCH_TEST_MAX_ANGLE_ECD;
    gimbalPitchCtrl.minECD=PITCH_TEST_MIN_ANGLE_ECD;
    gimbalPitchCtrl.middleECD=PITCH_TEST_INIT_ANGLE_MIDDLE_ECD;

    gimbalYawCtrl.maxECD=YAW_TEST_MAX_ANGLE_ECD;
    gimbalYawCtrl.minECD=YAW_TEST_MIN_ANGLE_ECD;
    gimbalYawCtrl.middleECD=YAW_TEST_INIT_ANGLE_MIDDLE_ECD;
    //初始化转过的角度
    gimbalPitchCtrl.maxRounds=0;
    #ifdef HAVE_SLIP_RING
    gimbalYawCtrl.maxRounds=30000;
    #else
    gimbalYawCtrl.maxRounds=NO_SLIP_RING_MAX_ROUNDS;
    #endif

    // 初始化PID参数
    PID_init(&(gimbalPitchCtrl.spd_pid),PID_POSITION,pitchSpdPIDco,PITCH_VOLT_MAX_OUT,PITCH_VOLT_MAX_IOUT);
    PID_init(&(gimbalYawCtrl.spd_pid),PID_POSITION,yawSpdPIDco,YAW_VOLT_MAX_OUT,YAW_VOLT_MAX_IOUT);
    
    //滤波初始化
    const static fp32 gimbal_pitch_order_filter[1] = {PITCH_SPD_FILTER_NUM};
    const static fp32 gimbal_yaw_order_filter[1] = {YAW_SPD_FILTER_NUM};

    first_order_filter_init(&(gimbalPitchCtrl.spd_filter), GIMBAL_TASK_CTRL_TIME, gimbal_pitch_order_filter);
    first_order_filter_init(&(gimbalYawCtrl.spd_filter), GIMBAL_TASK_CTRL_TIME, gimbal_yaw_order_filter);
    
    //初始化当前角度和目标角度、pid参数、里程碑栈法数据
    for(i=0;i<2;i++)
    {
        (gimbalCtrl[i])->zeroVoltMark=0;
        (gimbalCtrl[i])->nowAbsoluteAngle=*((gimbalCtrl[i])->anglePoint);
        (gimbalCtrl[i])->nowTime=HAL_GetTick();
        gimbalCtrl[i]->wantedAbsoluteAngle=(gimbalCtrl[i])->nowAbsoluteAngle;
        (gimbalCtrl[i]->mstack).head=0;
        (gimbalCtrl[i]->mstack).stack[0]=0;
    }
    PID_init(&(gimbalCtrl[0]->agl_pid),PID_POSITION,pitchAglPIDco,PITCH_AGL_SPD_MAX_OUT,PITCH_AGL_SPD_MAX_IOUT);
    PID_init(&(gimbalCtrl[1]->agl_pid),PID_POSITION,yawAglPIDco,YAW_AGL_SPD_MAX_OUT,YAW_AGL_SPD_MAX_IOUT);
    
    // 初始化toChassis控制量
    toChassis.effective=0;
    toChassis.w=0;
}

/**
 * @brief 将角度化为(-PI,PI)范围内
 * 
 * @param rawAngle 
 * @return fp32 
 */
fp32 radFormat(fp32 rawAngle)   //test done
{
    while(rawAngle>PI)
        rawAngle-=(2*PI);
    while(rawAngle<(-PI))
        rawAngle+=(2*PI);
    return rawAngle;
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

/**
 * @brief 将0-8191的ECD值转化为-PI~PI的弧度值,只是映射一一过去
 * 0->-PI,8192->PI
 * 
 * @param ecd 
 * @return fp32 
 */
fp32 ECD2Rad(uint16_t ecd)  //test done
{
    return ((fp32)ecd)*(2*PI)/ECD_FULL_ROUND-PI;
}

/**
 * @brief 计算转圈的PID。内部error先循环转化为(-PI,PI)范围
 * 
 * @param pid 
 * @param ref 反馈值
 * @param set 设定值
 * @return fp32 
 */
fp32 gimbal_PID_calc(pid_type_def *pid, fp32 ref, fp32 set)
{
    if (pid == NULL)
    {
        return 0.0f;
    }

    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    pid->set = set;
    pid->fdb = ref;
    pid->error[0] = radFormat(set - ref);
//		if(fabs(pid->error[0])<0.003)
//			pid->error[0]=0;
    if(pid->mode == PID_POSITION)
    {
        pid->Pout = pid->Kp * pid->error[0];
			if(toe_is_error(DBUS_TOE))
				pid->Iout=0.0f;
       pid->Iout += pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        range_limit_inside(pid->Iout, pid->max_iout);
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        range_limit_inside(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)
    {
			  if(pid->error[0]<100){
					pid->error[0]=0;
				
				}
				else{
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        pid->Iout = pid->Ki * pid->error[0];
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        pid->Dout = pid->Kd * pid->Dbuf[0];
        pid->out += pid->Pout + pid->Iout + pid->Dout;
				}
    }
    return pid->out;
}

void refreshAngleStates(struct gimbalMotorCtrl_s * c)
{
    //当前姿态角和角速度
    c->lastAbsoluteAngle=c->nowAbsoluteAngle;
    c->lastTime=c->nowTime;

    c->nowAbsoluteAngle=*(c->anglePoint);
    c->nowTime=xTaskGetTickCount();

    float angleDifference = c->nowAbsoluteAngle - c->lastAbsoluteAngle;

		if (angleDifference > PI) {
				angleDifference -= 2 * PI; // 超过π，减去2π，保持在-π到π之间
		} else if (angleDifference < -PI) {
				angleDifference += 2 * PI; // 小于-π，加上2π，保持在-π到π之间
		}

		c->radSpeed = angleDifference / (c->nowTime - c->lastTime);
    //当前ECD角度
    c->nowECD=*(c->ECDPoint);
}

/**
	* @brief 非线性调整机器人控制状态
 * 
 */
int adjust_channel(int16_t channel, float ctrl_prop, float ctrl_exp)
{
	float a = powf((fabs(channel)/CHANNEL_MAX), ctrl_exp) * CHANNEL_MAX * (channel > 0 ? 1.f : -1.f) * ctrl_prop;
	//usart_printf("%f, %f\r\n", (float)channel, a);
	return a;
	
}

void getControlAngles(void)
{
		
    fp32 selfTargetOffsetPitch=0,selfTargetOffsetYaw=0;
    int16_t yaw_channel = 0, pitch_channel = 0;
    rc_deadband_limit(rc_p->rc.ch[GIMBAL_YAW_OR_CHASSIS_W_CHANNEL], yaw_channel, RC_DEADLINE);
    rc_deadband_limit(rc_p->rc.ch[GIMBAL_PITCH_CHANNEL], pitch_channel, RC_DEADLINE);
	
//	  usart_printf("%d\r\n",*robotMode);

//根据模式选择标志
//视觉测试版本
     
		//usart_printf("%f,%f\r\n",gimbalPitchCtrl.wantedAbsoluteAngle,gimbalPitchCtrl.nowAbsoluteAngle);
    if(*robotMode==RobotState_e_Powerless)
    {
        toChassis.effective=0;
        toChassis.w=0;
        gimbalYawCtrl.zeroVoltMark=1;
        gimbalPitchCtrl.zeroVoltMark=1;
    }    
    else 
    {
				if(lastMode==RobotState_e_Powerless)
				{
						//usart_printf("OK\r\n");
						gimbalPitchCtrl.wantedAbsoluteAngle = gimbalPitchCtrl.nowAbsoluteAngle;
						gimbalYawCtrl.wantedAbsoluteAngle = gimbalYawCtrl.nowAbsoluteAngle;
						gimbalYawCtrl.zeroVoltMark=0;
						gimbalPitchCtrl.zeroVoltMark=0;
				}
        
			  
				//pitch轴目标值控制
				gimbalPitchCtrl.wantedAbsoluteAngle+=1*pitch_channel*PITCH_RC_SEN-10*rc_p->mouse.y*PITCH_MOUSE_SEN;
//				//gimbalPitchCtrl.wantedAbsoluteAngle += adjust_channel(pitch_channel, K_CTRL_PROP_Y, K_CTRL_EXP_Y) * PITCH_RC_SEN - adjust_channel(rc_p->mouse.y, K_CTRL_MOUSE_Y, K_CTRL_EXP_Y) * PITCH_MOUSE_SEN ;

				
				//yaw轴目标值控制
        gimbalYawCtrl.wantedAbsoluteAngle+=5*yaw_channel*YAW_RC_SEN+20*rc_p->mouse.x * YAW_MOUSE_SEN;
				//gimbalYawCtrl.wantedAbsoluteAngle +=  adjust_channel(yaw_channel, K_CTRL_PROP_X, K_CTRL_EXP_X) * YAW_RC_SEN + adjust_channel(rc_p->mouse.x,K_CTRL_MOUSE_X,K_CTRL_EXP_MOUSE_X)*YAW_MOUSE_SEN;

				//修复bug
				if(gimbalYawCtrl.wantedAbsoluteAngle>=3.086)
					gimbalYawCtrl.wantedAbsoluteAngle-=2*PI;
				if(gimbalYawCtrl.wantedAbsoluteAngle<=-3.196)
					gimbalYawCtrl.wantedAbsoluteAngle+=2*PI;
				
			// LYL修改
			//usart_printf("%d,%d\n", robotIsAuto(),fricOn);
//			if(robotIsAuto() && !fricOn)
			if(robotIsAuto())
			{
				wantFricOn = 1;
				delta_pitch =  -nuc_p->pitch.data;
				delta_yaw = nuc_p->yaw.data;
				// LYL
				//usart_printf("detect:%f,%f\tnow:%f,%f\r\n",delta_yaw, delta_pitch ,gimbalYawCtrl.nowAbsoluteAngle, gimbalPitchCtrl.nowAbsoluteAngle);
				if(delta_pitch > 0.00001 || delta_pitch < -0.00001){
					gimbalPitchCtrl.wantedAbsoluteAngle = delta_pitch;
				} else {
					gimbalPitchCtrl.wantedAbsoluteAngle = gimbalPitchCtrl.nowAbsoluteAngle;
				}
				if(delta_yaw > 0.00001 || delta_yaw < -0.00001){
					gimbalYawCtrl.wantedAbsoluteAngle = delta_yaw;
				} else {
					gimbalYawCtrl.wantedAbsoluteAngle = gimbalYawCtrl.nowAbsoluteAngle;
				}
			} else {
				wantFricOn = 0;
				delta_pitch = 0;
				delta_yaw = 0;
			}

    }
    
    lastMode=*robotMode;
    
}

void monitorRounds(struct gimbalMotorCtrl_s * c)
{
    uint8_t j;
    for(j=0;j<MILESTONE_NUMBER;j++)    //枚举每一个里程碑所在位置
    {
        fp32 relativeRealECD;
        relativeRealECD=ECDFormat((int16_t)c->nowECD-(int16_t)c->middleECD);
        //失败原因是0的比较出现了问题
        
        if(ECDFormat(relativeRealECD-j*ECD_FULL_ROUND/MILESTONE_NUMBER)<MILESTONE_NEAR_THRESHHOLD)
                //当前位置落在相应里程碑点所在区域内
        {
            if(j!=(c->mstack.stack[c->mstack.head]))
               //不等说明到达了一个新位置，将此新位置加入栈中
            {
                c->mstack.head++;
                #ifdef WATCH_ARRAY_OUT
                if(c->mstack.head>=MILESTONE_NUMBER)
                {
                    itHappens();    //  让usb task输出此数组越界信息
                    c->mstack.head=MILESTONE_NUMBER-1;
                }
                    
                #endif
                c->mstack.stack[c->mstack.head]=j;
            }
        }
    }
    if(((c->mstack.head)-2)>=0)
    {
        if(c->mstack.stack[c->mstack.head]==c->mstack.stack[c->mstack.head-2])
            (c->mstack.head)-=2;
    }
    if(((c->mstack.head)-3)>=0)
    {
        if(c->mstack.stack[c->mstack.head]==c->mstack.stack[c->mstack.head-3])
        {//到达了一圈
            if(c->mstack.stack[1]==1)//正向旋转（逆时针）
                c->nowRounds +=1;
            else
                c->nowRounds -=1;
            c->mstack.head=0;     // 清空栈，回到初始为0的时候
        }
    }
}

void limitAngles(struct gimbalMotorCtrl_s * c)
{
    // 超过最大2PI角度时,限制幅度   //test done
    if(radFormat(c->wantedAbsoluteAngle - c ->nowECD) > 4096)
        c->wantedAbsoluteAngle = c ->nowECD + 4096;
    if(radFormat(c->wantedAbsoluteAngle - c ->nowECD) < -4096)
        c->wantedAbsoluteAngle = c ->nowECD - 4096;
    
#ifndef HAVE_SLIP_RING
    // 达到或超过最大圈数且还要加强越界的旋转时
    fp32 posiSafe=radFormat(ECD2Rad(c->maxECD)-ECD2Rad(c->nowECD));      //大于0
    fp32 negSafe=radFormat(ECD2Rad(c->minECD)-ECD2Rad(c->nowECD));       //小于0
#endif

    //不限制自转速度。自转速度交给底盘控制任务去限制
}

/**
 * @brief 用ECD限制pitch角度，一定要事先确定pitch中央ECD不含0
 * 
 */
void limitAnglesSecond(void)
{
     
    if(gimbalPitchCtrl.wantedAbsoluteAngle>PITCH_TEST_RAD_MAX)
        gimbalPitchCtrl.wantedAbsoluteAngle=PITCH_TEST_RAD_MAX;
    if(gimbalPitchCtrl.wantedAbsoluteAngle<PITCH_TEST_RAD_MIN)
        gimbalPitchCtrl.wantedAbsoluteAngle=PITCH_TEST_RAD_MIN;
    
}
int count =0;
void calcPID(void)
{
	
		// 软件限幅
	if(gimbalPitchCtrl.wantedAbsoluteAngle > PITCH_MAX_ANGLE_ECD)
	{
		gimbalPitchCtrl.wantedAbsoluteAngle = PITCH_MAX_ANGLE_ECD;
	} else if (gimbalPitchCtrl.wantedAbsoluteAngle < PITCH_MIN_ANGLE_ECD)
	{
		gimbalPitchCtrl.wantedAbsoluteAngle = PITCH_MIN_ANGLE_ECD;
	}

        gimbal_PID_calc(&(gimbalYawCtrl.agl_pid),gimbalYawCtrl.nowAbsoluteAngle,gimbalYawCtrl.wantedAbsoluteAngle);  // 关乎旋转方向的PID控制器
        // 输出了所需旋转速度。

        //对速度进行一阶滤波
        #ifdef GIMBAL_FILTER_ON
        first_order_filter_cali(&(gimbalYawCtrl.spd_filter),(gimbalYawCtrl.agl_pid).out);
				
        #endif
				
//				if(count>500&&count<1000)
//				{
//						gimbalYawCtrl.spd_filter.out = 0.005;
//				}
//				if(count<500&&count>0)
//				{
//					gimbalYawCtrl.spd_filter.out = -0.005;
//				}
//				if(count == 1000)
//					count = 0;
//				++count;
				
				// LYL:限制死区
				if(gimbalPitchCtrl.wantedAbsoluteAngle-gimbalPitchCtrl.nowAbsoluteAngle < GIMBAL_DEAD_BOND && gimbalPitchCtrl.wantedAbsoluteAngle-gimbalPitchCtrl.nowAbsoluteAngle > -GIMBAL_DEAD_BOND ){
					gimbalPitchCtrl.wantedAbsoluteAngle = gimbalPitchCtrl.nowAbsoluteAngle;
				}
				if(gimbalYawCtrl.spd_filter.out-gimbalYawCtrl.radSpeed < YAW_DEAD_BOND && gimbalYawCtrl.spd_filter.out-gimbalYawCtrl.radSpeed > -YAW_DEAD_BOND ){
					gimbalYawCtrl.spd_filter.out = gimbalYawCtrl.radSpeed;
				}
				
        PID_calc(&(gimbalYawCtrl.spd_pid),gimbalYawCtrl.radSpeed,gimbalYawCtrl.spd_filter.out);   // 普通的速度控制环
        gimbalYawCtrl.giveVolt=gimbalYawCtrl.spd_pid.out;     //给电压
				if(gimbalYawCtrl.zeroVoltMark)
        {
           gimbalYawCtrl.giveVolt=0;
        }
				
	      gimbal_PID_calc(&(gimbalPitchCtrl.agl_pid),gimbalPitchCtrl.nowAbsoluteAngle,gimbalPitchCtrl.wantedAbsoluteAngle);  // 关乎旋转方向的PID控制器
        // 输出了所需旋转速度。

        //对速度进行一阶滤波
        #ifdef GIMBAL_FILTER_ON
        first_order_filter_cali(&(gimbalPitchCtrl.spd_filter),(gimbalPitchCtrl.agl_pid).out);
				
        #endif
				
        PID_calc(&(gimbalPitchCtrl.spd_pid),gimbalPitchCtrl.radSpeed,gimbalPitchCtrl.spd_filter.out);   // 普通的速度控制环
        gimbalPitchCtrl.giveVolt=gimbalPitchCtrl.spd_pid.out;     //给电压

				//usart_printf("%d\r\n",c->giveVolt);
        if(gimbalPitchCtrl.zeroVoltMark)
        {
           gimbalPitchCtrl.giveVolt=0;
        }
				
}

int16_t * getTriggerCurrentP(void); //从另一个文件获取拨弹轮电机的电流
void gimbal_task(void const *pvParameters)
{
    uint8_t i;
    osDelay(GIMBAL_TASK_INIT_TIME);
    initGimbalCtrls();
    robotMode=getRobotPresentMode();
    rc_p=get_remote_control_point();    //获取遥控器数据和NUC数据指针
    nuc_p=get_nuc_control_point();
    osDelay(GIMBAL_TASK_INIT_TIME);
    PID_clear(&(gimbalYawCtrl.spd_pid));
    PID_clear(&(gimbalYawCtrl.agl_pid));
    PID_clear(&(gimbalPitchCtrl.spd_pid));
    PID_clear(&(gimbalPitchCtrl.agl_pid));
    int16_t *triggerCurrentP=getTriggerCurrentP();//获取shoot_task源文件里计算的M2006的电流。
		lastMode=RobotState_e_Powerless;  //上一次初始化为无力
		gimbalYawCtrl.zeroVoltMark=0;
		gimbalPitchCtrl.zeroVoltMark=0;
		gimbalYawCtrl.wantedAbsoluteAngle = gimbalYawCtrl.nowAbsoluteAngle;
		gimbalPitchCtrl.wantedAbsoluteAngle = gimbalPitchCtrl.nowAbsoluteAngle;

    while(1)
    {
			  laser_on();
        for(i=0;i<2;i++)
            refreshAngleStates(gimbalCtrl[i]);   // 从传感器获得当前角度数据 //test done
        getControlAngles();                  // 从遥控器获得目标角度     //test half done
        for(i=0;i<2;i++)
        {
            monitorRounds(gimbalCtrl[i]);        // 监控转过的圈数           //test
        }
				limitAnglesSecond();
//				gimbalPitchCtrl.wantedAbsoluteAngle = 0.0f;
        calcPID();
        CAN_cmd_gimbal(gimbalYawCtrl.giveVolt,gimbalPitchCtrl.giveVolt,*triggerCurrentP,0);       
				//CAN_cmd_yaw(gimbalYawCtrl.giveVolt);
				
			// LYL：这个串口打印函数一放上电机就会卡	
		//usart_printf("%f,%f\n",gimbalYawCtrl.nowAbsoluteAngle,gimbalPitchCtrl.nowAbsoluteAngle);
//				usart_printf("%d\n",*robotMode);
				
				// LYL
//				usart_printf("now:%f,%f,%f\r\n",gimbalYawCtrl.nowAbsoluteAngle,gimbalPitchCtrl.nowAbsoluteAngle,gimbalPitchCtrl.wantedAbsoluteAngle);
				//usart_printf("%d\r\n", fricOn);
				
				osDelay(GIMBAL_TASK_CTRL_TIME);
				
//			usart_printf("%f,%f,%f,%f\n",nuc_p->yaw,nuc_p->pitch,nuc_p->is_fire,nuc_p->confidence);
    }
}
const struct GimbalToChassis_s * getBadYawWantedSpd(void)
{
    return &toChassis;
}

const struct gimbalMotorCtrl_s* get_wantedyaw_point()
{
	return &gimbalYawCtrl;
}

const int16_t * getGimbalNowRoundsPoint(void)
{
    return &(gimbalYawCtrl.nowRounds);
}
//to do :
// 角速度能力测试
// 