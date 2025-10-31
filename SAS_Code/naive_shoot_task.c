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

//ע�⣺��ң�����Ҳ���ָ�������ʱ�򣬲��������ٿ�

// #define ZERO_CURRENT_SAFE

//*************��������
#define SHOOT_TASK_INIT_TIME 500
#define SHOOT_CTRL_TIME 5



//**************���������Ƴ��� 15 0.40 0.24 0.53   1.0 0.6 1.72 1.0 18 0.45
//#define SHOOT_SPEED_LIMIT 0.525f        //Ħ�����ٶȡ�δ�����Բ���Ħ�����ٶȺ����ٵĹ�ϵ 0.60f 
double SHOOT_SPEED_LIMIT = 0.525;
bool_t is_x_pressed = 0;
#define SHOOT_TRIGGER_SPEED_LIMIT 0.5f   //�����ֿ���ʱ�ٶ�
#define READY_TRIGGER_SPEED 0.625f				//������׼���ٶ�



#define PRESS_LONG_TIME     200    

//**********����Ȧ���������
#define TRIGGER_ROUNDS_FOR_A_BULLET_A 4   
#define TRIGGER_ROUNDS_FOR_A_BULLET_B 4     
#define TRIGGER_ROUNDS_FOR_A_BULLET_C 4
#define ECD_FULL_ROUND 8192

//************ң�����ͼ�������
#define SHOOT_MODE_CHANNEL 1    //ң�����󲦸˿��Ʒ���
// #define MOUSE_SHOOT  ����������̧�𡪡����������²��š����������Ҽ����£�����NUC���Ʒ���       
#define KEY_FRIC_ON     KEY_PRESSED_OFFSET_SHIFT    //��shift����Ħ����

// #define TEAMER_ALLOW_SHOOT  //��������Ҽ����ţ�����nuc���Ʒ���

//*************����ٶȻ�PID
#define M3505_MOTOR_SPEED_PID_KP 60000.0f
#define M3505_MOTOR_SPEED_PID_KI 0.0f
#define M3505_MOTOR_SPEED_PID_KD 0.0f
//�ϲ����� 20000
#define M2006_MOTOR_SPEED_PID_KP 20000.0f

#define M2006_MOTOR_SPEED_PID_KI 0.0f   //��Ҫ��������������������ֹͣ�󻹻�ת
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

//*************************��̱�ջ���Ŀ��Ʋ���***********//
#define MILESTONE_NEAR_THRESHHOLD 2000
#define MILESTONE_NUMBER 3

//**********************�������***********************//
#define STUCK_TIME_LIMIT 1500   //���ڷ���һ����ģʽ�г���һ��ʱ�䲻�˳��󣬽��뿨�����ģʽ
#define SOLVING_TIME_LIMIT 500  //�ڳ��Կ������ģʽ��ͣ����ʱ�䡣



const static fp32 motor_speed_pid[3] = {M3505_MOTOR_SPEED_PID_KP, M3505_MOTOR_SPEED_PID_KI, M3505_MOTOR_SPEED_PID_KD};
const static fp32 trigger_motor_speed_pid[3] = {M2006_MOTOR_SPEED_PID_KP, M2006_MOTOR_SPEED_PID_KI, M2006_MOTOR_SPEED_PID_KD};

//***********�����ֵ��ģʽö������
enum TriggerMode_e{
    TriggerMode_e_Stop,         //ֹͣ����
    TriggerMode_e_ShootOne,     //����һ�����������ּ���ǰ�ĵ��ת4Ȧ����Ӧ��������תһ��
    TriggerMode_e_ShootMulti,   //�෢����ʵ�ǵȴ����뷢��һ����״̬
    TriggerMode_e_StuckSolve,   //���Խ������
};

//**********������ECDȦ�������
struct milestoneStack_s {
    uint8_t head;
    uint8_t stack[MILESTONE_NUMBER+1];
};

struct TriggerControl_s{
    struct milestoneStack_s mstack; //��̱�ջ
    const uint16_t * ECDPoint;           //���ECD����λ��
    uint16_t    initECD;                //��ʼECD
    uint16_t    nowECD;                 //���ڵ�ECD
    int16_t     nowRounds;              //����ת����Ȧ��
};


//**********����ָ��
struct InstructBuff_s{
    uint8_t turnOnFric  ; //Ϊ1ʱ��ʾָ���Ħ���֣�Ϊ0ʱ��ʾָ��ر�Ħ����
    uint8_t shootOne    ;
    uint8_t shootMulti  ;
};


//*******************ȫ�ֱ���********//
// Ħ���ֿ���
bool_t fricOn=0;
extern bool_t wantFricOn; // LYL
bool_t wantTriggerOn = 0; // LYL
bool_t mouseWantTriggerOn = 0;
bool_t keywantFricOn = 0;

// �����ֿ���
uint8_t triggerOn=0;
uint8_t reverse = 0;

static struct InstructBuff_s insBuff;   //ȫ�ַ����������ָ���
static enum TriggerMode_e triggerMode;  //�����ֵ��״̬
// ����̨�����п����ٶ�

static int zeroCurrentMark;    //������ʱ��ֱ�ӷ����������ʱ��
const static RC_ctrl_t *rc_p;
static const enum RobotState_e *robotMode;   //������ģʽ
static const toSTM32_t *nuc_p;  //NUC����λ�á�δ������ȫ�Զ�������ʱ��Ҫ

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
 * @brief ��ECD��ֵ�Ȼ�Ϊ(0,8191)��Χ
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

static void initTriggerECDRoundsMonitor()  //��ʼ��������Ȧ�����
{
    triggerCtrl.ECDPoint=&(get_trigger_motor_measure_point()->ecd);
    triggerCtrl.initECD=*(triggerCtrl.ECDPoint);
}   

//���NUC���Ʒ���
static void getInstructionAndBuff(void)
{
    static uint32_t pressTime=0;
    static uint8_t up=0,down=0;
    zeroCurrentMark=0;
            
    //ͨ����������Ħ���֣�Ҳ���ô�����������ң�����ľ��㹻�ˣ�����������ȷ��������Ħ���ְɣ�
    if(rc_p->key.v & KEY_FRIC_ON)
		{
			if(insBuff.turnOnFric==1)
				insBuff.turnOnFric=0;
		}
    //��ң�����󲦸�ָ�������ʱ���������ٿ�
    if(switch_is_mid(rc_p->rc.s[SHOOT_MODE_CHANNEL]))
    {
        if(rc_p->mouse.press_l) //�������
        {
            if(pressTime<PRESS_LONG_TIME)
                pressTime+=SHOOT_CTRL_TIME;
            else    //�Ѿ������㹻����ʱ���ˣ����Բ����������ˣ�
            {
                insBuff.shootMulti=1;
            }
        }
        else    //���̧��
        {
            if(pressTime<PRESS_LONG_TIME)   //���²��ܿ�̧�𣬷���һ��
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
        //ͨ���ϲ����²���ʵ��Ħ����״̬�Ŀ���
        if(up==1)   // ���ϵ��� �²�
        {
            if(fricOn)
            {
                // fricOn=0;
                // triggerOn=0;
                //Ϊ�˷�ֹ������ʹ�û���ָ����ơ���������ֹͣʱ���ܹر�Ħ���֡�
                insBuff.shootMulti=0;
                insBuff.shootOne=0;
                insBuff.turnOnFric=0;   //ϣ���ر�Ħ����
            }    
            else
            {
                // fricOn=1;        
                insBuff.turnOnFric=1;   //ϣ������Ħ����
            }
            up=0;
        }
        if(down==1)  // �ո����·������µ��е��ϲ�
        {
            insBuff.shootMulti=0;   //ֹͣ����
            if(pressTime<PRESS_LONG_TIME)   //���²��ܿ�̧�𣬷���һ��
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
        else    //�Ѿ������㹻����ʱ���ˣ����Բ����������ˣ�
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
    // ����ECD
    triggerCtrl.nowECD=*(triggerCtrl.ECDPoint);

    for(j=0;j<MILESTONE_NUMBER;j++)    //ö��ÿһ����̱�����λ��
    {
        fp32 relativeRealECD;
        relativeRealECD=ECDFormat((int16_t)triggerCtrl.nowECD-(int16_t)triggerCtrl.initECD);
        //ʧ��ԭ����0�ıȽϳ���������
        
        if(ECDFormat(relativeRealECD-j*ECD_FULL_ROUND/MILESTONE_NUMBER)<MILESTONE_NEAR_THRESHHOLD)
                //��ǰλ��������Ӧ��̱�������������
        {
            if(j!=(triggerCtrl.mstack.stack[triggerCtrl.mstack.head]))
               //����˵��������һ����λ�ã�������λ�ü���ջ��
            {
                triggerCtrl.mstack.head++;
                #ifdef WATCH_ARRAY_OUT
                if(c->mstack.head>=MILESTONE_NUMBER)
                {
                    itHappens();    //  ��usb task���������Խ����Ϣ
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
        {//������һȦ
            if(triggerCtrl.mstack.stack[1]==1)//������ת����ʱ�룩
                triggerCtrl.nowRounds +=1;
            else
                triggerCtrl.nowRounds -=1;
            triggerCtrl.mstack.head=0;     // ���ջ���ص���ʼΪ0��ʱ��
        }
    }
}
static void initShootModes(void)
{
    //��ʼ��ָ��
    insBuff.shootMulti=0;
    insBuff.shootOne=0;
    insBuff.turnOnFric=0;
    triggerMode=TriggerMode_e_Stop;
    fricOn=0;
    triggerOn=0;
}


static void fricModeChange(void)
{
    //�ȴ������ֹر�ʱ�ٹر�Ħ����
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
    static uint32_t shootOneEnterTime = 0;  //���뷢��һ����״̬��ʱ��
    // static uint32_t shootOneStayTime = 0;   // �ڷ���һ����״̬��ͣ����ʱ��
    static uint32_t solveStuckEnterTime = 0;   // �ڷ���һ����״̬��ͣ����ʱ��
    // static uint32_t solveStuckNowTime = 0;   // �ڳ��Խ������״̬�е�����ʱ�䣬ֱ��ʹ��nowTime ����
    
    static uint32_t nowTime=0;
    nowTime=HAL_GetTick();  //ÿ�ν���˺����������
    
    if(!fricOn) //Ħ���ֹر�״̬��ʼ���ò����ֹر�
    {
        triggerMode=TriggerMode_e_Stop;
        return;
    }

    //��ÿ��״̬ö��
    if(triggerMode==TriggerMode_e_Stop)
    {
        //�����˲���������״̬ʱ�����մ�stop״̬ת��Ϊ����״̬��ָ��
        if((*robotMode)!=RobotState_e_Powerless)
        {
            if(insBuff.shootOne)
            {
                triggerMode=TriggerMode_e_ShootOne;
                shootOneEnterTime = nowTime;
                insBuff.shootOne=0; //��������ָ��
            }
            else if (insBuff.shootMulti)
            {
                triggerMode=TriggerMode_e_ShootMulti;
                insBuff.shootMulti=0;   //��������ָ��
            }
                
        }
    }
    else if(triggerMode==TriggerMode_e_ShootOne)
    {
				//�Ľ�Ȧ��+ECD����se3333
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
        //�����������״̬

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
        wantedVShootMotor[0]=-SHOOT_SPEED_LIMIT;//ʵ�����ת�����򡣷���������ʵ�鿨��ʱ���˵�
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
        wantedVTriggerMotor=SHOOT_TRIGGER_SPEED_LIMIT;//�����ڽ��stuck(reverse)����������v/4���ٶ�ת��
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
	
    initShootPIDs();    //��ʼ��PID
    initTriggerECDRoundsMonitor();  //��ʼ��������Ȧ�����
    initShootModes();   //��ʼ������״̬
    // initWantedSpeed();  //��ʼ������ٶȺ͵��� ȫ�ֱ����Լ�����0��
    robotMode=getRobotPresentMode();
    rc_p=get_remote_control_point();    //��ȡң�������ݺ�NUC����ָ��
		nuc_p = get_nuc_control_point();
    // triggerMonitorSubOn=1;
    while(1)
    {
        getInstructionAndBuff();    //��ȡָ�����
        fricModeChange();           //Ħ����״̬�ı�
        //��Ħ���ֹر�״̬�����������������ָ��

        triggerModeChange();        //������״̬ת��
        setSpeedByMode();

        // monitorTriggerECDRound();        //��ز�����ECDȦ��
        getShootMotorSpeed();
			
			// LYL
//			usart_printf("%f,%f\r\n",presentVShootMotor[0],presentVShootMotor[1]);
			
        calcGiveCurrent1();                   //����PID
                                    //���Ϳ��Ƶ���
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
//    struct milestoneStack_s mstack; //��̱�ջ
//    const uint16_t * ECDPoint;           //���ECD����λ��
//    uint16_t    initECD;                //��ʼECD
//	  uint16_t    targetECD;              //Ŀ��ECD
//    uint16_t    nowECD;                 //���ڵ�ECD
//    int16_t     nowRounds;              //����ת����Ȧ��
//		int16_t     targetRounds;           //Ŀ��ת����Ȧ��
//		int16_t	    set_speed;								//Ŀ��ת��
//		int16_t*    now_speed;								//��ǰת��
//		enum M2006Mode mode;								//��ǰ2006��ģʽ
//		pid_type_def       pid;                    //�ٶ�PID
//	  pid_type_def       angle_pid;               //�Ƕ�PID
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
////�����̿�������

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
////Ϊ�˼�С�ӳ�ʹ�õ�trigger monitor��������shoot�������
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

