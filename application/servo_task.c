/**
 ******************************************************************************
 * @file    servo.c
 * @author  Hu Zijian
 * @version V1.0.0
 * @date    2024/3/27
 * @brief   舵机任务
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "servo_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_servo_pwm.h"
#include "remote_control.h"

#define SERVO_MIN_PWM   1000
#define SERVO_MAX_PWM   1850

servo_t servo;
const RC_ctrl_t* servo_rc;
/**
  * @brief          servo_task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          舵机任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void servo_init(void)
{
	HAL_TIM_Base_Start(&htim1);
	HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
	servo.id = 0;
	servo.htim = htim1;
	servo.channel = TIM_CHANNEL_1;
}

bool_t servo_open = 1;
void servo_task(void const * argument)
{
    servo_rc = get_remote_control_point();
		servo_init();
    while(1)
    {
			//打开
				if(servo_rc->key.v & KEY_PRESSED_OFFSET_Z)
				{
					servo_open = 1;
					single_servo_ctrl(&servo,SERVO_MIN_PWM);
					osDelay(500);
				}
			//关上
        if(servo_rc->key.v & KEY_PRESSED_OFFSET_X)
				{
					servo_open = 0;
					single_servo_ctrl(&servo,SERVO_MAX_PWM);
					osDelay(500);
				}
        osDelay(20);
    }
}

bool_t get_servo_state(void)
{
		return servo_open;
}


