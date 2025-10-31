/**
 ******************************************************************************
 * @file    communication_task.c
 * @author  Hu Zijian
 * @version V2.0.0
 * @date    2024/3/10
 * @brief
 ******************************************************************************
 * @attention
 *  用来同上位机和底盘c版通信
 ******************************************************************************
 */
 
#include "communication_task.h"
#include "main.h"
#include "CAN_receive.h"
#include "nucCommu.h"
#include "INS_task.h"
#include "usart.h"
#include "bsp_usart.h"
#include "cmsis_os.h"
#include "CAN_receive.h"
#include "robot_total_mode.h"


//******************全局变量*******************
static uint8_t GimbalAngleMsg[40];


void communication_task(void const *pvParameters){
		int count = 0;
    while (1)
    {
				//以200帧的速率发送云台信息
				if(count % 5  ==  0)
				{
						Encode1(GimbalAngleMsg,get_INS()->Yaw,-get_INS()->Pitch,get_INS()->Roll);
						HAL_UART_Transmit(&huart1, GimbalAngleMsg, 17, 100);
						usart1_tx_dma_enable(GimbalAngleMsg, 17);
				}
				if(count % 100 == 0)
				{
					//usart_printf("%d", get_refree_point()->robot_id);
					Encode2(GimbalAngleMsg,get_refree_point()->robot_id>100?0:1,get_refree_point()->robot_id>100?get_refree_point()->robot_id-100:get_refree_point()->robot_id,1,robotIsAuto());
						HAL_UART_Transmit(&huart1, GimbalAngleMsg, 9, 100);
						usart1_tx_dma_enable(GimbalAngleMsg, 9);
				}
				if(count % 1 ==0)
//				CAN1_send_yaw();
					CAN1_send_channel();
				count++;
				if(count>1000)
					count = 0;
				osDelay(1);
		}
}
