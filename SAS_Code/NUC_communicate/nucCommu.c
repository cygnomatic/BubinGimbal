#include "nucCommu.h"
#include "main.h"
#include "bsp_nucCom.h"
// #include "monitor_task.h"
#include "struct_typedef.h"
#include "detect_task.h"
#include "OLED.h"
#include <stdlib.h>
#include <string.h>
#include "bsp_buzzer.h"
#include "usb_task.h"
#include "CRC8_CRC16.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

static void sbus_to_nucCtrl(volatile const uint8_t *sbus_buf, toSTM32_t *nuc_info);

// nuc control data
// nuc控制变量
// 以通过测试，输入输出无bug
toSTM32_t GimbalRxMsg;

// added at 2022年1月26日23点54分：NUC有效时间
static uint32_t lastValidTime = 0;
static int dataIsValid = 0;
static int buzzer_time;
// added at 2022年1月26日23点54分：NUC有效时间 控制函数
void setNUCValid(void)
{
    dataIsValid = 1;
    return;
}

void setNUCInvalid(void)
{
    dataIsValid = 0;
}

uint32_t getNUCLastValidTime_ms(void)
{
    return lastValidTime;
}

int NUCIsValid(void)
{
    return dataIsValid;
}

// receive data, 44 bytes one frame, but set 88 bytes
// 接收原始数据，为44个字节，给了88个字节长度，防止DMA传输越界
static uint8_t nucinfo_rx_buf[2][NUCINFO_RX_BUF_NUM];
// 提供高层次封装，避免直接引用较低层次头文件
void nuc_control_init(void)
{
    NUC_com_init(nucinfo_rx_buf[0], nucinfo_rx_buf[1], NUCINFO_RX_BUF_NUM);
}

/**
 * @brief          获取nuc数据指针
 * @param[in]      none
 * @retval         nuc数据指针
 */
const toSTM32_t *get_nuc_control_point(void)
{
    return &GimbalRxMsg;
}
////先发指令1，再发指令2
//void Encode(uint8_t* RawData, fp64 gimbal_yaw, fp64 gimbal_pitch , fp64 gimbal_roll, enum Robot_Colors self_color, enum Robot_ID self_id,  enum Auto_Num auto_num,enum Attack_mode attack_mode)
//{
//    RawData[0] = 0xE7;
//    RawData[1] = 0x7E;

//    // 发送任务码 1：云台数据
//    RawData[2] = 0x01;
//    memcpy(&RawData[3], &gimbal_yaw, sizeof(fp64));
//    memcpy(&RawData[11], &gimbal_pitch, sizeof(fp64));
//    memcpy(&RawData[19], &gimbal_roll, sizeof(fp64));

//    // 添加CRC校验码
//    append_CRC16_check_sum(RawData, 29);

//    // 发送任务码 2：机器人相关数据
//    RawData[29] = 0xE7;
//    RawData[30] = 0x7E;
//    RawData[31] = 0x02;
//    uint8_t id = (uint8_t) self_id;
//    uint8_t color = (uint8_t) self_color;
//    uint8_t A_num = (uint8_t) auto_num;
//    uint8_t A_mode = (uint8_t) attack_mode;
//    memcpy(&RawData[32], &color, sizeof(uint8_t));
//    memcpy(&RawData[33], &id, sizeof(uint8_t));
//    memcpy(&RawData[34], &A_num, sizeof(uint8_t));
//    memcpy(&RawData[35], &A_mode, sizeof(uint8_t));

//    // 添加CRC校验码
//    append_CRC16_check_sum(&RawData[29], 9);
//}

// 发送任务码 1：云台数据 3+12+2 = 17
void Encode1(uint8_t* RawData, fp32 gimbal_yaw, fp32 gimbal_pitch , fp32 gimbal_roll) {
    RawData[0] = 0xE7;
    RawData[1] = 0x7E;
    RawData[2] = 0x01;
    memcpy(&RawData[3], &gimbal_yaw, sizeof(fp32));
    memcpy(&RawData[7], &gimbal_pitch, sizeof(fp32));
    memcpy(&RawData[11], &gimbal_roll, sizeof(fp32));
    append_CRC16_check_sum(RawData, 17);
}

// 发送任务码 2：机器人相关数据 3+4+2=9
void Encode2(uint8_t* RawData, enum Robot_Colors self_color, enum Robot_ID self_id, enum Auto_Num auto_num, enum Attack_mode attack_mode) {
    RawData[0] = 0xE7;
    RawData[1] = 0x7E;
    RawData[2] = 0x02;
    uint8_t id = (uint8_t)self_id;
    uint8_t color = (uint8_t)self_color;
    uint8_t A_num = (uint8_t)auto_num;
    uint8_t A_mode = (uint8_t)attack_mode;
    memcpy(&RawData[3], &color, sizeof(uint8_t));
    memcpy(&RawData[4], &id, sizeof(uint8_t));
    memcpy(&RawData[5], &A_num, sizeof(uint8_t));
    memcpy(&RawData[6], &A_mode, sizeof(uint8_t));
    append_CRC16_check_sum(RawData, 9);
}



// 串口中断
extern TIM_HandleTypeDef htim5;
void USART1_IRQHandler(void)
{
    if (huart1.Instance->SR & UART_FLAG_RXNE) // 接收到数据//接受中断
    {
        __HAL_UART_CLEAR_PEFLAG(&huart1);
    }
    else if (USART1->SR & UART_FLAG_IDLE) // 空闲中断
    {
        static uint16_t this_time_rx_len = 0;

        __HAL_UART_CLEAR_PEFLAG(&huart1);

        if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET)
        {
            /* Current memory buffer used is Memory 0 */

            // disable DMA
            // 失效DMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            // get receive data length, length = set_data_length - remain_length
            // 获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = NUCINFO_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

            // reset set_data_lenght
            // 重新设定数据长度
            hdma_usart1_rx.Instance->NDTR = NUCINFO_RX_BUF_NUM;

            // set memory buffer 1
            // 设定缓冲1
            hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;

            // enable DMA
            // 使能DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            if (this_time_rx_len == NUCINFO_FRAME_LENGTH)
            {
                sbus_to_nucCtrl(nucinfo_rx_buf[0], &GimbalRxMsg);
//							usart_printf("%d,%f,%f,%f\n",GimbalRxMsg.is_fire,GimbalRxMsg.confidence,GimbalRxMsg.pitch,GimbalRxMsg.yaw);
                //usart_printf("%d,%d,%d,%f,%f,%f,%f\r\n",GimbalRxMsg.is_fire,GimbalRxMsg.is_spin,GimbalRxMsg.recognized,GimbalRxMsg.yaw.data,GimbalRxMsg.pitch.data,GimbalRxMsg.velocity_x.data,GimbalRxMsg.velocity_y.data);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            // disable DMA
            // 失效DMA
            __HAL_DMA_DISABLE(&hdma_usart1_rx);

            // get receive data length, length = set_data_length - remain_length
            // 获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = NUCINFO_RX_BUF_NUM - hdma_usart1_rx.Instance->NDTR;

            // reset set_data_lenght
            // 重新设定数据长度
            hdma_usart1_rx.Instance->NDTR = NUCINFO_RX_BUF_NUM;

            // set memory buffer 0
            // 设定缓冲??0
            DMA2_Stream5->CR &= ~(DMA_SxCR_CT);

            // enable DMA
            // 使能DMA
            __HAL_DMA_ENABLE(&hdma_usart1_rx);

            if (this_time_rx_len == NUCINFO_FRAME_LENGTH)
            {
                sbus_to_nucCtrl(nucinfo_rx_buf[1], &GimbalRxMsg);
//                usart_printf("%d,%f,%f,%f\n",GimbalRxMsg.is_fire,GimbalRxMsg.confidence,GimbalRxMsg.pitch,GimbalRxMsg.yaw);
            }
        }
    }
}

/**
 * @brief          nuc数据协议解析
 * @param[in]      sbus_buf: 原生数据指针
 * @param[out]     rc_ctrl: nuc数据指针
 * @retval         none
 */

//17个字节 2+4+4+1+4+2=17 
static void sbus_to_nucCtrl(volatile const uint8_t *sbus_buf, toSTM32_t *nuc_ctrl)
{
    if (sbus_buf[0] == SOF1 && sbus_buf[1] == SOF2 && verify_CRC16_check_sum((unsigned char*)sbus_buf,17))
    {
				memcpy(&(nuc_ctrl->yaw),(const void*)&sbus_buf[2],4);
        memcpy(&(nuc_ctrl->pitch),(const void*)&sbus_buf[6],4);
        memcpy(&(nuc_ctrl->is_fire),(const void*)&sbus_buf[10],1);
			  memcpy(&(nuc_ctrl->confidence),(const void*)&sbus_buf[11],4);

//			usart_printf("%x\n",sbus_buf);
//			for(int i=0;i<17;i++) {
//				usart_printf("%x ", sbus_buf[i]);
//			}
//			usart_printf("\n");
//			for(int i=0;i<4;i++){
//			 usart_printf("%x ", ((unsigned char *)(&nuc_ctrl->yaw))[i]);
//			}
//			usart_printf("\n");
//			for(int i=0;i<4;i++){
//			 usart_printf("%x ", ((unsigned char *)(&nuc_ctrl->pitch))[i]);
//			}
//			usart_printf("\n");
//			for(int i=0;i<1;i++){
//			 usart_printf("%x ", ((unsigned char *)(&nuc_ctrl->is_fire))[i]);
//			}
//			usart_printf("\n");
//			for(int i=0;i<4;i++){
//			 usart_printf("%x ", ((unsigned char *)(&nuc_ctrl->confidence))[i]);
//			}
//			usart_printf("\n");
				//usart_printf("want:%f,%f,%d,%f\r\n",nuc_ctrl->yaw.data,nuc_ctrl->pitch.data,nuc_ctrl->is_fire,nuc_ctrl->confidence.data);
//			usart_printf("-------------------------\n");
    }
}

// 找到了返回1，未找到则返回0.当发送的数据起始为'S'则为找到
// int foundArmor(void)
//{
//     if(toe_is_error(MINI_PC_TOE))
//         return 0;
//     if(((char)(nuc_ctrl.startFlag))==SEND_START)
//         return 1;
//     else
//         return 0;
// }

// void OLED_nuc(void)
//{
//     OLED_printf(0,0,"a:%d,f:%d,pitch:%f,yaw:%f",robotIsAuto(),foundArmor(),GimbalRxMsg.pitch,GimbalRxMsg.yaw);
//     //东西太多，发过去会闪
//     // OLED_printf(0,0,"s:%c,task:%d,tht:%f,phi:%f,r:%f,color:%d,end:%c,auto:%d",
//     // (char)(nuc_ctrl.startFlag),nuc_ctrl.taskType,nuc_ctrl.theta,nuc_ctrl.phi,nuc_ctrl.r,nuc_ctrl.detectedColor,(char)(nuc_ctrl.endFlag),robotIsAuto());
// }

// void OLED_nuc(void)
// {
//     OLED_printf(0,0,"s:%c,task:%d,tht:%.1f,phi%.1f",
//     (char)(nuc_ctrl.startFlag),nuc_ctrl.taskType,nuc_ctrl.theta,nuc_ctrl.phi);
// }