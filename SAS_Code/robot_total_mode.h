/**
 * @file robot_total_mode.h
 * @author ��׿ѫ ����� UCAS-SAS �й���ѧԺ��ѧ�����˶�
 * @brief ����ң��������жϲ��������˵���״̬�������Ƿ��Զ���������̨�����̡�����������������״̬�жϷ�״̬����֤һ����
 *          ���⣬��.c�ļ��ж����������¼robotAuto״̬��
 *
 * @version 0.2
 * @date 2022-02-05 �޸�Ϊ�ĸ�ģʽ����������ͨ����С���ݳ�����yaw��λ��
 *                                 ����    C       V        B
 *                                          ��      ��      ��
 * �����̿�ʼ���ƺ�ң�������˾���ʱʧȥ�����ˡ�������״̬�����ı䣬��ô������̿��Ʊ�ǣ�ң�������»�ø���״̬��Ȩ��
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef ROBOT_TOTAL_MODE_H
#define ROBOT_TOTAL_MODE_H
#include "struct_typedef.h"
//version 0.2
//shift ����Ħ���֣�ctrl�ر�Ħ����
// ��������һ�·���һ������ס�������Ҽ���ס����nuc���Ʒ��䣨nuc����ѡ�񵥷�����������
// cvb���벻ͬ����ģʽ
// z��������
// x�ر�����
// �������pitch�����ң���badyaw����������������ת������ͨ����С���ݳ�������̨����������ת
// ǰ����������֡�es����ws��
// ��̨��һ��ʱ���ڿ����޷�����ָ��λ�ã��ͷ�������
enum RobotState_e{
    RobotState_e_Powerless=0,     

    RobotState_e_CommonCar=3,     

    RobotState_e_GimbalCar=1,        
                                

    RobotState_e_Spinner=2,     
};

extern int robotIsAuto(void);  // �����ṩ�Ľӿڣ�����Ŀǰ�������Ƿ��Զ�
const enum RobotState_e * getRobotPresentMode(void);  //���ػ����˵�ǰ״ָ̬��
void robot_total_mode_task(void const *pvParameters);
bool_t get_fric(void);
#endif // !ROBOT_TOTAL_MODE_H

/*
�������������֮�����ϵ�ͱ��ʡ�

PID�͵�����������Ļ����йء��������Ŀ��ĵ�λ�ı���ı䡣




*/
