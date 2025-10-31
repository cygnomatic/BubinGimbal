#ifndef SERVO_TASK_H
#define SERVO_TASK_H
#include "main.h"
#include "struct_typedef.h"

extern TIM_HandleTypeDef htim1;


extern void servo_task(void const * argument);
extern bool_t  get_servo_state(void);
#endif
