#ifndef STATE_TASK_H
#define STATE_TASK_H
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
void STATE_TASK(void const * argument);
float chassis_sport_ramp_function(float target_speed,float acc_speed,float real_speed);
void worke_out_chassis_sport(void);
void limit_ctrl_data(void);
#endif

