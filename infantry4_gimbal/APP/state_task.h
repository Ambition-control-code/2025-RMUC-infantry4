#ifndef __STATE_TASK_H
#define __STATE_TASK_H
#include "main.h"
#include "shoot_task.h"
void state_task(void const * argument);

void led_green_off(void);
void led_green_on(void);
void led_green_toggle(void);
void led_red_off(void);
void led_red_on(void);
void led_red_toggle(void);
void led_blue_off(void);
void led_blue_on(void);
void led_blue_toggle(void);
typedef  struct
{
    float input;        //输入数据
    float out;          //滤波输出的数据
    float num[1];       //滤波参数
    float frame_period; //滤波的时间间隔 单位 s
} first_filter_type;

void worke_out_to_chassis(void);
float chassis_sport_ramp_function(float target_speed,float acc_speed,float real_speed);
void worke_out_chassis_sport(void);
void link_dt7_vt13(void);
#endif