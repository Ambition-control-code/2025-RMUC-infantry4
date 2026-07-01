#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H
#include"main.h"
typedef enum
{
	COMMON=0,
  AIM,
	BUFF,
	WEA_COM
}gimbal_contion;


typedef struct
{
   float yaw_set;
	 float pitch_set;//角度环设定值
	 
   float yaw_out;
	 float pitch_out;//速度环设定值
	
	 float yaw_position_value;
	 float pitch_position_value;
	
	 float auto_yaw;
	 float auto_pitch;
}gimbal_sport;

typedef struct {
    float target_speed;     // 目标速度
    float current_speed;    // 当前指令速度
    float max_accel_step;   // 最大加速步长（初始值）
    float max_decel_step;   // 最大减速步长（初始值）
    float min_step;         // 最小步长（防止步长过小）
    float adapt_gain;       // 自适应系数（调节灵敏度）
} Ramp_Controller;

void Ramp_Init(Ramp_Controller *ramp, float max_accel, float max_decel, float min_step,float adapt_gain, float target_speed);
void Ramp_Update(Ramp_Controller *ramp);

void gimbal_pid_init(void);
void gimbal_mode(float yaw_set,float pitch_set);
void gimbal_set_update(void);
void gimbal_angle_return_init(void);
void send_data_to_chassis(int sl,int sr);
float my_fabs(float num);
#endif

