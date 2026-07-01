#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H
#include "struct_typedef.h"
void chassis_task(void);
typedef enum
  {  
		ROTATE,//小陀螺
		FOLLOW,
    NO_FOLLOW, 	 	
  }MODE_SWITCH;
typedef struct speed
{
	float speed_set[4];
//	float speed_set1;
//	float speed_set2;
//	float speed_set3;
	float A;
	float B;
	float C;
	float D;
	float location_set;
	float motor_out[4];
	float motor_limitout[4];
	float x_set;
	float y_set;
	
}motor_speed;//底盘电机速度期望值
typedef struct chassis
{
	float yaw_count;//yaw轴圈数
	float yaw_remainder;//yaw轴位置
	int8_t spin_flag;
	float rotate_speed;//旋转速度
	float follow_angle_set;
	int16_t head_mode;
	float angle_compensate;//跟随角度补偿
}chassis_sport;//底盘运动
void indep_task();
void follow_task();
void chassis_task(void);
void all_pid_init(void);
void chassis_mode(void);
void rotate(float num);
void chassis_speedloop(void);
void dynamic_solution(void);
float my_fabs(float num);

#endif

