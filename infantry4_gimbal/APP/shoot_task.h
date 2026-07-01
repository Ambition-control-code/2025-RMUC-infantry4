#ifndef __SHOOT_TASK_H
#define __SHOOT_TASK_H
#include"main.h"
#define shoot_15_rate 15
#define super_speed 2000
#define single_speed 100
typedef struct
{
	float feed_set;//拨弹电机速度设定值
	float feed_real;//拨弹电机速度真实值
	float shootL_set;//左摩擦轮速度设定值
	float shootR_set;
	float shootL_real;//左摩擦轮速度真实值
	float shootR_real;
	float feed_out;
	float shootL_out;
	float shootR_out;
}weapon;
typedef enum
{
	CONTINUE,//连续
	SINGLE,//单发
	STOP,//停止
	ERR,//卡弹
}shoot_mode;
extern void all_pid_init(void);
extern void fireMotorPidReal_update();
extern void fireMotorSingleLoopPid_out_update(void);
extern void solve_motor_error(uint16_t*a);
extern void shoot_speed_ctrl(void);
void single_bullet(void);
void aim_shoot(void);
void aim_shoot_buff(void);
#endif