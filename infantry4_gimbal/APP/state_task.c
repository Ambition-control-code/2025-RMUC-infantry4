#include "state_task.h"
#include "cmsis_os.h"
#include "gimbal_task.h"
#include "vision_task.h"
#include "remote_control.h"
#include "tele_link.h"
#include "bsp_bull_ctr.h"
int aim_flag=0;//自瞄
extern RC_ctrl_t rc_ctrl;
extern gimbal_contion gimbal_state;
extern uint8_t rc_gim_mode;
extern int aim_ok_flag;
extern VT13_data use_data;
extern VT13_ctrl_t VT13_ctrl;
float  chassis_vx_set;
float chassis_vy_set;
float out_x_set;
float out_y_set;
int chassis_mode;
int rotate_flag,unrotate_flag;
uint8_t feed_frequency=1;
int8_t vision_flag;
int8_t spin_flag;
int8_t spin_flags=0;
uint8_t vt13_moca,vt13_aim,vt13_buff;
void state_task(void const * argument)
{
  /* USER CODE BEGIN state_task */
  /* Infinite loop */
  for(;;)
  {
		work_out_init();
		bullet_fre_ctrl();
		/*右键开启自瞄*/ //自瞄模块/
		if(rc_ctrl.L_mouse_r==1)gimbal_state = vt13_aim = 1;
		else if(rc_ctrl.L_mouse_r!=1)gimbal_state = vt13_aim = 0;
		//底盘模式改变
/*********************chassis:40,rotate_normal(速度优先);50,follow_normal（普通跟随）;6,no_power;**************************/
/*********************chassis:41,rotate_strong（转速优先）;6,no_power;**************************/
		if(rc_ctrl.L_Q ==1||rc_ctrl.L_R==1||rc_ctrl.L_E == 1){spin_flags=1;}
		if(rc_ctrl.sl==1||rc_ctrl.L_Q==1){spin_flag = 1;}//rotate//速度优先
		else if (rc_ctrl.L_R==1){spin_flag = 2 ;}//转速优先
		else if((rc_ctrl.sl==3&&spin_flags==0)||rc_ctrl.L_E==1){spin_flag = 0;}//follow
		//else if(rc_ctrl.sl==2){chassis_mode=6;}//no_power
		if(spin_flag==0){chassis_mode = 5;}
		else if(spin_flag == 1){chassis_mode = 40;}
		else if (spin_flag == 2){chassis_mode = 41;}
		//if(rc_ctrl.sl==2&&spin_flags==0){chassis_mode=6;}
		if(rc_ctrl.L_V==1){chassis_mode=43;}
		//拨弹盘模式改变//1低2高
		if(rc_ctrl.L_F == 1)feed_frequency = 1;
		else if(rc_ctrl.L_G == 1)feed_frequency = 2;
		//云台和超电模式
		if(gimbal_state==BUFF&&rc_ctrl.L_SHIFT==1)rc_gim_mode=0x11;//BUFF+super
		else if(gimbal_state==BUFF&&rc_ctrl.L_SHIFT==0)rc_gim_mode=0x10;//BUFF
		else if (gimbal_state==COMMON&&rc_ctrl.L_SHIFT==1)rc_gim_mode=0x21;//COMMON+super
		else if(gimbal_state==COMMON&&rc_ctrl.L_SHIFT==0)rc_gim_mode=0x20;//COMMON
		else if(gimbal_state==AIM&&rc_ctrl.L_SHIFT==1)rc_gim_mode=0x31;//AIM+super
		else if(gimbal_state==AIM&&rc_ctrl.L_SHIFT==0)rc_gim_mode=0x30;//AIM
		//if(rc_ctrl.off_line_flag==1)rc_gim_mode=4;//rc_off_line
		//自瞄及拨弹频率/***********************************************************************/
		if((aim_ok_flag==-1||aim_ok_flag==0)&&feed_frequency == 1)vision_flag=10;//自瞄未识别，拨弹频率为低
		else if((aim_ok_flag==-1||aim_ok_flag==0)&&feed_frequency == 2)vision_flag=20;//自瞄未识别，拨弹频率为高
		else if(((aim_ok_flag !=-1)&&(aim_ok_flag !=0))&&feed_frequency == 1)vision_flag=11;//自瞄识别，拨弹频率为低
		else if(((aim_ok_flag !=-1)&&(aim_ok_flag !=0))&&feed_frequency == 2)vision_flag=21;//自瞄识别，拨弹频率为高
		if((aim_ok_flag !=-1)&&(aim_ok_flag !=0)&&rc_ctrl.L_mouse_l==1&&feed_frequency == 1)vision_flag=12;//自瞄识别，允许自动开火，拨弹频率为低
		else if((aim_ok_flag !=-1)&&(aim_ok_flag !=0)&&rc_ctrl.L_mouse_l==1&&feed_frequency == 2)vision_flag=22;//自瞄识别，允许自动开火，拨弹频率为高
		
		 if(xTaskGetTickCount()%500==0)
		 {
			 if(rc_ctrl.last_cnts==rc_ctrl.cnts)
			 {
				 rc_ctrl.off_line_flag=1;
			 }
			 else{
				 rc_ctrl.off_line_flag=0;
			 }
			 rc_ctrl.last_cnts=rc_ctrl.cnts;//遥控器离线检测，离线c板亮红灯，在线亮绿灯
			 rc_ctrl.last_lun = rc_ctrl.lun;
		 }
		 if(VT13_ctrl.pause==1)
		 {
				vt13_aim = 1;
		 }
		 else
		 {
		   vt13_aim = 0;
		 }
	     osDelay(1);
  }
}

void worke_out_chassis_sport(void)
{
		if(rc_ctrl.L_W == 1 && rc_ctrl.L_S == 0 && rc_ctrl.L_SHIFT == 0)
			{
				chassis_vy_set=chassis_sport_ramp_function(-660,1.32,chassis_vy_set);
			}
		else if(rc_ctrl.L_W == 0 && rc_ctrl.L_S == 1 && rc_ctrl.L_SHIFT ==0)
			{
				chassis_vy_set=chassis_sport_ramp_function(660,1.32,chassis_vy_set);
			}
		else if(rc_ctrl.L_W == 1 && rc_ctrl.L_S == 0 && rc_ctrl.L_SHIFT == 1)
			{	
				chassis_vy_set=chassis_sport_ramp_function(-660,2.2,chassis_vy_set);
			}
		else if(rc_ctrl.L_W == 0 && rc_ctrl.L_S == 1 && rc_ctrl.L_SHIFT ==1)
			{
				chassis_vy_set=chassis_sport_ramp_function(660,2.2,chassis_vy_set);
			}
		else if(rc_ctrl.L_W == 0 && rc_ctrl.L_S == 0&&rc_ctrl.R_x==0&&rc_ctrl.R_y==0)
			{
				chassis_vy_set=0;
			}
		if(rc_ctrl.L_A == 1 && rc_ctrl.L_D == 0 && rc_ctrl.L_SHIFT == 0)
		  {	
				chassis_vx_set=chassis_sport_ramp_function(660,1.32,chassis_vx_set);
		  }
		else if(rc_ctrl.L_A == 0 && rc_ctrl.L_D == 1 && rc_ctrl.L_SHIFT == 0)
			{
				chassis_vx_set=chassis_sport_ramp_function(-660,1.32,chassis_vx_set);
			}
		else if(rc_ctrl.L_A == 1 && rc_ctrl.L_D == 0 && rc_ctrl.L_SHIFT == 1)
			{
				chassis_vx_set=chassis_sport_ramp_function(-660,2.2,chassis_vx_set);
			}
		else if(rc_ctrl.L_A == 0 && rc_ctrl.L_D == 1 && rc_ctrl.L_SHIFT== 1)
			{
				chassis_vx_set=chassis_sport_ramp_function(-660,2.2,chassis_vx_set);
			}
		else if(rc_ctrl.L_A == 0 && rc_ctrl.L_D == 0&&rc_ctrl.R_x==0&&rc_ctrl.R_y==0)
			{
				chassis_vx_set=0;
			}
		out_x_set=	chassis_vx_set+ rc_ctrl.R_x+VT13_ctrl.R_x;
		out_y_set=	chassis_vy_set+ rc_ctrl.R_y+VT13_ctrl.R_y;
/******************************************************************/	
}

//float target_speed;//斜坡规划目标速度
//float acc_speed;//斜坡规划加速度
//uint8_t plan_priority;//0目标优先，1实际优先
//acc_speed=1.32
float chassis_sport_ramp_function(float target_speed,float acc_speed,float real_speed)
{
	float out_speed = 0;
	float last_real_speed;
	if(target_speed>0)
	{
		if(real_speed<0){acc_speed = ((target_speed - real_speed)/target_speed)*acc_speed;}
		real_speed += acc_speed;
			if(real_speed > target_speed)real_speed = target_speed;
		out_speed = real_speed;
	}
	if(target_speed<0)
	{
		if(real_speed>0){{acc_speed = ((target_speed - real_speed)/target_speed)*acc_speed;}}
	  real_speed -= acc_speed;
		if(real_speed < target_speed)real_speed = target_speed;
		out_speed = real_speed;
	}
	if(target_speed==0)
	{
		if(real_speed > 0)
		{  
		  real_speed -= acc_speed;
		}
		if(real_speed < 0)real_speed += acc_speed;
		{
			real_speed += acc_speed;
		}
		if((last_real_speed>0&&real_speed<0)||(last_real_speed<0&&real_speed>0))real_speed=0;
		out_speed = real_speed;
	}
	return out_speed;
}

void link_dt7_vt13()
{
	rc_ctrl.L_A = rc_ctrl.A|VT13_ctrl.A;
	rc_ctrl.L_B = rc_ctrl.B|VT13_ctrl.B;
	rc_ctrl.L_C = rc_ctrl.C|VT13_ctrl.C;
	rc_ctrl.L_CTRL = rc_ctrl.Ctrl|VT13_ctrl.Ctrl;
	rc_ctrl.L_D = rc_ctrl.D|VT13_ctrl.D;
	rc_ctrl.L_E = rc_ctrl.E|VT13_ctrl.E;
	rc_ctrl.L_F = rc_ctrl.F|VT13_ctrl.F;
	rc_ctrl.L_G = rc_ctrl.G|VT13_ctrl.G;
	rc_ctrl.L_Q = rc_ctrl.Q|VT13_ctrl.Q;
	rc_ctrl.L_R = rc_ctrl.R|VT13_ctrl.R;
	rc_ctrl.L_S = rc_ctrl.S|VT13_ctrl.S;
	rc_ctrl.L_SHIFT = rc_ctrl.Shift|VT13_ctrl.Shift;
	rc_ctrl.L_V = rc_ctrl.V|VT13_ctrl.V;
	rc_ctrl.L_W = rc_ctrl.W|VT13_ctrl.W;
	rc_ctrl.L_X = rc_ctrl.X|VT13_ctrl.X;
	rc_ctrl.L_Z = rc_ctrl.Z|VT13_ctrl.Z;
	rc_ctrl.L_mouse_l = rc_ctrl.mouse_l|VT13_ctrl.mouse_l;
	rc_ctrl.L_mouse_r = rc_ctrl.mouse_r|VT13_ctrl.mouse_r;
}






