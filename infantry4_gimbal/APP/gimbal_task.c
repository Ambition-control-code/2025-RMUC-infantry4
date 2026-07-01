 #include "gimbal_task.h"
#include "freertos.h"
#include "cmsis_os.h"
#include "queue.h"
#include "vision_task.h"
#include "shoot_task.h"
#include "state_task.h"
#include "pid.h"
#include "bsp_tr.h"
#include "remote_control.h"
#include "tele_link.h"
#include "imu_task.h"
#include "math.h"
extern DMA_HandleTypeDef hdma_usart3_rx;
extern bucket_imu imu;
extern motor_measure_t Pitch_Motor;//pitch电机
extern RC_ctrl_t rc_ctrl;
extern motor_measure_t Yaw_Motor;//yaw电机
extern int rc_ok_flag;
extern receive_struct receive_vision;
extern motor_measure_t	FrictioL_Motor;//左摩擦轮
extern motor_measure_t	FrictioR_Motor;//右摩擦轮
extern motor_measure_t	Feed_Motor;//供弹电机
extern weapon gimbal;//武器结构体变量
extern int fire_ok;//允许自瞄开火标志位
extern int aim_flag;//开启自瞄标志位
extern int aim_ok_flag;//自瞄锁定标志位
Ramp_Controller chassis_x;//底盘x轴运动斜坡函数
Ramp_Controller chassis_y;//底盘y轴运动斜坡函数
float  pitch_init=0.0f;
float  yaw_init=1411;
PID pitch_speedloop; 			PID pitch_positionloop;
PID yaw_speedloop;   			PID yaw_positionloop;
PID auto_pitch_speedloop; PID auto_pitch_positionloop;
PID auto_yaw_speedloop;   PID auto_yaw_positionloop;
PID buff_pitch_speedloop; PID buff_pitch_positionloop;
PID buff_yaw_speedloop;   PID buff_yaw_positionloop;
gimbal_contion gimbal_state;
gimbal_sport gimbal_motor;
float send_usage;
uint8_t rc_gim_mode;
extern PID shootL_speed;//摩擦轮电机pid结构体
extern PID shootR_speed;
extern PID feed_speed;
extern int chassis_mode;
extern float chassis_vx_set;
extern float chassis_vy_set;
extern float out_x_set;
extern float out_y_set;
extern int8_t vision_flag;
extern float fabs_differ;
extern float auto_yaw_count;
extern float yaw_accel,pitch_accel;
extern VT13_ctrl_t VT13_ctrl;
extern uint8_t vt13_moca,vt13_aim,vt13_buff;
double diff_yaw=0;float diff_pitch=0,cur_yaw=0,cur_pitch=0,last_diff_yaw=0,temp_cur_yaw=0;
//chassis:4,rotate;5,follow;6,no_power;
/************************************************************************************/
u8 temp_of_one;
void gimbal_task(void const * argument)
{
	gimbal_pid_init();//pid初始化
	for(;;)
	{
		link_dt7_vt13();
		if(vt13_aim == 1||VT13_ctrl.mouse_r==1)
		{
			gimbal_state = AIM;
		}
		else if(VT13_ctrl.C == 1)
		{
			gimbal_state = BUFF;
		}
		else 
		{
			gimbal_state = COMMON;
		}
		gimbal_set_update();//角度设定值更新
		gimbal_mode(gimbal_motor.yaw_set,gimbal_motor.pitch_set);//pitch输出值计算
		worke_out_chassis_sport();
		if(VT13_ctrl.mode==0)
		{send_to_gimbal_motor(0,0);}
		else{
		send_to_gimbal_motor(gimbal_motor.yaw_out,gimbal_motor.pitch_out);}//发送电流给云台电机
		osDelay(1);
	}
}

/**
  * @brief         云台模式切换
  * @author        Z&S
  * @param[in]     yaw目标值
  * @param[in]     pitch目标值
  * @retval        当前云台模式下pid计算输出值
  */
void gimbal_mode(float yaw_set,float pitch_set)
{
	if(gimbal_state == COMMON||gimbal_state == WEA_COM)//手动
	{
		gimbal_motor.yaw_position_value = pid_positionloop(&yaw_positionloop,imu.yaw[2],yaw_set);
		gimbal_motor.yaw_out = pid_speedloop(&yaw_speedloop,yaw_accel,gimbal_motor.yaw_position_value);
		gimbal_motor.pitch_position_value = pid_positionloop(&pitch_positionloop,imu.pitch,pitch_set);
		gimbal_motor.pitch_out = pid_speedloop(&pitch_speedloop,pitch_accel,gimbal_motor.pitch_position_value);
	}
	else if(gimbal_state == AIM)//自瞄
	{
		if(aim_ok_flag==-1||aim_ok_flag==0)
		{
			gimbal_motor.yaw_position_value = pid_positionloop(&yaw_positionloop,imu.yaw[2],gimbal_motor.yaw_set);
			gimbal_motor.yaw_out = pid_speedloop(&yaw_speedloop,yaw_accel,gimbal_motor.yaw_position_value);
			gimbal_motor.pitch_position_value = pid_positionloop(&pitch_positionloop,imu.pitch,pitch_set);
			gimbal_motor.pitch_out = pid_speedloop(&pitch_speedloop,pitch_accel,gimbal_motor.pitch_position_value);

		}
		else
		{
			
				while(gimbal_motor.yaw_set - imu.yaw[2] > 300)
				{gimbal_motor.yaw_set -= 360;}
				while(gimbal_motor.yaw_set - imu.yaw[2] < -300)
				{gimbal_motor.yaw_set += 360;}
			
			gimbal_motor.yaw_position_value = pid_positionloop(&auto_yaw_positionloop,imu.yaw[2],gimbal_motor.yaw_set);
			gimbal_motor.yaw_out = pid_speedloop(&auto_yaw_speedloop,yaw_accel,gimbal_motor.yaw_position_value);
			gimbal_motor.pitch_position_value = pid_positionloop(&auto_pitch_positionloop,imu.pitch,pitch_set);
			gimbal_motor.pitch_out = pid_speedloop(&auto_pitch_speedloop,pitch_accel,gimbal_motor.pitch_position_value);

			aim_shoot();
		}
	}
	else if(gimbal_state == BUFF)//打符//
	{
		if(aim_ok_flag==-1||aim_ok_flag==0)
		{
			gimbal_motor.yaw_position_value = pid_positionloop(&yaw_positionloop,imu.yaw[2],gimbal_motor.yaw_set);
			gimbal_motor.yaw_out = pid_speedloop(&yaw_speedloop,yaw_accel,gimbal_motor.yaw_position_value);
			gimbal_motor.pitch_position_value = pid_positionloop(&pitch_positionloop,imu.pitch,pitch_set);
			gimbal_motor.pitch_out = pid_speedloop(&pitch_speedloop,pitch_accel,gimbal_motor.pitch_position_value);
		}
		else
		{
					while(gimbal_motor.yaw_set - imu.yaw[2] > 300)
						{gimbal_motor.yaw_set -= 360;}
					while(gimbal_motor.yaw_set - imu.yaw[2] < -300)
						{gimbal_motor.yaw_set += 360;}
						gimbal_motor.yaw_position_value = pid_positionloop(&buff_yaw_positionloop,imu.yaw[2],gimbal_motor.yaw_set);
						gimbal_motor.yaw_out = pid_speedloop(&buff_yaw_speedloop,yaw_accel,gimbal_motor.yaw_position_value);
						gimbal_motor.pitch_position_value = pid_positionloop(&buff_pitch_positionloop,imu.pitch,pitch_set);
						gimbal_motor.pitch_out = pid_speedloop(&buff_pitch_speedloop,pitch_accel,gimbal_motor.pitch_position_value);
						aim_shoot_buff();
		}
	}
}
/*****************************************************************************************/
float watch_yaw;
extern float temp_auto_yaw;
/**
  * @brief         云台角度设定值更新
  * @author        Z&S
  * @retval        yaw目标值；pitch目标值
  */
void gimbal_set_update(void)//pitch，yaw，电机设定值更新
{
	if(gimbal_state == COMMON||gimbal_state == WEA_COM)//手动
	{
		gimbal_motor.pitch_set -= (rc_ctrl.L_y*0.0012 + (rc_ctrl.mouse_y*0.0012));
		gimbal_motor.yaw_set +=  (rc_ctrl.L_x*0.0012 - (rc_ctrl.mouse_x*0.0020));
		
		if((temp_of_one==0)&&(VT13_ctrl.F==1))
		{
			temp_of_one=1;
			gimbal_motor.yaw_set+=180;
		}
		else if(VT13_ctrl.F==0)
		{
		  temp_of_one=0;
		}
		if(rc_ctrl.off_line_flag==1)
		{
			gimbal_motor.pitch_set -= (VT13_ctrl.L_y*0.0012 + (VT13_ctrl.mouse_y*0.0012));
			gimbal_motor.yaw_set +=  (VT13_ctrl.L_x*0.0012 - (VT13_ctrl.mouse_x*0.0020));
		}
	}
	else if(gimbal_state == AIM)//自瞄
	{
		if(aim_ok_flag==-1||aim_ok_flag==0)
			{	
					gimbal_motor.pitch_set -= (rc_ctrl.L_y*0.0012 + (rc_ctrl.mouse_y*0.0010));
					gimbal_motor.yaw_set +=  (rc_ctrl.L_x*0.0012 - (rc_ctrl.mouse_x*0.0035));
				if(rc_ctrl.off_line_flag==1)
				{
					gimbal_motor.pitch_set -= (VT13_ctrl.L_y*0.0012 + (VT13_ctrl.mouse_y*0.0012));
					gimbal_motor.yaw_set +=  (VT13_ctrl.L_x*0.0012 - (VT13_ctrl.mouse_x*0.0020));
				}
			}
		else
			{
				watch_yaw = temp_auto_yaw;
				gimbal_motor.auto_yaw = watch_yaw;
				gimbal_motor.pitch_set = gimbal_motor.auto_pitch;
				gimbal_motor.yaw_set = gimbal_motor.auto_yaw;
			}
	}
	else if (gimbal_state == BUFF)//打符
	{
		if(aim_ok_flag==-1||aim_ok_flag==0)
			{	
				gimbal_motor.pitch_set -= (rc_ctrl.L_y*0.0012 + (rc_ctrl.mouse_y*0.0010));
				gimbal_motor.yaw_set +=  (rc_ctrl.L_x*0.0012 - (rc_ctrl.mouse_x*0.0035));
				if(rc_ctrl.off_line_flag==1)
					{
						gimbal_motor.pitch_set -= (VT13_ctrl.L_y*0.0012 + (VT13_ctrl.mouse_y*0.0012));
						gimbal_motor.yaw_set +=  (VT13_ctrl.L_x*0.0012 - (VT13_ctrl.mouse_x*0.0020));
					}
			}
		else
			{
					watch_yaw = temp_auto_yaw;
					gimbal_motor.auto_yaw = watch_yaw;
					gimbal_motor.pitch_set = gimbal_motor.auto_pitch;
					gimbal_motor.yaw_set = gimbal_motor.auto_yaw;
			}
	}
	gimbal_motor.pitch_set = gimbal_motor.pitch_set > 23.3 ? 23.3:gimbal_motor.pitch_set;
	gimbal_motor.pitch_set = gimbal_motor.pitch_set < -31 ? -31:gimbal_motor.pitch_set;
}
/*****************************************************************************************/

/**
  * @brief         pid结构体初始化
  * @author        Z&S
  * @retval        各云台模式下pid结构体初始化值
  */
float kp,ki,kd,KP,KI,KD;
void gimbal_pid_init(void)//pitch,yaw电机pid参数初始化
{
	/*手动控制*/
	pid_speedloop_init(&pitch_speedloop,-1500,0.0,0.0, 28000.0f, 28000.0f, 10000.0f, 4000.0f, 100.0f);//-1500
	pid_positionloop_init(&pitch_positionloop,8,0.0,0.0, 3000.0f, 3000.0f, 100.0f, 500.0f,10000);//9
	pid_speedloop_init(&yaw_speedloop,1800,0, 0,23000.0f, 23000.0f, 1000.0f, 30000.0f, 50);//80
	pid_positionloop_init(&yaw_positionloop,2.0f,0.0,60,5000,5000,800,500,28000);//20
	/*自瞄*/
	pid_speedloop_init(&auto_pitch_speedloop,-1800,0,0.0, 28000.0f, 28000.0f, 10000.0f, 4000.0f, 100.0f);
	pid_positionloop_init(&auto_pitch_positionloop,9,0.0004,0.0, 3000.0f, 3000.0f, 100.0f, 500.0f,10000);//6.5
	pid_speedloop_init(&auto_yaw_speedloop,2300,0, 0,28000.0f, 28000.0f, 1000.0f, 30000.0f, 50);//80//4000
	pid_positionloop_init(&auto_yaw_positionloop,4.0,0.0001,0.0,1000,1000,800,500,28000);//20//2
	/*打符*/
	pid_speedloop_init(&buff_pitch_speedloop,-1800,0,0.0, 28000.0f, 28000.0f, 10000.0f, 4000.0f, 100.0f);
	pid_positionloop_init(&buff_pitch_positionloop,9,0.0004,0.0, 3000.0f, 3000.0f, 100.0f, 500.0f,10000);
	pid_speedloop_init(&buff_yaw_speedloop,2200,0, 0,28000.0f, 28000.0f, 1000.0f, 30000.0f, 50);//80
	pid_positionloop_init(&buff_yaw_positionloop,3.8,0.0001,0.0,1000,1000,800,500,28000);//20
}
/*****************************************************************************************/



float my_fabs(float num)
{
	if(num<0)num=-num;
	else if(num>=0)num=num;
	return num;

}
















































































