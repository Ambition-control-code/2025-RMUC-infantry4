#include "vision_task.h"
#include "cmsis_os.h"
#include "math.h"
#include "kalman.h"
#include "string.h"
#include "bsp_tr.h"
#include "gimbal_task.h"
#include "usbd_cdc_if.h"
#include "imu_task.h"
#include "remote_control.h"
extKalman_t yaw_kalman;
extKalman_t pitch_kalman;
int fire_ok;
int aim_ok_flag=0;
extern gimbal_sport gimbal_motor;
extern measure_chassis chassis_data;
extern bucket_imu imu;
extern RC_ctrl_t rc_ctrl;
extern gimbal_contion gimbal_state;
receive_struct receive_vision;
send_struct send_vision;
float temp_auto_yaw;
float last_temp_auto_yaw;
float auto_yaw_count;
float temp_auto_yaw_diff;
float fabs_differ;
extern float out_x_set;
extern float out_y_set;
extern uint8_t rc_gim_mode;
extern int chassis_mode;
extern int8_t vision_flag;
/*接收数据用联合体*/
union 
{
receive_struct receive_vision;//数据接收结构体与vision_data数组共用一片内存
uint8_t vision_data[16];
}vision_recieve_union;//联合体内数据公用一个内存，有利与数据格式的转换且节省内存


void vision_task(void const * argument)
{
  /* USER CODE BEGIN state_task */
	KalmanCreate( &yaw_kalman, 1, 40 );
	KalmanCreate( &pitch_kalman, 1, 40 );
  /* Infinite loop */
  for(;;)
  {
		send_vision_task();//发送数据时调用该函数
		gimbal_motor.auto_pitch=-KalmanFilter(&pitch_kalman,vision_recieve_union.receive_vision.pitch);
		last_temp_auto_yaw = temp_auto_yaw;
		temp_auto_yaw = vision_recieve_union.receive_vision.yaw;//KalmanFilter(&yaw_kalman,vision_recieve_union.receive_vision.yaw);
		temp_auto_yaw_diff = temp_auto_yaw - last_temp_auto_yaw;
		if(temp_auto_yaw_diff>150)
				auto_yaw_count--;
		else if(temp_auto_yaw_diff < -150)
				auto_yaw_count++;	
		fire_ok= vision_recieve_union.receive_vision.fire;
		aim_ok_flag=vision_recieve_union.receive_vision.distance;
		send_to_chassis(out_x_set,out_y_set,chassis_mode,rc_gim_mode,vision_flag);//发送信息给底盘	
    osDelay(10);
  }
  /* USER CODE END state_task */
}
/*出于各种原因，rmuc2025四号步兵并未实现自瞄
  由于四号步兵出车较晚，且出车过程十分坎坷，在
  后来的调试过程中也经常出现较大的毛病，导致
  自喵的进度一直被拖延，尽管调车的过程十分艰辛，
  但也十分快乐，未能上场的自瞄也成了我2025赛季最
  大的遗憾，出于我个人的原因，也十分的对不起视觉组
  同学一直以来的努力和坚持。希望后来者以我为戒。
  路虽远，行则将至；
  事虽难，做则即成。
  功步唐捐，玉汝于成；
  行而不辍，未来可期。*/





/**
  * @brief         视觉信息接收
  * @author        Z&S
  * @param[in]     信息数组
  * @retval        解包后视觉数据
  */
void receive_vision_function(uint8_t *buf)//调用该函数接收数据
{
	if(buf[0]==0xFF)
	{
		if(buf[15]==0x0D)
		{
			memcpy(&vision_recieve_union.vision_data,buf,16);
		}
	}
}


/**
  * @brief         视觉信息发送
  * @author        Z&S
  * @param[in]     敌方颜色
  * @param[in]     pitch_angle
	* @param[in]     yaw_angle
  * @retval       
  */
int8_t aim_mode;
void send_vision_task()//发送数据时调用该函数
{
	if((gimbal_state == AIM)&&rc_ctrl.L_C !=1 )
	{
		aim_mode = 1;
	}
	else if(rc_ctrl.L_C == 1)
	{
		aim_mode = 2;
	}
	send_vision.head=0xff;
	send_vision.mode=1;//chassis_data.self_color;
	send_vision.roll=imu.roll;
	send_vision.pitch=-imu.pitch;
	send_vision.yaw=imu.yaw[2] ;
	send_vision.empty=0;
	send_vision.rear=0x0d;
	uint8_t buf[16];
	memcpy(&buf,&send_vision,16);	
	CDC_Transmit_FS(buf,16);
}
















































