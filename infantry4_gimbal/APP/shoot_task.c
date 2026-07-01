#include "shoot_task.h"
#include "cmsis_os.h"
#include "gimbal_task.h"
#include "vision_task.h"
#include "bsp_tr.h"
#include "remote_control.h"
#include "pid.h"
#include "bsp_bull_ctr.h"
#include "tele_link.h"
//近距离低速高射频，远距离高速低射频

weapon gimbal;//武器结构体变量
extern VT13_ctrl_t VT13_ctrl;
extern int aim_flag;
extern int aim_ok_flag;
extern measure_chassis chassis_data;
extern shoot_mode mode;//设计状态
extern motor_measure_t	FrictioL_Motor;//左摩擦轮
extern motor_measure_t	FrictioR_Motor;//右摩擦轮
extern motor_measure_t	Feed_Motor;//供弹电机
extern RC_ctrl_t rc_ctrl;//遥控器参数结构体
extern uint8_t moca_flag;//摩擦轮电机状态标志
extern measure_chassis chassis_data;
float ctrl_ecd_angle;
extern gimbal_contion gimbal_state;
extern int fire_ok;
extern uint8_t feed_frequency;
extern float bullet_fre;
PID shootL_speed;//摩擦轮电机pid结构体
PID shootR_speed;
PID feed_speed;
PID feed_position;
float ctr_bulsped;//控制弹速缩放倍率
float bullet_speed_diff;
// 30  1050   15 595  18 667   23.5 850  25 900   26 950   27.5 1000  28.5 1050

float out_shoot,pinlu;


void shoot_task(void const * argument)
{
	all_pid_init();
	for(;;)
	{
		fireMotorPidReal_update();
		send_to_frictio_motor(gimbal.shootL_out,gimbal.shootR_out,feed_speed.out);
		osDelay(2);    
		pinlu++;
	}
}



float just_shoot,shoot_time;


/**
	* @brief         键鼠/拨杆控制下发射机构设定值更新
	* @author        Z&S
	* @retval        当前模式下发射机构pid计算输出值
  */
void fireMotorPidReal_update()
{
	  if(VT13_ctrl.mode==2||gimbal_state == AIM||gimbal_state ==BUFF)
		{
			gimbal.shootL_set = 6000; gimbal.shootR_set = -6000;
			gimbal.shootL_out = pid_speedloop(&shootL_speed,FrictioL_Motor.speed_rpm,gimbal.shootL_set);
			gimbal.shootR_out = pid_speedloop(&shootR_speed,FrictioR_Motor.speed_rpm,gimbal.shootR_set);
		if((rc_ctrl.L_mouse_l==1)||(VT13_ctrl.trigger==1))
		{	
			just_shoot=4000;
			shoot_time++;
			if(shoot_time>250)
			{
				just_shoot=3000;
			}
			
		  out_shoot = pid_speedloop(&feed_speed,Feed_Motor.speed_rpm,just_shoot);
		}
		else
		{
			shoot_time=0;
		  out_shoot = pid_speedloop(&feed_speed,Feed_Motor.speed_rpm,0);
		}
		}
		else 
		{	
					gimbal.shootL_out = pid_speedloop(&shootL_speed,FrictioL_Motor.speed_rpm,0);
					gimbal.shootR_out = pid_speedloop(&shootR_speed,FrictioR_Motor.speed_rpm,0);
		    	out_shoot = pid_speedloop(&feed_speed,Feed_Motor.speed_rpm,0);
		}

		
}

/**
	* @brief         拨轮控制发射机构
  * @author        Z&S
	* @retval        拨轮控制下发射机构pid输出值
*/
//本逻辑与键鼠逻辑互不干扰（检录和调试使用）
void fireMotorSingleLoopPid_out_update(void)
{
	if( VT13_ctrl.lun < 0)//高射频低射速
	{		
			gimbal.shootL_out = pid_speedloop(&shootL_speed,FrictioL_Motor.speed_rpm,gimbal.shootL_set);
			gimbal.shootR_out = pid_speedloop(&shootR_speed,FrictioR_Motor.speed_rpm,gimbal.shootR_set);
			if(VT13_ctrl.lun<0)
	{
		VT13_ctrl.lun=-VT13_ctrl.lun;
	}
	else{VT13_ctrl.lun=VT13_ctrl.lun;}
		if( FrictioR_Motor.speed_rpm <= -3000&&FrictioL_Motor.speed_rpm >= 3000)gimbal.feed_out = pid_speedloop(&feed_speed,Feed_Motor.speed_rpm,VT13_ctrl.lun*3);
			
	}
	
	else if(VT13_ctrl.lun > 0)//低射频高射速
	{   gimbal.shootL_out = pid_speedloop(&shootL_speed,FrictioL_Motor.speed_rpm,gimbal.shootL_set);
			gimbal.shootR_out = pid_speedloop(&shootR_speed,FrictioR_Motor.speed_rpm,gimbal.shootR_set);
			if(VT13_ctrl.lun<0)
	{
		VT13_ctrl.lun=-VT13_ctrl.lun;
	}
	else{VT13_ctrl.lun=VT13_ctrl.lun;}
		if( FrictioR_Motor.speed_rpm <= -3000&&FrictioL_Motor.speed_rpm >= 3000)gimbal.feed_out = pid_speedloop(&feed_speed,Feed_Motor.speed_rpm,VT13_ctrl.lun*3);
	}
}
int speed_set;

/**
  * @brief         初始射速切换及弹速自动调节
  * @author        Z&S
	* @retval        当前等级下摩擦轮转速目标值
  */
void shoot_speed_ctrl(void)
{
	switch (15) 
		{
      case 30:	gimbal.shootL_set = 7200; gimbal.shootR_set = -7200;break;// 弹速28.0-29.0的实测ref//8200
      case 18:	gimbal.shootL_set = 5600; gimbal.shootR_set = -5600;break;//17.0-17.8	//5600																												// 17.0-17.8
			case 15:	gimbal.shootL_set = 6000; gimbal.shootR_set = -6000;break;//13.9-14.7//4800
      case 0 :  gimbal.shootL_set = 	 0; gimbal.shootR_set = 		  0;break;//13.9-14.7// 刹车
      default:	gimbal.shootL_set = 3000; gimbal.shootR_set = -3000;break;// 与最低弹速保持一致
    }
}

/**
	* @brief         发射机构pid结构体初始化
  * @author        Z&S
	* @retval        初始化后结构体
  */
void all_pid_init(void)//pid初始化
{
	pid_speedloop_init(&shootL_speed,10,0.07,0, 28000, 28000, 12000.0f, 15500.0f,20000.0f);//
	pid_speedloop_init(&shootR_speed,10,0.07,0, 28000, 28000, 12000.0f, 15500.0f,20000.0f);
	pid_speedloop_init(&feed_speed,25,0.1,0, 10000.0f, 10000.0f, 1000.0f, 0.0f,20000);
//	pid_positionloop_init(&feed_position,20,0,0,15000, 15000.0f, 500.0f, 100.0f,28000);
}

/**
	* @brief        自瞄模式下拨弹盘控制
  * @author       Z&S
	* @param[in]    视觉传输的开火标志位
	* @retval       拨弹盘pid运算输出值
  */
//extern int fire_ok;
void aim_shoot(void)
{
	if(fire_ok==1&&rc_ctrl.L_mouse_l==1)
	{gimbal.feed_out = pid_speedloop(&feed_speed,Feed_Motor.speed_rpm,3000);}
	else{gimbal.feed_out = pid_speedloop(&feed_speed,Feed_Motor.speed_rpm,0);}
}
void aim_shoot_buff(void)
{
	uint8_t cnt;
	
	if(fire_ok==1)
	{gimbal.feed_out = pid_speedloop(&feed_speed,Feed_Motor.speed_rpm,500);cnt=0;}
	else{gimbal.feed_out = pid_speedloop(&feed_speed,Feed_Motor.speed_rpm,0);cnt=0;}
}
//单发模式
void single_bullet(void)
{
		if(fire_ok==1)
		{
			ctrl_ecd_angle+=1541;
		}
			float feed_position_value = pid_positionloop(&feed_position,Feed_Motor.ecd_angle,ctrl_ecd_angle);
			gimbal.feed_out = pid_speedloop(&feed_speed,Feed_Motor.speed_rpm,feed_position_value);
}
