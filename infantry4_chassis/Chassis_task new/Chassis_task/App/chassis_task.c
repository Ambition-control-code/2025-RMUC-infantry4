#include "chassis_task.h"
#include "main.h"
#include "INS_task.h"
#include "bsp_pid.h"
//#include "AHRS_MiddleWare.h"
//#include "AHRS.h"
#include "math.h"

#define DISTANCE 0.22
#define YAW_MECHANICAL_CENTRE -172.945923
#define OMNI_WHEEL//选择轮组
/*声明外部变量*/
extern measure_t chassis;
extern motor_measure_t CHISSIS_M[4];
extern motor_measure_t YAW_Motor;
extern measure_t chassis;
extern IMU_angle imu_angle ;
/***************************/
/*定义pid结构体*/
PID chassis_motor_speedloop[4];
PID chassis_motor_positionloop;
PID position_loop;
PID	follow_Yaw_speed_loop;
PID follow_Yaw_angle_loop;

float kp,ki,kd,KP,KI,KD;
MODE_SWITCH chiss_mode;//模式选择
motor_speed motor;
chassis_sport chassis_run;

float error;
void chassis_task(void)
{
	//all_pid_init();
	//McKnum_wheel();
}

/**
  * @brief         pid初始化
  * @author        Z&S
  * @retval        初始化完成后结构体指针
  */
float K_p;
void all_pid_init(void)
{
	pid_speedloop_init(&chassis_motor_speedloop[0], 10, 0.18f, 0.0f, 16000.0f, 16000.0f, 5000.0f, 2000.0f,20000);//ID 0x201
	pid_speedloop_init(&chassis_motor_speedloop[1], 10, 0.18f, 0.0f, 16000.0f, 16000.0f, 5000.0f, 2000.0f,20000);//8.5
	pid_speedloop_init(&chassis_motor_speedloop[2], 10, 0.18f, 0.0f, 16000.0f, 16000.0f, 5000.0f, 2000.0f,20000);
	pid_speedloop_init(&chassis_motor_speedloop[3], 10, 0.18f, 0.0f, 16000.0f, 16000.0f, 5000.0f, 2000.0f,20000);
	
	//pid_speedloop_init(&follow_Yaw_speed_loop,0.4,0, 0, 20000.0f, 20000.0f, 10000.0f, 10000.0f,10000);//0.8
	
	pid_speedloop_init(&follow_Yaw_angle_loop,140,0.1,120, 36000.0f, 36000.0f,1200.0f, 36000.0f,1000);//底盘跟随初始化//120
	
//	
	pid_positionloop_init(&chassis_motor_positionloop,8.0f, 0.0f, 0.0f, 20000.0f, 20000.0f, 0.0f, 0.0f);//小陀螺//实际应为速度环
//	pid_positionloop_init(&position_loop,-30,0,2,10000,20000,20000,20000);//抗干扰//跟随

}


/**
  * @brief         小陀螺
  * @author        Z&S
  * @retval        小陀螺旋转pid输出量，小陀螺圈数，yaw轴最终角度
  */
void rotate(float num)//小陀螺
{
	chassis_run.rotate_speed=(CHISSIS_M[0].speed_rpm+CHISSIS_M[1].speed_rpm+CHISSIS_M[2].speed_rpm+CHISSIS_M[3].speed_rpm)/4;

	motor.location_set = num;//pid_positionloop(&chassis_motor_positionloop,chassis_run.rotate_speed,num);

	
	chassis_run.yaw_count = (int)(YAW_Motor.ecd_angle-YAW_MECHANICAL_CENTRE)/360;//小陀螺状态下转的圈数
	chassis_run.yaw_remainder = (int)(YAW_Motor.ecd_angle-YAW_MECHANICAL_CENTRE)%360;//小陀螺模式下最终yaw轴角度
	
}

/**
  * @brief         底盘电机电流初始值计算
  * @author        Z&S
	* @param   			 对应结构体指针，每个电机的原始值，每个电机的设定值
  * @retval        电机电流初始值
  */   
void chassis_speedloop()//pid计算
{
	pid_speedloop(&chassis_motor_speedloop[0],CHISSIS_M[0].speed_rpm,motor.speed_set[0]);//BR
	pid_speedloop(&chassis_motor_speedloop[1],CHISSIS_M[1].speed_rpm,motor.speed_set[1]);//BL
	pid_speedloop(&chassis_motor_speedloop[2],CHISSIS_M[2].speed_rpm,motor.speed_set[2]);//FL
	pid_speedloop(&chassis_motor_speedloop[3],CHISSIS_M[3].speed_rpm,motor.speed_set[3]);//FR
	taskPowerControl(CHISSIS_M);
  CAN_cmd_chassis(motor.motor_out[0],motor.motor_out[1],motor.motor_out[2],motor.motor_out[3]);
}
/*
|        |  0x      3   4                             —— 
																								
									   2    1
																						     
|        |

  * @brief   底盘解算任务
	* @author  Z&S
  * @param   底盘轮组结构，底盘运动方向设定值
  * @retval  底盘电机pid参数设定值
*/
//通过PID解决直线行驶跑偏问题
float compe=0.2;
float spin_speed;
void dynamic_solution()//全向轮解算
{
		spin_speed = (CHISSIS_M[0].real_speed + CHISSIS_M[0].real_speed + CHISSIS_M[0].real_speed + CHISSIS_M[0].real_speed)/4 ;
	/**********************麦轮***********************************/	
		float ecd_angle_difference;
		if(chassis.mode ==7)
		{
			ecd_angle_difference=-(YAW_Motor.ecd_angle - chassis_run.follow_angle_set);//+spin_speed*compe;//+k_w*v_w//前馈
		}
		else if(chassis.mode ==4||chassis.mode==5)
		{
			ecd_angle_difference=-(YAW_Motor.ecd_angle - chassis_run.follow_angle_set)+compe*spin_speed;
		}
/*	
	单位时间内底盘与云台间的角度差即为角速度 chassis_run.yaw_remainder%360-YAW_MECHANICAL_CENTRE
	通过陀螺仪得到底盘角速度 ecd_angle_difference
	compre=pid(real,target);
	real = v_w;
*/	
	  motor.x_set=-(chassis.x_setl*10)*cos(ecd_angle_difference/57.29578f) + (chassis.y_setl*10)*sin(ecd_angle_difference/57.29578f);//左右
	  motor.y_set=(chassis.y_setl*10)*cos(ecd_angle_difference/57.29578f) + (chassis.x_setl*10)*sin(ecd_angle_difference/57.29578f);//前进
#ifdef MACNAM_WHEEL

	  //以上语句保证车子前进方向始终为云台正方向
	  motor.A =  ((+motor.x_set + motor.y_set)*6 + (motor.location_set)*DISTANCE);//A
    motor.B =  ((+motor.x_set - motor.y_set)*6 + (motor.location_set)*DISTANCE);//B
	  motor.C =  ((-motor.x_set - motor.y_set)*6 + (motor.location_set)*DISTANCE);//C
    motor.D =	 ((-motor.x_set + motor.y_set)*6 + (motor.location_set)*DISTANCE);//D
	
		if(chassis_run.head_mode==0)
			{
				motor.speed_set[0]=motor.D;
				motor.speed_set[1]=motor.C;
				motor.speed_set[2]=motor.B;
				motor.speed_set[3]=motor.A;
			}
			else if(chassis_run.head_mode==180)
			{
				motor.speed_set[0]=motor.B;
				motor.speed_set[1]=motor.A;
				motor.speed_set[2]=motor.D;
				motor.speed_set[3]=motor.C;
			}
/************************全向轮******************************/		
#else
float max_speed,scal_lim;
	   motor.A =  -motor.x_set - motor.y_set -(motor.location_set)*DISTANCE;// -/A-----\B-
     motor.B =	-motor.x_set + motor.y_set - (motor.location_set)*DISTANCE;//
		 max_speed = my_fabs(motor.B)>my_fabs(motor.A)?my_fabs(motor.B):my_fabs(motor.A);
	   motor.C =  motor.x_set + motor.y_set - (motor.location_set)*DISTANCE;// 
		 max_speed = my_fabs(motor.C)>max_speed?my_fabs(motor.C):max_speed;
     motor.D =  motor.x_set - motor.y_set - (motor.location_set)*DISTANCE;//  -\D-----/C-
		 max_speed = my_fabs(motor.D)>max_speed?my_fabs(motor.D):max_speed;
		 if(max_speed>8000)
		 {
				scal_lim = 8000.0f/max_speed;
		 }
		 else 
		 {
				scal_lim = 1.0f;
		 }
		 motor.A = motor.A*scal_lim;
		 motor.B = motor.B*scal_lim;
		 motor.C = motor.C*scal_lim;
		 motor.D = motor.D*scal_lim;
//		  chassis_task.A =  -chassis_task.vx_set  + chassis_task.wz_set*DISTANCE;// -/A-----\B-
//			chassis_task.B =  chassis_task.vy_set  + chassis_task.wz_set*DISTANCE;//        
//			chassis_task.C =  chassis_task.vx_set + chassis_task.wz_set*DISTANCE;//   
//			chassis_task.D =  -chassis_task.vy_set + chassis_task.wz_set*DISTANCE;// -\D-----/C-
		
    if(chassis_run.head_mode==0)
		{
			motor.speed_set[0]=motor.C;//1//BR
			motor.speed_set[1]=motor.D;//2//BL
			motor.speed_set[2]=motor.A;//3//FL
			motor.speed_set[3]=motor.B;//4//FR
		}
//		else if(chassis_run.head_mode==90)
//		{
//			motor.speed_set[0]=motor.D;
//			motor.speed_set[1]=motor.A;
//			motor.speed_set[2]=motor.B;
//			motor.speed_set[3]=motor.C;
//		}
		else if(chassis_run.head_mode==180)
		{
			motor.speed_set[0]=motor.A;
			motor.speed_set[1]=motor.B;
			motor.speed_set[2]=motor.C;
			motor.speed_set[3]=motor.D;
		}
//		else if(chassis_run.head_mode==270)
//		{
//			motor.speed_set[0]=motor.B;
//			motor.speed_set[1]=motor.C;
//			motor.speed_set[2]=motor.D;
//			motor.speed_set[3]=motor.A;
//		}
#endif
}
/**
  * @brief   底盘跟随任务
	* @author  Z&S
  * @param   yaw轴角度，底盘相对云台角度
  * @retval  底盘跟随pid输出值
*/
void follow_task(void)
{
#ifdef MACNAM_WHEEL
/********************************麦克纳姆轮*************************************/
	  if(chassis_run.yaw_remainder<=90&&chassis_run.yaw_remainder>=0)  
			{chassis_run.spin_flag=1;chassis_run.angle_compensate=0.0f;}	
		else if(chassis_run.yaw_remainder<=270&&chassis_run.yaw_remainder>90) 
			{chassis_run.spin_flag=3;chassis_run.angle_compensate=180.0f;}			
		else if(chassis_run.yaw_remainder<=360&&chassis_run.yaw_remainder>270)  
			{chassis_run.spin_flag=1;chassis_run.angle_compensate=360.0f;}	
		else if(chassis_run.yaw_remainder<=0&&chassis_run.yaw_remainder>-90)  
			{chassis_run.spin_flag=1;chassis_run.angle_compensate=0.0f;}
		else if(chassis_run.yaw_remainder<=-90&&chassis_run.yaw_remainder>-270) 
			{chassis_run.spin_flag=3;chassis_run.angle_compensate=-180.0f;}
		else if(chassis_run.yaw_remainder<=-270&&chassis_run.yaw_remainder>-360)  
			{chassis_run.spin_flag=1;chassis_run.angle_compensate=-360.0f;}
		if(chassis_run.spin_flag==1)chassis_run.head_mode=0;
		else if(chassis_run.spin_flag==3)chassis_run.head_mode=180;
/**************************************全向轮***************************************/
#else
	 if(chassis_run.yaw_remainder<=90&&chassis_run.yaw_remainder>=0)  
			{chassis_run.spin_flag=1;chassis_run.angle_compensate=0.0f;}	
		else if(chassis_run.yaw_remainder<=270&&chassis_run.yaw_remainder>90) 
			{chassis_run.spin_flag=3;chassis_run.angle_compensate=180.0f;}			
		else if(chassis_run.yaw_remainder<=360&&chassis_run.yaw_remainder>270)  
			{chassis_run.spin_flag=1;chassis_run.angle_compensate=360.0f;}	
		else if(chassis_run.yaw_remainder<=0&&chassis_run.yaw_remainder>-90)  
			{chassis_run.spin_flag=1;chassis_run.angle_compensate=0.0f;}
		else if(chassis_run.yaw_remainder<=-90&&chassis_run.yaw_remainder>-270) 
			{chassis_run.spin_flag=3;chassis_run.angle_compensate=-180.0f;}
		else if(chassis_run.yaw_remainder<=-270&&chassis_run.yaw_remainder>-360)  
			{chassis_run.spin_flag=1;chassis_run.angle_compensate=-360.0f;}
		if(chassis_run.spin_flag==1)chassis_run.head_mode=0;
		else if(chassis_run.spin_flag==3)chassis_run.head_mode=180;
#endif			

	if(chassis_run.spin_flag==1)chassis_run.head_mode=0;
	else if(chassis_run.spin_flag==2)chassis_run.head_mode=90;
	else if(chassis_run.spin_flag==3)chassis_run.head_mode=180;
	else if(chassis_run.spin_flag==4)chassis_run.head_mode=270;
	/*
	float v_w;float k_w;float r_wheel;flaot r_car//通过前馈解决小陀螺行进过程中走不直的问题
	v_w = chassis_run.rotate*PI*r_wheel/r_car;//角速度
	
	*/
	chassis_run.rotate_speed=(CHISSIS_M[0].speed_rpm+CHISSIS_M[1].speed_rpm+CHISSIS_M[2].speed_rpm+CHISSIS_M[3].speed_rpm)/4;		
	chassis_run.follow_angle_set=YAW_MECHANICAL_CENTRE+chassis_run.yaw_count*360.0f+chassis_run.angle_compensate;//更新跟随角度
	//若为全向轮，四方均可作云台前进正方向
	//若为麦轮，前后两面均可作云台前进正方向
	//以上用来找到最快回中的角度
			
	static float last_yaw_motor;
	float real_out;
	real_out=0.6*YAW_Motor.ecd_angle+0.4*last_yaw_motor;
	last_yaw_motor=YAW_Motor.ecd_angle;
			
	motor.location_set=pid_speedloop(&follow_Yaw_angle_loop,real_out,chassis_run.follow_angle_set);
	
//	if(fabs(follow_Yaw_angle_loop.error[0])<0.8f){follow_Yaw_angle_loop.sum_error=0;}
			
  chassis_run.rotate_speed=(CHISSIS_M[0].speed_rpm+CHISSIS_M[1].speed_rpm+CHISSIS_M[2].speed_rpm+CHISSIS_M[3].speed_rpm)/4;
			
 // motor.location_set=pid_speedloop(&follow_Yaw_speed_loop,chassis_run.rotate_speed,motor.location_set);//底盘跟随pid
}

	

void indep_task(void)
{
	
	motor.location_set=0;
	motor.motor_out[0]=pid_speedloop(&chassis_motor_speedloop[0],CHISSIS_M[0].speed_rpm,chassis.y_set*10);
	motor.motor_out[1]=pid_speedloop(&chassis_motor_speedloop[1],CHISSIS_M[1].speed_rpm,chassis.y_set*10);
	motor.motor_out[2]=pid_speedloop(&chassis_motor_speedloop[2],CHISSIS_M[2].speed_rpm,chassis.y_set*10);
	motor.motor_out[3]=pid_speedloop(&chassis_motor_speedloop[3],CHISSIS_M[3].speed_rpm,chassis.y_set*10);
//		motor.A =  chassis.y_set*5;
//    motor.B =  chassis.y_set*5;
//	  motor.C =  chassis.y_set*5;
//    motor.D =	 chassis.y_set*5;
}	
float my_fabs(float num)
{
	if(num>=0)num=num;
	else if(num<0)num=-num;
	return num;
}