#include "main.h"
#include "motor.h"
#include "referee.h"
#include "bsp_pid.h"
#include "power_task.h"
#include "arm_math.h"
extern ext_power_heat_data_t      power_heat_data;
extern ext_robot_status_t			  	robot_state;
extern measure_t chassis;
extern motor_speed motor;
extern measure_t chassis;
extern PID chassis_motor_speedloop[4];
extern measure_t chassis;
extern super_energy super_cap;
uint16_t max_power_limit=0;
float chassis_max_power=0;
float input_power=0;           //电源功率（来自裁判系统）
float initial_give_power[4];   //PID计算的初始功率
float initial_total_power=0;   //总功率
fp32 scaled_give_power[4];
extern PID powerControl_PID;

	
fp32 chassis_power=0.0f;
fp32 chassis_power_buffer=0.0f;
	
fp32 CT=1.99688994e-6f;         //力矩电流系数(20/16384)*(0.3)*(187/3591)/9.55
fp32 k1 = 1.23e-07;             // k1//1.23
fp32 k2 = 3.753e-07;            // k2//1.453
fp32 constant = 1.181f;
fp32 power_scale;
fp32 ka,kb,kc;
fp32 inside;
fp32 temp;
float cmd;
float s_ch_power;//临时缩放功率系数
/******************************************************获取裁判系统数据*********************************************************/
void get_chassis_and_buffer(float *power,float *buffer)
{
	*power=power_heat_data.chassis_power;
	*buffer=(float)power_heat_data.chassis_power_buffer;
}

void get_chassis_max_power(uint16_t  *max_power_limit)
{
	*max_power_limit=robot_state.chassis_power_limit;
}

/*******************************************************功率控制函数*************************************************************/
void taskPowerControl(motor_measure_t *chassis_power_control)
{
	//chassis_max_power=0.0f;
	//pid_positionloop_init(&powerControl_PID,3,0 ,0,20000,18000,18000,18000);
	//get_chassis_and_buffer(&chassis_power,&chassis_power_buffer);//得到底盘功率和缓冲能量(裁判系统)
	//pid_positionloop(&powerControl_PID,chassis_power_buffer,55); //缓冲能量闭环
	//get_chassis_max_power(&max_power_limit);                     //得到最大功率
//	pid_positionloop(&powerControl_PID,(super_cap.Voltage/100),15.5);
//	if(powerControl_PID.out<-10)
//	{
//		powerControl_PID.out=-10;
//	}
//	else if(powerControl_PID.out<-10)
//	{
//		powerControl_PID.out=-10;
//	}
	
//	  powerControl_PID.out=-25-12*(super_cap.Voltage/100 - 14.5);
//	if(powerControl_PID.out<-100) {powerControl_PID.out=-100;}
//	if(powerControl_PID.out>-40) {powerControl_PID.out=-40;}

	input_power=robot_state.chassis_power_limit;//*0.933);//-powerControl_PID.out;
  
  if(super_cap.Voltage>1550)
	{
		if(chassis.super_mode==0)
		{
			chassis_max_power=input_power;
			//s_ch_power = chassis_max_power*0.933;
		}
		else if(chassis.super_mode==1)
		{
			chassis_max_power=input_power +100;
			//s_ch_power = chassis_max_power*0.933;
		}
	}
	else
  {
		chassis_max_power=input_power;
		//s_ch_power = chassis_max_power*0.933;
	}
	initial_total_power=0.0f;
	for(uint8_t i=0;i<4;i++)
	{
		initial_give_power[i]=chassis_motor_speedloop[i].out*CT*chassis_power_control[i].speed_rpm+
		                      k1*chassis_power_control[i].speed_rpm*chassis_power_control[i].speed_rpm+
		                      k2*chassis_motor_speedloop[i].out*chassis_motor_speedloop[i].out+constant;   //初始每个轮子的功率
		if (initial_give_power[i] < 0) 
			continue;
		initial_total_power += initial_give_power[i];                                                       //计算总功率
	}
	
	if (initial_total_power >chassis_max_power ) //是否大于最大功率
	{
		//按电机分配功率
		power_scale = chassis_max_power  / initial_total_power;
		for (uint8_t i = 0; i < 4; i++)
		{
			
			scaled_give_power[i] = initial_give_power[i] * power_scale; // 得到缩放功率
			if (scaled_give_power[i] < 0)
			{
				continue;
			}
			
			ka=k1;
			kb = CT * chassis_power_control[i].speed_rpm;
			kc = k2 * chassis_power_control[i].speed_rpm * chassis_power_control[i].speed_rpm - scaled_give_power[i] + constant;
			inside = kb * kb - 4 * ka * kc;

			if (inside < 0)
			{
				continue;
			}
			else if (chassis_motor_speedloop[i].out > 0) // 根据原始力矩方向选择计算公式
			{
				arm_sqrt_f32(inside,&cmd);
				temp = (-kb + cmd) / (2 * ka);
				motor.motor_out[i]=(temp>16000?16000:temp);
			}
			else
			{
				arm_sqrt_f32(inside,&cmd);
				temp = (-kb - cmd) / (2 * ka);
				motor.motor_out[i]=(temp<-16000?-16000:temp);
			}
		}
		return;
	}
	//若总功率未超过最大功率，则将原始速度环PID的值发给底盘
	for(uint8_t i=0;i<4;i++)
	{
		motor.motor_out[i]=chassis_motor_speedloop[i].out;
	}
}
