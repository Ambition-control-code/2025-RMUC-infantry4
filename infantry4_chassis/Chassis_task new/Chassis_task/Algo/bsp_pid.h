#ifndef __PID_H
#define __PID_H
#include "stm32f4xx.h"
typedef struct PIDset
  {  
		 
     double PMAX;//比例最大输出限制
		 double IMAX;//积分最大输出限制
		 double DMAX;//积分最大输出限制
		 double Sum_error_max;//误差累计限制
		
		 float OMAX;
		
		 float Kp,Ki,Kd;//pid参数
		
		 float out;//pid输出
		 
		 float Pout;//比例输出
		 float Iout;//积分输出
		 float Dout;//微分输出
		
     float error[3];     
		 float sum_error;
		 float sum_errorlast;
     float set;//目标值
     float real;//真实值
     	 	
  }PID;

void pid_positionloop_init(PID *pid,float kp,float ki,float kd, float omax, float pmax, float imax, float dmax);
void pid_speedloop_init(PID *pid,float kp,float ki,float kd, float omax, float pmax, float imax, float dmax,float sum_error_max);
float  pid_speedloop(PID *Pid,float real,float set);
float  pid_positionloop(PID *Pid,float real,float set);
float  PID_Incremental(PID *Pid,float real,float set);
	
	
#endif
	
	
	

	