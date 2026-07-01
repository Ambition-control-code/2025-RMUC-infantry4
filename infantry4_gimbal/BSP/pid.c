#include "pid.h"
#include"main.h"

void pid_positionloop_init(PID *pid,float kp,float ki,float kd, float omax, float pmax, float imax, float dmax,float sum_error_max)
{
	pid->OMAX=omax;
	pid->PMAX=pmax;
	pid->IMAX=imax;
	pid->DMAX=dmax;
	pid->Sum_error_max=sum_error_max;
	
	pid->Kp=kp;
	pid->Ki=ki;
	pid->Kd=kd;
	
	pid->out=0.0f;
	pid->Pout=0.0f;
	pid->Iout=0.0f;
	pid->Dout=0.0f;
	pid->error[0]=0.0f;
	pid->error[1]=0.0f;
	pid->error[2]=0.0f;
	pid->set=0.0f;
	pid->real=0.0f;
	
}

void pid_speedloop_init(PID *pid,float kp,float ki,float kd, float omax, float pmax, float imax, float dmax, float sum_error_max)
{
	pid->OMAX=omax;
	pid->PMAX=pmax;
	pid->IMAX=imax;
	pid->DMAX=dmax;
	pid->Sum_error_max=sum_error_max;
	
	pid->Kp=kp;
	pid->Ki=ki;
	pid->Kd=kd;
	
	pid->out=0.0f;
	pid->Pout=0.0f;
	pid->Iout=0.0f;
	pid->Dout=0.0f;
	pid->error[0]=0.0f;
	pid->error[1]=0.0f;
	pid->error[2]=0.0f;
	pid->set=0.0f;
	pid->real=0.0f;
	
}
float  pid_positionloop(PID *Pid,float real,float set)
{//Î»ÖÃ

	Pid->set = set;
	Pid->real = real;
	Pid->error[0] = set - real;
	
	Pid->sum_error += Pid->error[0];
  Pid->sum_error > Pid->Sum_error_max ? Pid->sum_error = Pid->Sum_error_max : 1;
	Pid->sum_error < -Pid->Sum_error_max ? Pid->sum_error = -Pid->Sum_error_max : 1;
	Pid->Pout=Pid->Kp * Pid->error[0] ;
	Pid->Pout > Pid->PMAX ? Pid->Pout = Pid->PMAX : 1;
	Pid->Pout < -Pid->PMAX ? Pid->Pout = -Pid->PMAX : 1;

	Pid->Iout=Pid->Ki * Pid->sum_error ;
	Pid->Iout > Pid->IMAX ? Pid->Iout=Pid->IMAX :1;
	Pid->Iout < -Pid->IMAX ?Pid->Iout = -Pid->IMAX :1;

	Pid->Dout=Pid->Kd * ( Pid->error[0] - Pid->error[1] );
	Pid->Dout > Pid->DMAX ? Pid->Dout = Pid->DMAX : 1;
	Pid->Dout < -Pid->DMAX ? Pid->Dout = -Pid->DMAX : 1;
	
	Pid->out=Pid->Pout+Pid->Iout+Pid->Dout;
	Pid->out > Pid->OMAX ? Pid->out = Pid->OMAX : 1;
	Pid->out < -Pid->OMAX ? Pid->out = -Pid->OMAX : 1;

	Pid->error[1] = Pid->error[0];
	

	return Pid->out;
	
	
}
 
float  pid_speedloop(PID *Pid,float real,float set)
{//Î»ÖÃ

	Pid->set = set;
	Pid->real = real;
	Pid->error[0] = set - real;
	
	Pid->sum_error += Pid->error[0];
  Pid->sum_error > Pid->Sum_error_max ? Pid->sum_error = Pid->Sum_error_max : 1;
	Pid->sum_error < -Pid->Sum_error_max ? Pid->sum_error = -Pid->Sum_error_max : 1;
	
	Pid->Pout=Pid->Kp * Pid->error[0] ;
	Pid->Pout > Pid->PMAX ? Pid->Pout = Pid->PMAX : 1;
	Pid->Pout < -Pid->PMAX ? Pid->Pout = -Pid->PMAX : 1;

	Pid->Iout=Pid->Ki * Pid->sum_error ;
	Pid->Iout > Pid->IMAX ? Pid->Iout=Pid->IMAX :1;
	Pid->Iout < -Pid->IMAX ?Pid->Iout = -Pid->IMAX :1;

	Pid->Dout=Pid->Kd * ( Pid->error[0] - Pid->error[1] );
	Pid->Dout > Pid->DMAX ? Pid->Dout = Pid->DMAX : 1;
	Pid->Dout < -Pid->DMAX ? Pid->Dout = -Pid->DMAX : 1;
	
	Pid->out=Pid->Pout+Pid->Iout+Pid->Dout;
	Pid->out > Pid->OMAX ? Pid->out = Pid->OMAX : 1;
	Pid->out < -Pid->OMAX ? Pid->out = -Pid->OMAX : 1;

	Pid->error[1] = Pid->error[0];
	

	return Pid->out;
	
	
}