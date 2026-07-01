#include "state_task.h"
#include "ui_protocol.h"
#include "motor.h"
#include "bsp_pid.h"
int rc_ok_flag=1;
int abcde=0;
int last_test;
extern int test;
extern motor_measure_t CHISSIS_M[4];
extern measure_t chassis;
extern motor_speed motor;
extern client_info_t client_info;
extern RC_ctrl_t rc_ctrl;
extern PID chassis_motor_speedloop[4];
extern float chassis_max_power;
int spin_sp;
PID powerControl_PID;
//chassis:4,rotate;5,follow;6,no_power;
void STATE_TASK(void const * argument)
{
	chassis_task();
	all_pid_init();
	for(;;)
	{
		//all_pid_init();
		worke_out_chassis_mode();
		limit_ctrl_data();
//		pid_positionloop_init(&powerControl_PID,33,0,0,150,150,0,0);
		switch(chassis.mode)//底盘运动模
			{
			//case 2 :   indep_task();break;//
			case 7 :   follow_task();break;//
			case 4 :   rotate(spin_sp);break;
			case 5 :   rotate(-36300);break;
			default :  {break;}
			}
			abcde+=1;
			worke_out_rc_gim_super();
			worke_out_vision_flag();
			dynamic_solution();
			
			chassis_speedloop();//底盘pid运算
			send_to_capicity();
//			 if(xTaskGetTickCount()%500==0)
//		 {
//				
//		 }
			
			//CAN_cmd_chassis(chassis_motor_speedloop[0].out,chassis_motor_speedloop[1].out,chassis_motor_speedloop[2].out,chassis_motor_speedloop[3].out);
		osDelay(1);
	}
}



/******************************************************************/	
void limit_ctrl_data(void)
{
	if(45<= chassis_max_power && chassis_max_power < 60)
	{
		chassis.x_setl = chassis.x_set*0.55;
		chassis.y_setl = chassis.y_set*0.55;
		spin_sp = 20000;
	}
	else if(60<=chassis_max_power&&chassis_max_power <80)
	{
		chassis.x_setl = chassis.x_set*0.65;
		chassis.y_setl = chassis.y_set*0.65;
		spin_sp = 24000;
	}
	else if(80<=chassis_max_power&&chassis_max_power <100)
	{
		chassis.x_setl = chassis.x_set*0.71;
		chassis.y_setl = chassis.y_set*0.71;
		spin_sp = 24000;
	}
	else if(100<=chassis_max_power&&chassis_max_power <120)
	{
		chassis.x_setl = chassis.x_set*0.72;
		chassis.y_setl = chassis.y_set*0.72;
		spin_sp = 27000;
	}
	else if(120<=chassis_max_power&&chassis_max_power <150)
	{
		chassis.x_setl = chassis.x_set*0.78;
		chassis.y_setl = chassis.y_set*0.78;
		spin_sp = 28000;
	}
	else if(150<=chassis_max_power&&chassis_max_power <180)
	{
		chassis.x_setl = chassis.x_set*0.84;
		chassis.y_setl = chassis.y_set*0.84;
		spin_sp = 30000;
	}
	else if(chassis_max_power>=180)
	{
		chassis.x_setl = chassis.x_set*0.88;
		chassis.y_setl = chassis.y_set*0.88;
		spin_sp = 32000;
	}
}
