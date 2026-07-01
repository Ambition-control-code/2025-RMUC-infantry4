#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "INS_task.h"
#include "MahonyAHRS.h"
#include "task.h"
#include "math.h"
#include "arm_math.h"
#include "AHRS.h"
bmi088_real_data_t bmi088_real_data;
IMU_angle imu_angle = {0};
void IMU_task(void const * argument)
{
	//imu_angle.q[0]=1;
	//imu_angle.flash_flag=1;
	for(;;)
	{
	//INS_task();
//send_to_capicity();
		osDelay(10);
	}
}
/**
  * @brief         解包陀螺仪数据
  * @author        Z&S
	* @attention     imu_angle.flash_flag=0即可重新校准陀螺仪
  * @retval        三轴角度及三轴加速度
  */
void INS_task()
{
	BMI088_read_(&bmi088_real_data);
  
	static fp32 accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
	static fp32 accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
	static fp32 accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
	static const fp32 fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};
	
		//判断是否第一次进入，如果第一次则初始化四元数，之后更新四元数计算角度单位rad
	if (imu_angle.cnt == 0)
	{
		
		if(imu_angle.flash_flag)
		{
		STMFLASH_Read(0X08020000,(u32*)imu_angle.gyro_zero,3);
		imu_angle.flash_read_flag=1;
		imu_angle.cnt=20000;
		}
		
		imu_angle.cnt++;
	
	}
	else if(imu_angle.cnt<20000)
	{
    imu_angle.gyro_zero[0]  += bmi088_real_data.gyro[0];
    imu_angle.gyro_zero[1]  += bmi088_real_data.gyro[1];
    imu_angle.gyro_zero[2]  += bmi088_real_data.gyro[2];	
		imu_angle.cnt++;		
	}
	else
	{		
		if(!imu_angle.flash_read_flag)
		{
		STMFLASH_Write(0X08020000,(u32*)imu_angle.gyro_zero,3); 
		imu_angle.flash_read_flag=1;
		}

    imu_angle.gyro_buff[0]  = bmi088_real_data.gyro[0]-imu_angle.gyro_zero[0]/20000;
    imu_angle.gyro_buff[1]  = bmi088_real_data.gyro[1]-imu_angle.gyro_zero[1]/20000;
    imu_angle.gyro_buff[2]  = bmi088_real_data.gyro[2]-imu_angle.gyro_zero[2]/20000;	
		
			//更新四元数
			MahonyAHRSupdate(imu_angle.q,imu_angle.gyro_buff[0],imu_angle.gyro_buff[1],imu_angle.gyro_buff[2]
			,bmi088_real_data.accel[0],bmi088_real_data.accel[1],bmi088_real_data.accel[2],0,0,0);

			imu_angle.yaw[1] = imu_angle.yaw[0]* 57.3f;
      get_angle(imu_angle.q,&imu_angle.yaw[0],&imu_angle.pitch,&imu_angle.roll);
			imu_angle.yaw_diff = imu_angle.yaw[0]* 57.3f - imu_angle.yaw[1];
			imu_angle.pitch = imu_angle.roll*57.3f;
			imu_angle.roll = imu_angle.pitch*57.3f;
			if(imu_angle.yaw_diff > 150.0f)
				imu_angle.yaw_cnt --;
			else if(imu_angle.yaw_diff < -150.0f)
				imu_angle.yaw_cnt ++;
			imu_angle.yaw[2] = imu_angle.yaw[0]* 57.3f + imu_angle.yaw_cnt * 360.0f;
	}          
}


