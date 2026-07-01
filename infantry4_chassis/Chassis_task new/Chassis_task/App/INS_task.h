#ifndef INS_TASK_H
#define INS_TASK_H
#include"bsp_flash.h"
#include "stm32f4xx.h"
#include "BMI088driver.h"
typedef float fp32;
#define INS_DELTA_TICK 1 //任务调用的间隔
#define IMU_BOARD_INSTALL_SPIN_MATRIX                           \
                                        { 0.0f, 1.0f, 0.0f},    \
                                        {-1.0f, 0.0f, 0.0f},    \
                                        { 0.0f, 0.0f, 1.0f}    \
//处理陀螺仪，加速度计，磁力计数据的线性度，零漂	
																				//0.0 1.0 0.0roll
																				//-1.0 0.0 0.0pitch
																				//0.0 0.0 0.0yaw
#define GYRO_CALIBRATE_TIME 60000   //较准时间 20000 / 1000 = 20s
#define GYRO_OFFSET_KP 0.0003f //调整这个可以调整陀螺仪校准速度，越大陀螺仪校准变化越快，但波动会变大3//0.0003
#include "motor.h"																				
extern uint8_t offset_OK_flag;																				
typedef struct 
{
    fp32 offset[3]; //x,y,z
    fp32 scale[3];  //x,y,z
} imu_cali_t;
typedef struct 
{
u8 flash_flag,flash_read_flag;
float acc[3];//加速度xyz
float gyro[3],gyro_zero[3],gyro_buff[3];
float temp,cnt;
float yaw[3],yaw_cnt,yaw_diff;
float pitch;
float roll;
float q[4];
float yaw_real;
} IMU_angle;		//真实角度和角速度 （角度制）

extern IMU_angle imu_angle;

void INS_task(void);
void IMU_task(void const * argument);

#endif
