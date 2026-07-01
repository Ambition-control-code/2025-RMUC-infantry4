#ifndef __IMU_TASK_H
#define __IMU_TASK_H
#include "struct_typedef.h"
#include "bsp_dwt.h"
#include "spi.h"
#include "ulitility_tools.h"
#include "bsp_dwt.h"
#include "BMI088reg.h"
#include "BMI088Middleware.h"
#include "BMI088driver.h"
#include "bsp_flash.h"
#include "cmsis_os.h"
#include "kalman_filter.h"
#include "QuaternionEKF.h"
#include "user_lib.h"
#include "MadgwickAHRS.h"
typedef struct{
uint8_t flash_flag,flash_read_flag;
float acc[3],acc_fliter[3];
float gyro[3],gyro_zero[3],gyro_buff[3];
float temp,cnt;
float yaw[3],yaw_cnt,yaw_diff;
float pitch;
float roll;
float q[4];
float xn[4],yn[4],zn[4],MotionAccel_b[4],AccelLPF,MotionAccel_n[4];
}bucket_imu;

void imu_calculate_madgwick(void);
void INS_Init(void);
void Imu_task(void);
void QuaternionToEularAngle(float* q, float* Yaw, float* Pitch, float* Roll);
#endif