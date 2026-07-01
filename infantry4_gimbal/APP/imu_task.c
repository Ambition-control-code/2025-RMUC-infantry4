#include "imu_task.h"

static float INS_LPF2p_Alpha[3]={1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};
LowPassFilter2p_Info_TypeDef INS_AcceLPF2p[3];  

float INS_Angle[3] = {0.0f, 0.0f, 0.0f}; 
float MSG[3] = {0.0f, 0.0f, 0.0f}; 

bucket_imu imu;


float yaw_accel,pitch_accel;
void ins_task(void const * argument)
{
  /* USER CODE BEGIN ins_task */
	imu.flash_flag=1;
	imu.q[0]=1;
    //set spi frequency
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    
    if (HAL_SPI_Init(&hspi2) != HAL_OK)
    {
        Error_Handler();
    }
		 BMI088_read(imu.gyro,imu.acc,&(imu.temp));
		
		 LowPassFilter2p_Init(&INS_AcceLPF2p[0],INS_LPF2p_Alpha);
     LowPassFilter2p_Init(&INS_AcceLPF2p[1],INS_LPF2p_Alpha);
     LowPassFilter2p_Init(&INS_AcceLPF2p[2],INS_LPF2p_Alpha);
		
  /* Infinite loop */
  for(;;)
  {
		BMI088_read(imu.gyro,imu.acc,&(imu.temp));
		
    imu.acc_fliter[0] = LowPassFilter2p_Update(&INS_AcceLPF2p[0],imu.acc[0]);
    imu.acc_fliter[1] = LowPassFilter2p_Update(&INS_AcceLPF2p[2],imu.acc[1]);
    imu.acc_fliter[2] = LowPassFilter2p_Update(&INS_AcceLPF2p[1],imu.acc[2]);	
		
		int ii;
		if(imu.cnt==6000){imu.cnt=6000;}
		else {imu.cnt++;}
		if(!imu.flash_flag)
		{		
			if(imu.cnt>=1000 && imu.cnt<6000)				
			{			
	    int i;
     	for(i = 0; i<=2; i++)
	    imu.gyro_zero[i]+=imu.gyro[i];		
			}			
		 for(ii = 0; ii<=2; ii++)
	  imu.gyro_buff[ii] = imu.gyro[ii] - imu.gyro_zero[ii]/4999;			
			if(imu.cnt==6000&&imu.flash_read_flag==0)
			{
			STMFLASH_Write(0X08040000,(u32*)imu.gyro_zero,3); 
			imu.flash_read_flag++;
			}	     
		}
			else if(imu.flash_flag)
    {			
      imu.cnt=6000;			
			if (!imu.flash_read_flag)
			{	
				imu.flash_read_flag=1;				
				STMFLASH_Read(0X08020000,(u32*)imu.gyro_zero,3);
				if(imu.gyro_zero[1]==0)
        {
				 imu.gyro_zero[0]=5.04292965;
				 imu.gyro_zero[1]=-4.81709909;
				 imu.gyro_zero[2]=11.9809351;
				}
      for(ii = 0; ii<=2; ii++)
	    imu.gyro_buff[ii] = imu.gyro[ii] - imu.gyro_zero[ii]/4999;
			}
			else 
			{
				for(ii = 0; ii<=2; ii++)
	      imu.gyro_buff[ii] = imu.gyro[ii] - imu.gyro_zero[ii]/4999;
			}		
     }			
		 if(imu.cnt==6000)
		 {
		   Imu_task();		 
		   imu.yaw[1]=imu.yaw[0];		 		  
       QuaternionToEularAngle(imu.q,&imu.yaw[0],&imu.pitch,&imu.roll);
			 float diff=imu.yaw[0]-imu.yaw[1];
			 if(diff<= -150)
			 imu.yaw_cnt++;
			 else if(diff>= 150)
			 imu.yaw_cnt--;
			 imu.yaw[2]=imu.yaw[0]+imu.yaw_cnt*360.0f;
			 yaw_accel = imu.gyro_buff[2]*10;
			 pitch_accel = imu.gyro_buff[0]*10;
		}
    osDelay(1);
  }
  /* USER CODE END ins_task */
}


void imu_calculate_madgwick(void)
{
MadgwickAHRSupdateIMU(imu.gyro_buff[0],imu.gyro_buff[1],imu.gyro_buff[2],imu.acc[0],imu.acc[1],imu.acc[2]);
}

const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};

uint32_t INS_DWT_Count = 0;
static float dt = 0, t = 0;
uint8_t ins_debug_mode = 0;
float RefTemp = 40;



// axis=sqrt(a2+b2+c2)/|a^2+b^2+c^2|;

// ʹ�ü��ٶȼƵ����ݳ�ʼ��Roll��Pitch,��Yaw��0,�������Ա����ڳ�ʼʱ�����̬�������
static void InitQuaternion(float* init_q4) {
  float acc_init[3] = {0};
  float gravity_norm[3] = {0, 0, 1};  // ����ϵ�������ٶ�ʸ��,��һ����Ϊ(0,0,1)
  float axis_rot[3] = {0};  // ��ת��
  // ��ȡ100�μ��ٶȼ�����,ȡƽ��ֵ��Ϊ��ʼֵ
  for (uint8_t i = 0; i < 100; ++i) {
	 BMI088_read(imu.gyro,imu.acc,&(imu.temp));
    acc_init[0] += imu.acc[0];
    acc_init[1] += imu.acc[1];
    acc_init[2] += imu.acc[2];
    DWT_Delay(0.001);
  }
  for (uint8_t i = 0; i < 3; ++i)
    acc_init[i] /= 100;
	float angle;
  Norm3d(acc_init);
//  // ����ԭʼ���ٶ�ʸ���͵���ϵ�������ٶ�ʸ���ļн�
   angle = 0;//acosf(Dot3d(acc_init, gravity_norm));
   Cross3d(acc_init, gravity_norm, axis_rot);
   Norm3d(axis_rot);
  init_q4[0] = cosf(angle / 2.0f);
  for (uint8_t i = 0; i < 2; ++i)
    init_q4[i + 1] =
        axis_rot[i] * sinf(angle / 2.0f);  // ��ǹ�ʽ,������Ϊ0(û��z�����)
}


void INS_Init(void) {
  float init_quaternion[4] = {0};
  InitQuaternion(init_quaternion);
  IMU_QuaternionEKF_Init(10, 0.001, 10000000, 1, 0.001f,0); //ekf��ʼ��
  // imu heat init
//  INS.AccelLPF = 0.0085f;
//  INS.DGyroLPF = 0.008f;
}
/**
?* @brief ? ? ? ? ?Transform 3dvector from BodyFrame to EarthFrame
?* @param[1] ? ? ? vector in BodyFrame
?* @param[2] ? ? ? vector in EarthFrame
?* @param[3] ? ? ? quaternion
?*/
void BodyFrameToEarthFrame(const float* vecBF, float* vecEF, float* q) {
 vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
(q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
(q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
(0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
(q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
(q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
(0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
?* @brief ? ? ? ? ?Transform 3dvector from EarthFrame to BodyFrame
?* @param[1] ? ? ? vector in EarthFrame
?* @param[2] ? ? ? vector in BodyFrame
?* @param[3] ? ? ? quaternion
?*/
void EarthFrameToBodyFrame(const float* vecEF, float* vecBF, float* q) {
 vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
(q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
(q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

 vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
(0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
(q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

 vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
(q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
(0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}


void Imu_task(void) {
	static int i;
	if(!i)
	{
	INS_Init();
	i++;
	}
	
	imu.AccelLPF=0.0085f;
  static uint32_t count = 0;
  const float gravity[3] = {0, 0, 9.74884605f};
  dt = DWT_GetDeltaT(&INS_DWT_Count);

    // ���ĺ���,EKF������Ԫ��
    IMU_QuaternionEKF_Update(imu.gyro_buff[0],imu.gyro_buff[1],imu.gyro_buff[2],
                             imu.acc_fliter[0],imu.acc_fliter[1],imu.acc_fliter[2]);

    memcpy(imu.q, QEKF_INS.q, sizeof(QEKF_INS.q));
	
	  imu.yaw[2]=QEKF_INS.YawTotalAngle;

//    // ����ϵ������ת������������ϵ������ѡȡ����ϵΪ����ϵ
    BodyFrameToEarthFrame(xb, imu.xn, imu.q);
    BodyFrameToEarthFrame(yb, imu.yn, imu.q);
    BodyFrameToEarthFrame(zb, imu.zn, imu.q);

//    // �������ӵ�������ϵnת��������ϵb,�����ݼ��ٶȼ����ݼ����˶����ٶ�
    float gravity_b[3];
    EarthFrameToBodyFrame(gravity, gravity_b, imu.q);
    for (uint8_t i = 0; i < 3; i++)  // ͬ����һ����ͨ�˲�
    {
      imu.MotionAccel_b[i] =
          (imu.acc_fliter[i] - gravity_b[i]) * dt / (imu.AccelLPF + dt) +
          imu.MotionAccel_b[i] * imu.AccelLPF / (imu.AccelLPF + dt);
    }
    BodyFrameToEarthFrame(imu.MotionAccel_b, imu.MotionAccel_n,
                          imu.q);  // ת���ص���ϵn

//    // ��ȡ��������
//    INS.Yaw = QEKF_INS.Yaw;
//    INS.Pitch = QEKF_INS.Pitch;
//    INS.Roll = QEKF_INS.Roll;
//    INS.YawTotalAngle = QEKF_INS.YawTotalAngle;
  

}

void QuaternionToEularAngle(float* q, float* Yaw, float* Pitch, float* Roll) 
{
*Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]),
2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) *
57.295779513f;
*Pitch = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]),
 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) *
57.295779513f;
*Roll = asinf(2.0f * (q[0] * q[2] - q[1] * q[3])) * 57.295779513f;
}











