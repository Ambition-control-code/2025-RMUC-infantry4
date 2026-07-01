/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             ÕâÀïÊÇCANÖĞ¶Ï½ÓÊÕº¯Êı£¬½ÓÊÕµç»úÊı¾İ,CAN·¢ËÍº¯Êı·¢ËÍµç»úµçÁ÷¿ØÖÆµç»ú.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef CAN_RECEIVE_H
#define CAN_RECEIVE_H

#include "struct_typedef.h"

#define CAN_2 hcan2
#define CAN_1 hcan1

/* CAN send and receive ID */
typedef enum
{
    CAN_FrictioL_Motor_ID = 0x201,
    CAN_FrictioR_Motor_ID = 0x202,
    CAN_Feed_Motor = 0x203,
    CAN_YAW_MOTOR_ID = 0x206,	
    CAN_PITCH_MOTOR_ID = 0x205,
		GIMBAL_ID_1=0x1FF,
		GIMBAL_ID_2=0x200,
		CHASSIS_ID= 0x103,
} can_msg_id_e;

//rm motor data
typedef struct
{
		int32_t round_count;//ç”µæœºåœˆæ•°
		int32_t differ;//ä¸¤æ¬¡ç”µæœºåˆ»åº¦å·®
		int32_t raw_ecd;//ç”µæœºåˆ»åº¦åŸå§‹å€¼
		int32_t speed_rpm;//ç”µæœºè½¬é€Ÿ
    int16_t given_current;//åé¦ˆç”µæµ
		uint8_t temperate;//ç”µæœºæ¸©åº¦
		int32_t last_raw_ecd;//ç”µæœºä¸Šæ¬¡åŸå§‹åˆ»åº¦å€¼
		int32_t real_ecd;//ç”µæœºçœŸå®åˆ»åº¦
		int32_t total_real_ecd;//ç”µæœºçœŸå®ä½ç½®
		float ecd_angle;//ç”µæœºè§’åº¦
		int32_t speed_buf[6];
		int32_t buf_count;
		int32_t filter_sum;
		int32_t filter_speed;
} motor_measure_t;

typedef struct
{
		int16_t self_color;
		uint16_t Q_now;
		uint16_t Q_cd;
		uint16_t Q_max;
} measure_chassis;

void get_motor_measure(motor_measure_t*motor,uint8_t*data);
void motor_control(void);
float juedui(float a);
extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);
extern const motor_measure_t *get_trigger_motor_measure_point(void);
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);
void get_motor_measure(motor_measure_t*ptr,uint8_t*data) ;
void send_to_frictio_motor(int16_t fri_l,int16_t fri_r,int16_t feed);
void send_to_gimbal_motor(int16_t yaw,int16_t pitch);
void send_to_chassis(int16_t x,int16_t y,int8_t chssis_mode,int8_t rc_mode,int8_t vision_flag);
void get_measure(measure_chassis*chassis,uint8_t*data);   
#endif



