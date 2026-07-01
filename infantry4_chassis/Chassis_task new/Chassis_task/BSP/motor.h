/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  *             这里是CAN中断接收函数，接收电机数据,CAN发送函数发送电机电流控制电机.
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

#define CHASSIS_CAN hcan2
#define GIMBAL_CAN hcan1

/* CAN send and receive ID */
typedef enum
{
    CAN_CHASSIS_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,
    CAN_YAW_MOTOR_ID = 0x206,
		GIMBAL_ID        = 0x101,
		CAP_ID              = 0x211,
} can_msg_id_e;

//rm motor data
typedef struct
{
		int32_t round_count;//圈数
		int32_t differ;//两次编码器值的差
    int32_t raw_ecd;//转子机械角度
    int32_t speed_rpm;//转子转速
    int16_t given_current;//电流
    uint8_t temperate;//温度
    int32_t last_raw_ecd;//上次机械角度
		int32_t real_ecd;
		int32_t total_real_ecd;
		float ecd_angle;
		int32_t speed_buf[6];
		int32_t buf_count;
		int32_t filter_sum;
		int32_t filter_speed;
		uint64_t cnts;
		uint64_t last_cnts;
		float real_speed;
} motor_measure_t;

typedef struct
{
	int16_t x_set;//x
	int16_t y_set;//y
	int8_t  mode;//sport mode
	int8_t rc_gim_super;
	uint16_t key;//x
	uint8_t rc_mode;
	uint8_t gim_mode;
	uint8_t super_mode;
	uint8_t vision_flag;
	uint8_t temp_mode;
	int16_t x_setl;
	int16_t y_setl;
	uint8_t aim_ok_flag;
	uint8_t feed_frequency;
}measure_t;
typedef struct
{
	uint16_t Voltage;
	uint16_t ret_power;
}super_energy;


typedef struct
{
	uint8_t W;
	uint8_t S;
	uint8_t A;
	uint8_t D;
	uint8_t Q;
	uint8_t last_Q;
	uint8_t E;
	uint8_t last_E;
	
	uint8_t F;
	uint8_t last_G;
	uint8_t G;
	uint8_t Z;
	uint8_t X;
	uint8_t C;
	uint8_t last_C;
	uint8_t V;
	uint8_t B;
	uint8_t last_B;
	uint8_t Shift;
	uint8_t last_Shift;
	uint8_t Ctrl;
	uint8_t R;
	uint8_t last_R;
	uint8_t last_Ctrl;
	uint8_t last_F;
	
} RC_ctrl_t;








void get_cap(super_energy*cap,uint8_t*data);         
void work_out_key(void);
void worke_out_rc_gim_super(void);
void worke_out_vision_flag(void);
/**
  * @brief          send control current of motor (0x205, 0x206, 0x207, 0x208)
  * @param[in]      yaw: (0x205) 6020 motor control current, range [-30000,30000] 
  * @param[in]      pitch: (0x206) 6020 motor control current, range [-30000,30000]
  * @param[in]      shoot: (0x207) 2006 motor control current, range [-10000,10000]
  * @param[in]      rev: (0x208) reserve motor control current
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x205,0x206,0x207,0x208)
  * @param[in]      yaw: (0x205) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      pitch: (0x206) 6020电机控制电流, 范围 [-30000,30000]
  * @param[in]      shoot: (0x207) 2006电机控制电流, 范围 [-10000,10000]
  * @param[in]      rev: (0x208) 保留，电机控制电流
  * @retval         none
  */
void get_motor_measure(motor_measure_t*motor,uint8_t*data);
void motor_control(void);
float juedui(float a);
extern void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev);

/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          发送ID为0x700的CAN包,它会设置3508电机进入快速设置ID
  * @param[in]      none
  * @retval         none
  */
extern void CAN_cmd_chassis_reset_ID(void);

/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
  * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
/**
  * @brief          发送电机控制电流(0x201,0x202,0x203,0x204)
  * @param[in]      motor1: (0x201) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor2: (0x202) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor3: (0x203) 3508电机控制电流, 范围 [-16384,16384]
  * @param[in]      motor4: (0x204) 3508电机控制电流, 范围 [-16384,16384]
  * @retval         none
  */
extern void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);

/**
  * @brief          return the yaw 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回yaw 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_yaw_gimbal_motor_measure_point(void);

/**
  * @brief          return the pitch 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回pitch 6020电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_pitch_gimbal_motor_measure_point(void);

/**
  * @brief          return the trigger 2006 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
/**
  * @brief          返回拨弹电机 2006电机数据指针
  * @param[in]      none
  * @retval         电机数据指针
  */
extern const motor_measure_t *get_trigger_motor_measure_point(void);

/**
  * @brief          return the chassis 3508 motor data point
  * @param[in]      i: motor number,range [0,3]
  * @retval         motor data point
  */
/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */void send_to_capicity();
extern const motor_measure_t *get_chassis_motor_measure_point(uint8_t i);
void get_motor_measure(motor_measure_t*ptr,uint8_t*data) ;
void get_measure(measure_t*chassis,uint8_t*data);
void send_to_gimbal(uint16_t self_color,uint16_t bullet_speed);
void worke_out_chassis_mode(void);
#endif



