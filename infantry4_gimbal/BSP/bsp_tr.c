#include "bsp_tr.h"
#include "main.h"
#include "remote_control.h"
#include "tele_link.h"
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern RC_ctrl_t rc_ctrl;//遥控器数据结构体
//motor data read
uint8_t data[8];
motor_measure_t Pitch_Motor;//pitch电机
motor_measure_t Yaw_Motor;//yaw电机
motor_measure_t	FrictioL_Motor;//左摩擦轮
motor_measure_t	FrictioR_Motor;//右摩擦轮
motor_measure_t	Feed_Motor;//供弹电机
measure_chassis chassis_data;
int FLAG=0;//电机上电标志位，用于消除电机转动后刻度
float rem_ecd;//保存电机上电刻度
/**
  * @brief          can报文解算函数
  * @author         Z&S
  * @param[in]      保存数据用结构体
  * @param[in]      can报文原始数组
  * @retval         报文解算完后结构体数据
  */
void get_motor_measure(motor_measure_t*motor,uint8_t*data)                                    
{                                                                   
		(motor)->last_raw_ecd = (motor)->raw_ecd;                                  
    (motor)->raw_ecd = (uint16_t)((data)[0] << 8 | (data)[1])-rem_ecd;            
    (motor)->speed_rpm = (int16_t)((data)[2] << 8 | (data)[3]);      
    (motor)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  
    (motor)->temperate = (data)[6];
		(motor)->differ = (motor)->raw_ecd-(motor)->last_raw_ecd;
		if((motor)->differ < -4096)
			{
				(motor)->round_count++;
				(motor)->real_ecd = (motor)->differ+8191;
			}
			else if((motor)->differ > 4096)
			{
				(motor)->round_count--;
				(motor)->real_ecd = (motor)->differ-8191;
			}
			else
			{
					(motor)->real_ecd = (motor)->differ;
			}
		(motor)->total_real_ecd = (motor)->raw_ecd+(motor)->round_count*8191;
		(motor)->ecd_angle = (((motor)->raw_ecd*360)/8191+((motor)->round_count*360));
}

/**
  * @brief          can中断回调函数
  * @author         Z&S
  * @param[in]      can中断函数自动调用
  * @retval         报文解算完后结构体数据
  */
static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];
 CAN_RxHeaderTypeDef rx_header;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)//can中断回调函数，对收到的报文id进行选择并进行数据解包
{
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    switch (rx_header.StdId)
    {
        case CAN_FrictioL_Motor_ID:				get_motor_measure(&FrictioL_Motor,rx_data);break;
        case CAN_FrictioR_Motor_ID:				get_motor_measure(&FrictioR_Motor,rx_data);break;
        case CAN_Feed_Motor:							get_motor_measure(&Feed_Motor,rx_data);break;
        case CAN_YAW_MOTOR_ID:						get_motor_measure(&Yaw_Motor,rx_data);break;
        case CAN_PITCH_MOTOR_ID:					get_motor_measure(&Pitch_Motor,rx_data);break;
				case CHASSIS_ID:								  get_measure(&chassis_data,rx_data);break;
        default:
				{
					break;
        }
    }
}

/**
  * @brief          发送给底盘c板数据
  * @author         Z&S
  * @param[in]     	x方向移动量，y方向移动量，底盘模式，遥控器状态，超级电容状态，ui状态
  * @retval         
  */
void send_to_chassis(int16_t x,int16_t y,int8_t chssis_mode,int8_t rc_mode,int8_t vision_flag)//发送给底盘
{
	 // uint8_t joint = chssis_mode>>5|rc_mode>>7|power_mode>>7|ui_flag>>7; 
		uint32_t send_mail_box;
    gimbal_tx_message.StdId = 0x101;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] =(uint8_t)(x >> 8);
    gimbal_can_send_data[1] =(uint8_t)x;
    gimbal_can_send_data[2] =(uint8_t)(y >> 8);
    gimbal_can_send_data[3] =(uint8_t)y;
    gimbal_can_send_data[4] =(uint8_t) chssis_mode;
    gimbal_can_send_data[5] =(uint8_t) rc_mode;
    gimbal_can_send_data[6] =(uint8_t)vision_flag;
		gimbal_can_send_data[7] =0;
    HAL_CAN_AddTxMessage(&CAN_1, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

///**
//  * @brief          发送给底盘电容数据
//  * @author         Z&S
//  * @param[in]     	x方向移动量，y方向移动量，键鼠数据
//  * @retval         
//  */
//#include "tele_link.h"	
//extern VT13_ctrl_t VT13_ctrl;

//void send_to_chassis(int16_t x,int16_t y,int8_t chssis_mode,int8_t rc_mode,int8_t vision_flag)//发送给底盘
//{
//		uint32_t send_mail_box;
//    gimbal_tx_message.StdId = 0x101;
//    gimbal_tx_message.IDE = CAN_ID_STD;
//    gimbal_tx_message.RTR = CAN_RTR_DATA;
//    gimbal_tx_message.DLC = 0x08;
//    gimbal_can_send_data[0] =(uint8_t)(x >> 8);
//    gimbal_can_send_data[1] =(uint8_t)x;
//    gimbal_can_send_data[2] =(uint8_t)(y >> 8);
//    gimbal_can_send_data[3] =(uint8_t)y;
//    gimbal_can_send_data[4] =(uint8_t) VT13_ctrl.Ctrl;
//    gimbal_can_send_data[5] =(uint8_t) VT13_ctrl.Q|VT13_ctrl.s_l;
//    gimbal_can_send_data[6] =(uint8_t) VT13_ctrl.E|VT13_ctrl.s_r;
//		gimbal_can_send_data[7] = VT13_ctrl.mode;
//    HAL_CAN_AddTxMessage(&CAN_1, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
//}

/**
	* @brief         发送给云台电机数据
  * @author        Z&S
* @param[in]       yaw轴电机电流值，pitch电机电流值
  * @retval        报文解算完后结构体数据
  */
int16_t pitch_out,yaw_out;
void send_to_gimbal_motor(int16_t yaw,int16_t pitch)//发送给云台的pitch和yaw电机
{
	pitch_out=pitch;yaw_out=yaw;
	if(rc_ctrl.sl==2)
	{
		yaw_out=0;pitch_out=0;
	}
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = 0x1FF;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;
    gimbal_can_send_data[0] =(uint8_t)(pitch_out >> 8);
    gimbal_can_send_data[1] =(uint8_t)pitch_out;
    gimbal_can_send_data[2] =(uint8_t)(yaw_out >> 8);
    gimbal_can_send_data[3] =(uint8_t)yaw_out;
    gimbal_can_send_data[4] =0;
    gimbal_can_send_data[5] =0;
    gimbal_can_send_data[6] =0;
    gimbal_can_send_data[7] =0;
    HAL_CAN_AddTxMessage(&CAN_1, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

/**
	* @brief          发送给发射机构电机数据
  * @author         Z&S
	* @param[in]     	左摩擦轮电流，右摩擦轮电流，拨弹盘电机电流
  * @retval         报文解算完后结构体数据
  */
void send_to_frictio_motor(int16_t fri_l,int16_t fri_r,int16_t feed)//发送给摩擦轮电机
{
	  uint8_t SEND_DATA[8];
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x200;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    SEND_DATA[0] = fri_l >> 8;
    SEND_DATA[1] = fri_l;
    SEND_DATA[2] = fri_r >> 8;
    SEND_DATA[3] = fri_r;
    SEND_DATA[4] = feed >> 8;
    SEND_DATA[5] = feed;

    HAL_CAN_AddTxMessage(&CAN_2, &chassis_tx_message, SEND_DATA, &send_mail_box);
}

/**
  * @brief          解包底盘达妙数据
  * @author         Z&S
  * @param[in]      can中断函数自动调用
  * @retval         报文解算完后结构体数据
  */
void get_measure(measure_chassis*chassis,uint8_t*data)                                    
    {                                                                                             
        (chassis)->self_color = (uint16_t)((data)[0] << 8 | (data)[1]);
				(chassis)->Q_now = (uint16_t)((data)[2] << 8 | (data)[3]);
				(chassis)->Q_cd = (uint16_t)((data)[4] << 8 | (data)[5]);
				(chassis)->Q_max = (uint16_t)((data)[6] << 8 | (data)[7]);  
		}
