#include "motor.h"
#include "main.h"
#include "ui_protocol.h"
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern ext_shoot_data_t		shoot_data;					//0x0207
extern ext_robot_status_t		robot_state;				//0x0201
extern ext_power_heat_data_t		  				power_heat_data;				//0x0202
extern client_info_t client_info;
super_energy super_cap;
//motor data read
uint8_t data[8];
motor_measure_t YAW_Motor;
motor_measure_t f_Motor;
motor_measure_t CHISSIS_M[4];
RC_ctrl_t rc_ctrl;
int FLAG=0;
motor_measure_t rem_f_motor;
measure_t chassis;

/**
  * @brief         can报文解包函数
  * @author        Z&S
  * @param[in]     保存数据用结构体指针
  * @param[in]     can报文原始数据
  * @retval        解包完成后结构体指针
  */
void get_motor_measure(motor_measure_t*motor,uint8_t*data)                                    
    {                                                                   
				(motor)->last_raw_ecd = (motor)->raw_ecd;                                  
        (motor)->raw_ecd = (uint16_t)((data)[0] << 8 | (data)[1])-rem_f_motor.raw_ecd;            
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
				(motor)->ecd_angle = ((motor)->raw_ecd*0.0439506775729459F+((motor)->round_count*360));
				(motor)->real_speed = (motor->speed_rpm)/14;
				motor ->cnts+=1;
			}

/**
  * @brief         can中断回调函数
  * @author        Z&S
  * @param[in]     can中断函数自动调用
  * @param[in]     can句柄
  * @retval        进入can数据解包数据
  */
static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];
CAN_RxHeaderTypeDef rx_header;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
   // CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    switch (rx_header.StdId)
    {
        case CAN_3508_M1_ID:							get_motor_measure(&CHISSIS_M[0],rx_data);break;
        case CAN_3508_M2_ID:							get_motor_measure(&CHISSIS_M[1],rx_data);break;
        case CAN_3508_M3_ID:							get_motor_measure(&CHISSIS_M[2],rx_data);break;
        case CAN_3508_M4_ID:							get_motor_measure(&CHISSIS_M[3],rx_data);break;
				case CAN_YAW_MOTOR_ID:						get_motor_measure(&YAW_Motor,rx_data);break;
				case GIMBAL_ID:										get_measure(&chassis,rx_data);break;
				case CAP_ID:                      get_cap(&super_cap,rx_data);break;
        default:
        {
            break;
        }
    }
}

/**
  * @brief         发送电机电流
  * @author        Z&S
  * @param[in]     电机电流值
  * @retval       
  */
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
	if(chassis.mode==6)
	{motor1=0;motor2=0;motor3=0;motor4=0;}
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x200;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
  * @brief         发送云台数据
  * @author        Z&S
  * @param[in]     己方颜色
  * @param[in]     当前射速
  * @retval        
  */
void send_to_gimbal(uint16_t self_color,uint16_t bullet_speed)
{
		uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x103;//
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = (uint8_t)self_color >> 8;
    chassis_can_send_data[1] = (uint8_t)self_color;
		chassis_can_send_data[2] = (uint8_t)power_heat_data.shooter_heat0>>8;
		chassis_can_send_data[3] = (uint8_t)power_heat_data.shooter_heat0;
		chassis_can_send_data[4] = (uint8_t)robot_state.shooter_barrel_cooling_value >> 8;
    chassis_can_send_data[5] = (uint8_t)robot_state.shooter_barrel_cooling_value ;
		chassis_can_send_data[6] = (uint8_t)robot_state.shooter_barrel_heat_limit>>8;
		chassis_can_send_data[7] = (uint8_t)robot_state.shooter_barrel_heat_limit;
    HAL_CAN_AddTxMessage(&GIMBAL_CAN , &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
  * @brief         发送给超电数据
  * @author        Z&S
  * @param[in]     当前底盘功率上线
  * @param[in]     缓冲能量实时值
	* @param[in]     底盘供电状态
  * @retval        
  */
uint16_t buff;
void send_to_capicity()
{
		uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x102;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 1;//chassis.super_mode;
    chassis_can_send_data[1] = (uint8_t)(power_heat_data.chassis_power_buffer >> 8);
    chassis_can_send_data[2] = (uint8_t)(power_heat_data.chassis_power_buffer);
    chassis_can_send_data[3] =robot_state.chassis_power_limit;
    chassis_can_send_data[4] =robot_state.power_management_chassis_output;
		chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN , &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
  * @brief         解包云台下发数据
  * @author        Z&S
  * @param[in]     存放数据结构体指针
  * @param[in]     原始数据数组
  * @retval        结构体指针
  */
int send_to_gimbal_flag=0;
void get_measure(measure_t*chassis,uint8_t*data)                                    
    {               
        (chassis)->x_set = (int16_t)((data)[0] << 8 | (data)[1]);            
        (chassis)->y_set = -(int16_t)((data)[2] << 8 | (data)[3]);
        (chassis)->temp_mode = (data)[4]; 
				(chassis)->rc_gim_super = (data)[5];
				(chassis)->vision_flag = (data)[6];
				send_to_gimbal(client_info.self_color,shoot_data.bullet_speed);send_to_gimbal_flag=1;//放在这里发送是为了降低发送频率，避免云台can线阻塞导致云台或底盘失控
		}

/**
  * @brief         解包超电发放数据
  * @author        Z&S
  * @param[in]     存放数据结构体指针
  * @param[in]     原始数据数组
  * @retval        结构体指针
  */
void get_cap(super_energy*cap,uint8_t*data)                                    
  {                                                                                             
     (cap)->Voltage = (uint16_t)((data)[0] << 8 | (data)[1]);
		 (cap)->ret_power = (uint16_t)((data)[2]<<8|(data)[3]);
	}
/**
  * @brief         解包云台下发的键盘数据
  * @author        Z&S
  * @param[in]     存放数据结构体指针
  * @param[in]     原始数据数组
  * @retval        结构体指针
  */
void worke_out_rc_gim_super(void)
{
	switch(chassis.rc_gim_super)//
			{
			case 0x10 :   {chassis.gim_mode=1;chassis.super_mode=0;chassis.rc_mode=0;break;}//B
			case 0x11 :   {chassis.gim_mode=1;chassis.super_mode=1;chassis.rc_mode=0;break;}
			case 0x20 :   {chassis.gim_mode=2;chassis.super_mode=0;chassis.rc_mode=0;break;}//C
			case 0x21 :   {chassis.gim_mode=2;chassis.super_mode=1;chassis.rc_mode=0;break;}	
			case 0x30 :   {chassis.gim_mode=3;chassis.super_mode=0;chassis.rc_mode=0;break;}
			case 0x31 :   {chassis.gim_mode=3;chassis.super_mode=1;chassis.rc_mode=0;break;}//A
			case 4 :   		{chassis.gim_mode=0;chassis.super_mode=0;chassis.rc_mode=1;break;}
			default :  {break;}
			}
}
void worke_out_chassis_mode(void)
{
	switch(chassis.temp_mode)//底盘模式选择
	{
		case 40 : {chassis.mode=4;break;}
		case 41 : {chassis.mode=5;break;}
		case 5 : {chassis.mode=7;break;}
		case 6 : {chassis.mode=6;break;}
		case 43: {chassis.mode=8;break;}
	}
}
void worke_out_vision_flag(void)
{
	switch(chassis.vision_flag)//底盘模式选择
	{
		case 10 : {chassis.aim_ok_flag=0;chassis.feed_frequency=1;break;}
		case 20 : {chassis.aim_ok_flag=0;chassis.feed_frequency=2;break;}
		case 11 : {chassis.aim_ok_flag=1;chassis.feed_frequency=1;break;}
		case 21 : {chassis.aim_ok_flag=1;chassis.feed_frequency=2;break;}
		case 12 : {chassis.aim_ok_flag=2;chassis.feed_frequency=1;break;}
		case 22 : {chassis.aim_ok_flag=2;chassis.feed_frequency=2;break;}
	}
}

