#ifndef __REFEREE_H
#define __REFEREE_H
#include "main.h"
#define FALSE 0
#define TRUE  1
#if JUDGE_VERSION == JUDGE_24

#define    LEN_HEADER    5        //帧头长
#define    LEN_CMDID     2        //命令码长度
#define    LEN_TAIL      2	      //帧尾CRC16


//起始字节,协议固定为0xA5
#define    JUDGE_FRAME_HEADER         (0xA5)
//裁判系统通信格式字节偏移量
typedef enum 
{
	FRAME_HEADER         = 0,
	CMD_ID               = 5,
	DATA                 = 7,
	
}JudgeFrameOffset;

//5字节帧头,偏移位置
typedef enum
{
	SOF          = 0,//起始位
	DATA_LENGTH  = 1,//帧内数据长度,根据这个来获取数据长度
	SEQ          = 3,//包序号
	CRC8         = 4 //CRC8
	
}FrameHeaderOffset;

/***************命令码ID********************/

/* 

	ID: 0x0001  Byte:  3/11    比赛状态数据       			发送频率 1Hz      
	ID: 0x0002  Byte:  1    比赛结果数据         		比赛结束后发送      
	ID: 0x0003  Byte:  32    比赛机器人血量数据   		3Hz发送 /服务器->全体机器人 
	ID: 0x0101  Byte:  4    场地事件数据   				事件改变后发送 1HZ
	ID: 0x0102  Byte:  3/4    场地补给站动作标识数据    	动作改变后发送 
	ID: 0X0103  Byte:  2    场地补给站预约子弹数据      参赛队发送，10Hz 
	ID: 0X0201  Byte: 15    机器人状态数据        		10Hz
	ID: 0X0202  Byte: 14    实时功率热量数据   			50Hz       
	ID: 0x0203  Byte: 16    机器人位置数据           	10Hz
	ID: 0x0204  Byte:  1    机器人增益数据           	增益状态改变后发送
	ID: 0x0205  Byte:  3    空中机器人能量状态数据      10Hz
	ID: 0x0206  Byte:  1    伤害状态数据           		伤害发生后发送
	ID: 0x0207  Byte:  6    实时射击数据           		子弹发射后发送
	ID: 0x0301  Byte:  n    机器人间交互数据           	发送方触发发送,10Hz
	
*/
/*
	ID: 0x0001  Byte:  11    比赛状态数据       			发送频率 1Hz      
	ID: 0x0002  Byte:  1    比赛结果数据         		比赛结束后发送      
	ID: 0x0003  Byte:  32    比赛机器人血量数据   		3Hz发送 /服务器->己方全体机器人 
	ID: 0x0101  Byte:  4    场地事件数据   				事件改变后发送 1HZ
	ID: 0x0102  Byte:  4    场地补给站动作标识数据    	动作改变后发送 
	ID: 0X0104  Byte:  3    裁判系统判罚数据      服务器->被判罚方全体机器人 
	ID: 0X0105  Byte:  3    飞镖发射相关数据       
	ID: 0X0201  Byte: 13    机器人性能体系数据        		10Hz
	ID: 0X0202  Byte: 16    实时功率热量数据   			50Hz       
	ID: 0x0203  Byte: 16    机器人位置数据           	1Hz
	ID: 0x0204  Byte:  6    机器人增益数据           	3HZ
	ID: 0x0206  Byte:  1    伤害状态数据           		伤害发生后发送
	ID: 0x0207  Byte:  6    实时射击数据           		子弹发射后发送
	ID: 0x0301  Byte:  n    机器人间交互数据           	发送方触发发送,10Hz
*/

//命令码ID,用来判断接收的是什么数据
typedef enum
{ 
	ID_game_state       							= 0x0001,//比赛状态数据
	ID_game_result 	   								= 0x0002,//比赛结果数据
	ID_game_robot_survivors       		= 0x0003,//比赛机器人存活数据
	ID_event_data  										= 0x0101,//场地事件数据 
	ID_supply_projectile_action   		= 0x0102,//场地补给站动作标识数据
	ID_penalize_data									= 0x0104,//裁判系统判罚数据
	ID_datas_data											= 0x0105,//飞镖数据
	ID_game_robot_state    						= 0x0201,//机器人性能体系数据
	ID_power_heat_data    						= 0x0202,//实时功率热量数据
	ID_game_robot_pos        					= 0x0203,//机器人位置数据
	ID_buff_musk											= 0x0204,//机器人增益数据
	ID_robot_hurt											= 0x0206,//伤害状态数据
	ID_shoot_data											= 0x0207,//实时射击数据

} CmdID;


//命令码数据段长,根据官方协议来定义长度
typedef enum
{
	LEN_game_state       							= 11,	//11,	//0x0001
	LEN_game_result       						= 1,	//0x0002
	LEN_game_robot_survivors       		= 32,	//32	//0x0003
	LEN_event_data  									= 4,	//0x0101
	LEN_supply_projectile_action      = 4,	//4//0x0102
	LEN_penalize_data									= 3,	//0x0104
	LEN_datas_data										= 3,	//0x105
	LEN_game_robot_state    					= 13,	//0x0201
	LEN_power_heat_data   						= 16,	//0x0202
	LEN_game_robot_pos        				= 16,	//0x0203
	LEN_buff_musk        							= 6,	//0x0204
	LEN_aerial_robot_energy        		= 2,	//0x0205
	LEN_robot_hurt        						= 1,	//0x0206
	LEN_shoot_data       							= 7,	//0x0207
	
} JudgeDataLength;

/* 自定义帧头 */
typedef __packed struct
{
	uint8_t  SOF;//1
	uint16_t DataLength;//2
	uint8_t  Seq;//1
	uint8_t  CRC8;//1
	
} xFrameHeader;

/* ID: 0x0001  Byte:  3    比赛状态数据 */
typedef __packed struct 
{ 
	uint8_t game_type : 4;
	uint8_t game_progress : 4;
	uint16_t stage_remain_time;
} ext_game_state_t; 


/* ID: 0x0002  Byte:  1    比赛结果数据 */
typedef __packed struct 
{ 
	uint8_t winner;
} ext_game_result_t; 


/* ID: 0x0003  Byte:  2    比赛机器人血量数据 */
typedef __packed struct 
{ 
	uint16_t red_1_robot_HP; 
 uint16_t red_2_robot_HP; 
 uint16_t red_3_robot_HP; 
 uint16_t red_4_robot_HP; 
 uint16_t red_5_robot_HP; 
 uint16_t red_7_robot_HP; 
 uint16_t red_outpost_HP; 
 uint16_t red_base_HP; 
 uint16_t blue_1_robot_HP; 
 uint16_t blue_2_robot_HP; 
 uint16_t blue_3_robot_HP; 

 uint16_t blue_4_robot_HP; 
 uint16_t blue_5_robot_HP; 
 uint16_t blue_7_robot_HP; 
 uint16_t blue_outpost_HP; 
 uint16_t blue_base_HP; 
}ext_game_robot_HP_t;



/* ID: 0x0101  Byte:  4    场地事件数据 */
typedef __packed struct 
{ 
	uint32_t event_type;
} ext_event_data_t; 


/* ID: 0x0102  Byte:  3    场地补给站动作标识数据 */
typedef __packed struct 
{ 
 uint8_t reserved; 
 uint8_t supply_robot_id; 
 uint8_t supply_projectile_step; 
 uint8_t supply_projectile_num; 
} ext_supply_projectile_action_t;
/* ID: 0X0103  */
typedef __packed struct 
{ 
 uint8_t level; 
 uint8_t offending_robot_id; 
 uint8_t count; 
}ext_referee_warning_t;


/* ID: 0X0201  Byte: 15    机器人状态数据 */
typedef __packed struct 
{ 
 uint8_t robot_id; 
 uint8_t robot_level; 
 uint16_t current_HP; 
 uint16_t maximum_HP; 
 uint16_t shooter_barrel_cooling_value; 
 uint16_t shooter_barrel_heat_limit; 
 uint16_t chassis_power_limit; 
 uint8_t power_management_gimbal_output : 1; 
 uint8_t power_management_chassis_output : 1; 
 uint8_t power_management_shooter_output : 1; 
}ext_robot_status_t;


/* ID: 0X0202  Byte: 14    实时功率热量数据 */
typedef __packed struct 
{ 
	uint16_t chassis_volt;   
	uint16_t chassis_current;    
	float chassis_power;   //瞬时功率 
	uint16_t chassis_power_buffer;//60焦耳缓冲能量
	uint16_t shooter_heat0;//17mm
	uint16_t shooter_heat1;  
} ext_power_heat_data_t; 


/* ID: 0x0203  Byte: 16    机器人位置数据 */
typedef __packed struct 
{   
	float x;   
	float y;   
	float angle; 
} ext_game_robot_pos_t; 


/* ID: 0x0204  Byte:  1    机器人增益数据 */
typedef __packed struct 
{ 
	uint8_t recovery_buff; 
	uint8_t cooling_buff; 
	uint8_t defence_buff; 
	uint8_t vulnerability_buff; 
 uint16_t attack_buff;
} ext_buff_musk_t; 
/* ID: 0x0206  Byte:  1    伤害状态数据 */
 
typedef __packed struct 
{ 
 uint8_t armor_id : 4; 
 uint8_t HP_deduction_reason : 4; 
}ext_hurt_data_t;



/* ID: 0x0207  Byte:  6    实时射击数据 */
typedef __packed struct 
{ 
	uint8_t bullet_type;   
	uint8_t bullet_freq;
	uint8_t shooter_number;
	float bullet_speed;  
} ext_shoot_data_t; 
#endif
bool_t judge_read_data(uint8_t *ReadFromUsart);
#endif