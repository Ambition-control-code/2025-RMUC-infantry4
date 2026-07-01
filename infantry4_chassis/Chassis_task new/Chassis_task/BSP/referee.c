#include "referee.h"
#include "crc.h"
#include "string.h"
#if JUDGE_VERSION == JUDGE_24

/*****************系统数据定义**********************/
ext_game_state_t       						game_state;					//0x0001
ext_game_result_t            			game_result;					//0x0002
ext_game_robot_HP_t          			robot_hp;			//0x0003
ext_event_data_t        					event_data;					//0x0101
ext_supply_projectile_action_t		supply_projectil_action;		//0x0102
ext_referee_warning_t							juge_data;	//0x0103
ext_robot_status_t			  				robot_state;				//0x0201
ext_power_heat_data_t		  				power_heat_data;				//0x0202
ext_game_robot_pos_t							robot_pos;				//0x0203
ext_buff_musk_t										buff_musk;					//0x0204
ext_hurt_data_t										robot_hurt;					//0x0206
ext_shoot_data_t									shoot_data;					//0x0207

xFrameHeader              				FrameHeader;		//发送帧头信息
//ext_SendClientData_t      				ShowData;			//客户端信息
//ext_CommunatianData_t    				 	CommuData;		//队友通信信息
/****************************************************/

//bool Judge_Data_TF = FALSE;//裁判数据是否可用,辅助函数调用
uint8_t robot_self_ID;//当前机器人的ID
uint16_t robot_client_ID;//发送者机器人对应的客户端ID


/**************裁判系统数据辅助****************/
//uint16_t ShootNum;//统计发弹量,0x0003触发一次则认为发射了一颗
//bool Hurt_Data_Update = FALSE;//装甲板伤害数据是否更新,每受一次伤害置TRUE,然后立即置FALSE,给底盘闪避用
#define BLUE  0
#define RED   1
int C_mdID = 0;//数据命令码解析
	/**
  * @brief         读取裁判数据,中断中读取保证速度
  * @author        Z&S
  * @retval        是否对正误判断做处理
  */
bool_t judge_read_data(uint8_t *ReadFromUsart)
{
	bool_t retval_tf = FALSE;//数据正确与否标志,每次调用读取裁判系统数据函数都先默认为错误
	
	uint16_t judge_length;//统计一帧数据长度 
	
	int CmdID = 0;//数据命令码解析
	
	/***------------------*****/
	//无数据包，则不作任何处理
	if (ReadFromUsart == NULL)
	{
		return -1;
	}
	
	//写入帧头数据,用于判断是否开始存储裁判数据
	//memcpy(&FrameHeader, ReadFromUsart, LEN_HEADER);
	
	//判断帧头数据是否为0xA5
	if(ReadFromUsart[ SOF ] == JUDGE_FRAME_HEADER)
	{
		//帧头CRC8校验
		if (Verify_CRC8_Check_Sum( ReadFromUsart, LEN_HEADER ) == TRUE)
		{
			//统计一帧数据长度,用于CR16校验
			judge_length = ReadFromUsart[ DATA_LENGTH ] + LEN_HEADER + LEN_CMDID + LEN_TAIL;;

			//帧尾CRC16校验
			if(Verify_CRC16_Check_Sum(ReadFromUsart,judge_length) == TRUE)
			{
				retval_tf = TRUE;//都校验过了则说明数据可用
				
				CmdID = (ReadFromUsart[6] << 8 | ReadFromUsart[5]);
				//解析数据命令码,将数据拷贝到相应结构体中(注意拷贝数据的长度)
				C_mdID=CmdID;
				switch(CmdID)
				{
					case ID_game_state:        			//0x0001
						memcpy(&game_state, (ReadFromUsart + DATA), LEN_game_state);
					break;
					
					case ID_game_result:          		//0x0002
						memcpy(&game_result, (ReadFromUsart + DATA), LEN_game_result);
					break;
					
					case ID_game_robot_survivors:       //0x0003
						memcpy(&robot_hp, (ReadFromUsart + DATA), LEN_game_robot_survivors);
					break;
					
					case ID_event_data:    				//0x0101
						memcpy(&event_data, (ReadFromUsart + DATA), LEN_event_data);
					break;
					
					case ID_supply_projectile_action:   //0x0102
						memcpy(&supply_projectil_action, (ReadFromUsart + DATA), LEN_supply_projectile_action);
					break;
					
					case ID_penalize_data:  //0x0103
						memcpy(&juge_data, (ReadFromUsart + DATA), LEN_penalize_data);
					break;
					
					case ID_game_robot_state:      		//0x0201
						memcpy(&robot_state, (ReadFromUsart + DATA), LEN_game_robot_state);
					break;
					
					case ID_power_heat_data:      		//0x0202
						memcpy(&power_heat_data, (ReadFromUsart + DATA), LEN_power_heat_data);
					break;
					
					case ID_game_robot_pos:      		//0x0203
						memcpy(&robot_pos, (ReadFromUsart + DATA), LEN_game_robot_pos);
					break;
					
					case ID_buff_musk:      			//0x0204
						memcpy(&buff_musk, (ReadFromUsart + DATA), LEN_buff_musk);
					break;
					
					case ID_robot_hurt:      			//0x0206
						memcpy(&robot_hurt, (ReadFromUsart + DATA), LEN_robot_hurt);
						if(robot_hurt.HP_deduction_reason == 0)//非装甲板离线造成伤害
						//{	Hurt_Data_Update = TRUE;	}//装甲数据每更新一次则判定为受到一次伤害
					break;
					
					case ID_shoot_data:      			//0x0207
						memcpy(&shoot_data, (ReadFromUsart + DATA), LEN_shoot_data);
						//JUDGE_ShootNumCount();//发弹量统
					break;
					default:break;
				}
			}
		}
				if(ReadFromUsart[judge_length] == JUDGE_FRAME_HEADER)//出现多帧再次读取
				{judge_read_data(&ReadFromUsart[judge_length]);}
	}
	
	return retval_tf;//对数据正误做处理
}
#elif JUDGE_VERSION == JUDGE_25



#endif













