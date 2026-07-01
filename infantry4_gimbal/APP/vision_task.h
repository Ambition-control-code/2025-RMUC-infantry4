#ifndef __VISION_TASK_H
#define __VISION_TASK_H
#include "crc.h"
#include "gimbal_task.h"


typedef __packed struct 
{    
	  uint8_t head;                           //0帧头
	  uint8_t fire;
		float pitch;
		float yaw;
		float distance;	//0红1蓝2红符3蓝符
		uint8_t empty;
    uint8_t rear;                           //26
} receive_struct;




#pragma pack(1)
typedef  struct
{   
		uint8_t head;        
	  uint8_t mode;
		float roll;	//0红1蓝2符
		float pitch;
		float yaw;
		uint8_t empty;
    uint8_t rear;                           //26
} send_struct;
#pragma pack()


void vision_task(void const * argument);
void send_vision_task();
void receive_vision_function(uint8_t *buf);//调用该函数接收数据;
#endif


































































//#pragma pack(1)
//typedef struct 
//{
//  uint8_t header   ;
//  uint8_t tracking : 2;
//  uint8_t id : 3;          // 0-outpost 6-guard 7-base
//  uint8_t armors_num : 3;  // 2-balance 3-outpost 4-normal
//  
//  float x;
//  float y;
//  float z;
//  float yaw;
//	
//  float vx;
//  float vy;
//  float vz;
//  float v_yaw;
//	
//  float r1;
//  float r2;
//  float dz;
//	
//  uint32_t cap_timestamp;
//  uint16_t t_offset;
//  uint16_t checksum ;
//} RV_PacketVision_recieve;
//#pragma pack()


//#pragma pack(1)
//typedef struct
//{
//	uint8_t header;
//	uint8_t detect_color :1; // 0-red 1-blue
//	uint8_t task_mode :2;//0-auto 1-aim 2-buff
//	uint8_t reset_tracker :1;
//	uint8_t is_play : 1;
//	uint8_t change_target : 1;
//	uint8_t reserved :2;
//	float roll;//// 世界坐标系下云台当前的 roll 单位弧度
//	float pitch;// 世界坐标系下云台当前的 pitch
//	float yaw;// 世界坐标系下云台当前的 yaw
//	float aim_x;
//	float aim_y;
//	float aim_z;
//	uint16_t game_time;//(s)game time [o，450]
//	uint32_t timestamp;//(ms)board time
//	uint16_t checksum;   //34
//} RV_PacketVision_send;//
//#pragma pack()

//void receive_vision_task(uint8_t* Buf, uint32_t *Len);
//void vision_task(void const * argument);
//void send_vision_task(uint8_t* Buf, uint16_t Len);
//void recieve_vision_cdc(u8 *temp_data);
//void ST_Init(void);





