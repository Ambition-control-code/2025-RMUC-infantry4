#ifndef REMOTE_CONTROL_H
#define REMOTE_CONTROL_H
#include "struct_typedef.h"
#include "bsp_rc.h"

#define SBUS_RX_BUF_NUM 36u

#define RC_FRAME_LENGTH 18u

#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)
/* ----------------------- Data Struct ------------------------------------- */
typedef struct
{
  int16_t R_x;  //右手x		右1684 左364 中1024  1684-1024 = 1024 - 364 = 660
	int16_t R_y;	 //右手y    上1684 下364 中1024
	int16_t L_x;
	int16_t L_y;
	uint8_t sl;			//拨动左  上1 中3 下2
	uint8_t sr;
	uint8_t last_sr;
	int16_t mouse_x;	//鼠标
	int16_t mouse_y;
	int16_t mouse_z;
	uint8_t mouse_l;	//鼠标按键
	uint8_t mouse_r;
	uint16_t key;		//键盘
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
	int16_t lun;
	
	
	uint64_t cnts;
	uint64_t off_line_times;
	uint64_t last_cnts;
	uint64_t now_cnts;
	uint8_t off_line_flag;     
	
} RC_ctrl_t;

/* ----------------------- Internal Data ----------------------------------- */

/**
  * @brief          remote control init
  * @param[in]      none
  * @retval         none
  */
/**
  * @brief          遥控器初始化
  * @param[in]      none
  * @retval         none
  */
extern void remote_control_init(void);
/**
  * @brief          get remote control data point
  * @param[in]      none
  * @retval         remote control data point
  */
/**
  * @brief          获取遥控器数据指针
  * @param[in]      none
  * @retval         遥控器数据指针
  */
extern  RC_ctrl_t *get_remote_control_point(void);
void sbus_to_rc(uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl);

#endif
