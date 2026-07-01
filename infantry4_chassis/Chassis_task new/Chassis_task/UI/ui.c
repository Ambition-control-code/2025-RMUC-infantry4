#include "ui.h"
#include "ui_protocol.h"
#include "priority_ui.h"
#include "motor.h"
#include "power_task.h"
#define AIM_X  956
#define AIM_Y  395
extern motor_measure_t YAW_Motor;
extern measure_t chassis;
extern ext_robot_status_t	robot_state;
extern super_energy super_cap;
extern ext_shoot_data_t									shoot_data;					//0x0207
//void rotate_t( uint16_t *x,  uint16_t *y, float mid_x, float mid_y, float angle);
//void rotate_arc(ui_config_t *config, float mid_x, float mid_y, float angle);
//void Spin_Rp_Ui(void);

ui_info_t dynamic_ui_info [15] = 
{
	//超级电容能量动态显示
    [0] = {
    /*******不变配置*********/
    .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = LINE,         // UI内容类型
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 0,                // 图层数，0~9
    .ui_config.color = CYAN_BLUE,            // 颜色
    .ui_config.width = 30,                // 线条宽度
    .ui_config.start_x = 695,              // 起点 x 坐标
    .ui_config.start_y = 236,              // 起点 y 坐标
    .ui_config.end_x = 1225,                // 终点 x 坐标
    .ui_config.end_y = 236,                // 终点 y 坐标
  },
		//超电数显
		[1] ={
  /*******不变配置*********/
    .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = FLOAT,            // UI内容类型
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 2,                // 图层数，0~9
    .ui_config.color = RED_BLUE,         // 颜色bng
    .ui_config.size = 30,                // 字体大小
    .ui_config.width = 5,                // 线条宽度
    .ui_config.start_x = 1230,              // 起点 x 坐标
    .ui_config.start_y = 258,              // 起点 y 坐标
    .ui_config.int_num = 100,              // 显示的数字
	},
  //SHOOT_MODE_L
  [2] = {
  /*******不变配置*********/
    .ui_config.priority = MID_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = ARC,         // UI内容类型
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = GREEN,            // 颜色
    .ui_config.width = 5,                // 线条宽度
    .ui_config.start_x = 960,              // 圆心 x 坐标
    .ui_config.start_y = 540,              // 圆心 y 坐标
    .ui_config.end_x = 400,                // x 半轴长度
    .ui_config.end_y = 400,                // y 半轴长度
    .ui_config.start_angel = 250,          // 起始角度
    .ui_config.end_angel = 290,            // 终止角度
	},
	//SHOOT_MODE_R
	[3] = {
  /*******不变配置*********/
    .ui_config.priority = MID_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = ARC,         // UI内容类型
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = GREEN,            // 颜色
    .ui_config.width = 5,                // 线条宽度
    .ui_config.start_x = 960,              // 圆心 x 坐标
    .ui_config.start_y = 540,              // 圆心 y 坐标
    .ui_config.end_x = 400,                // x 半轴长度
    .ui_config.end_y = 400,                // y 半轴长度
    .ui_config.start_angel = 70,          // 起始角度
    .ui_config.end_angel = 110,            // 终止角度
  },
	//云台模式选择
    [4] = {
    /*******不变配置*********/
    .ui_config.priority = LOW_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = LINE,         // UI内容类型
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 0,                // 图层数，0~9
    .ui_config.color = CYAN_BLUE,            // 颜色
    .ui_config.width = 30,                // 线条宽度
    .ui_config.start_x = 1820,              // 起点 x 坐标
    .ui_config.start_y = 790,              // 起点 y 坐标
    .ui_config.end_x = 1860,                // 终点 x 坐标
    .ui_config.end_y = 790,                // 终点 y 坐标
  },
		//底盘模式选择
    [5] = {
    /*******不变配置*********/
    .ui_config.priority = LOW_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = LINE,         // UI内容类型
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 0,                // 图层数，0~9
    .ui_config.color = CYAN_BLUE,            // 颜色
    .ui_config.width = 30,                // 线条宽度
    .ui_config.start_x = 1740,              // 起点 x 坐标
    .ui_config.start_y = 690,              // 起点 y 坐标
    .ui_config.end_x = 1780,                // 终点 x 坐标
    .ui_config.end_y = 690,                // 终点 y 坐标
  },
		//拨弹盘模式选择
    [6] = {
    /*******不变配置*********/
    .ui_config.priority = LOW_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = LINE,         // UI内容类型
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 0,                // 图层数，0~9
    .ui_config.color = CYAN_BLUE,            // 颜色
    .ui_config.width = 30,                // 线条宽度
    .ui_config.start_x = 1740,              // 起点 x 坐标
    .ui_config.start_y = 590,              // 起点 y 坐标
    .ui_config.end_x = 1780,                // 终点 x 坐标
    .ui_config.end_y = 590,                // 终点 y 坐标
  },
		//小陀螺状态显示
		[7] = {
  /*******不变配置*********/
    .ui_config.priority = HIGH_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = ARC,         // UI内容类型
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 1,                // 图层数，0~9
    .ui_config.color = YELLOW,            // 颜色
    .ui_config.width = 10,                // 线条宽度
    .ui_config.start_x = 1366,              // 圆心 x 坐标
    .ui_config.start_y = 232,              // 圆心 y 坐标
    .ui_config.end_x = 50,                // x 半轴长度
    .ui_config.end_y = 50,                // y 半轴长度
    .ui_config.start_angel = 210,          // 起始角度
    .ui_config.end_angel = 150,            // 终止角度
  },
		//等级更新
		[8] ={
  /*******不变配置*********/
    .ui_config.priority = MID_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = INT,            // UI内容类型
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 0,                // 图层数，0~9
    .ui_config.color = RED_BLUE,         // 颜色
    .ui_config.size = 40,                // 字体大小
    .ui_config.width = 3,                // 线条宽度
    .ui_config.start_x = 1550,              // 起点 x 坐标
    .ui_config.start_y = 230,              // 起点 y 坐标
    .ui_config.int_num = 1,              // 显示的数字
	},
		[9] = {
    /*******不变配置*********/
			.ui_config.priority = LOW_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = CHAR,           // UI内容类型
    /*******可变配置*********/
    .ui_config.layer = 3,                // 图层数，0~9
    .ui_config.color = RED_BLUE,            // 颜色
    .ui_config.size = 30,                // 字体大小
    .ui_config.width = 3,                // 线条宽度
    .ui_config.start_x = 1743,              // 起点 x 坐标
    .ui_config.start_y = 665,              // 起点 y 坐标
    .ui_config.text = "L",            // 显示的文字
  },
  
  [10] = {
   /*******不变配置*********/
		.ui_config.priority = LOW_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = CHAR,           // UI内容类型
    /*******可变配置*********/
    .ui_config.layer = 3,                // 图层数，0~9
    .ui_config.color = RED_BLUE,            // 颜色
    .ui_config.size = 30,                // 字体大小
    .ui_config.width = 3,                // 线条宽度
    .ui_config.start_x = 1793,              // 起点 x 坐标
    .ui_config.start_y = 665,              // 起点 y 坐标
    .ui_config.text = "H",            // 显示的文字
  },
  
  [11] = {
  /*******不变配置*********/
		.ui_config.priority = LOW_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = CHAR,           // UI内容类型
    /*******可变配置*********/
    .ui_config.layer = 3,                // 图层数，0~9
    .ui_config.color = RED_BLUE,            // 颜色
    .ui_config.size = 30,                // 字体大小
    .ui_config.width = 3,                // 线条宽度
    .ui_config.start_x = 1843,              // 起点 x 坐标
    .ui_config.start_y = 665,              // 起点 y 坐标
    .ui_config.text = "F",            // 显示的文字
  },
	[12] = {
    /*******不变配置*********/
		.ui_config.priority = LOW_PRIORITY, // UI优先级(仅动态UI需要配置)
    .ui_config.ui_type = CHAR,           // UI内容类型
    /*******可变配置*********/
    .ui_config.layer = 3,                // 图层数，0~9
    .ui_config.color = RED_BLUE,            // 颜色
    .ui_config.size = 30,                // 字体大小
    .ui_config.width = 3,                // 线条宽度
    .ui_config.start_x = 1743,              // 起点 x 坐标
    .ui_config.start_y = 565,              // 起点 y 坐标
    .ui_config.text = "L",            // 显示的文字
  },
};
//静态UI绘制//960//540
ui_info_t const_ui_info [4] = 
{
  [0] = {
    /*******不变配置*********/
    .ui_config.ui_type = CIRCLE,           // UI内容类型       
    /*******可变配置*********/
    .ui_config.operate_type = MODIFY,    // 操作类型
    .ui_config.layer = 3,                // 图层数，0~9
    .ui_config.color = RED_BLUE,            // 颜色
    .ui_config.width = 3,                // 线条宽度
    .ui_config.start_x = 926,              // 圆心 x 坐标
    .ui_config.start_y = 526,              // 圆心 y 坐标
    .ui_config.radius = 5,               // 半径
	},   // 显示的文字
};


/**
  @brief: 初始化UI链表 main.c中调用
 */
float super_energyl;
void My_Ui_Init(void)
{
  Init_Ui_List(dynamic_ui_info, sizeof(dynamic_ui_info)/sizeof(ui_info_t),const_ui_info, sizeof(const_ui_info)/sizeof(ui_info_t));
}


extern uint8_t send_test;
void Ui_Info_Update(void)
{
  if (send_test !=0)
  {
    return;
  }
	if(super_cap.Voltage<=1000)
	{
		super_energyl = 1000;
	}
	else if(super_cap.Voltage>=2300)
	{
		super_energyl = 2300;
	}
	else
	{
		super_energyl = super_cap.Voltage;
	}
//	
//	super_energy=super_cap.Voltage<1400?1400:super_cap.Voltage;
//	super_energy=super_cap.Voltage>2300?2300:super_cap.Voltage;
//	int16_t super_energy=695+super_cap.Voltage*0.59-296;
	dynamic_ui_info[0].ui_config.end_x=super_energyl*0.407f+288.9f;
	Enqueue_Ui_For_Sending(&dynamic_ui_info[0]);
	//剩余能量值数显
	dynamic_ui_info[1].ui_config.int_num = super_energyl;
	//云台模式切换
	if(chassis.gim_mode == 3){dynamic_ui_info[4].ui_config.start_x = 1740;dynamic_ui_info[5].ui_config.end_x = 1780;}
	else if(chassis.gim_mode ==2){dynamic_ui_info[4].ui_config.start_x = 1780;dynamic_ui_info[5].ui_config.end_x = 1820;}
	else if(chassis.gim_mode ==1){dynamic_ui_info[4].ui_config.start_x = 1820;dynamic_ui_info[5].ui_config.end_x = 1860;}
	Enqueue_Ui_For_Sending(&dynamic_ui_info[4]);
	//底盘模式切换
	if(chassis.mode == 4){dynamic_ui_info[5].ui_config.start_x = 1740;dynamic_ui_info[5].ui_config.end_x = 1780;}
	else if(chassis.mode ==5){dynamic_ui_info[5].ui_config.start_x = 1780;dynamic_ui_info[5].ui_config.end_x = 1820;}
	else if(chassis.mode ==7){dynamic_ui_info[5].ui_config.start_x = 1820;dynamic_ui_info[5].ui_config.end_x = 1860;}
	Enqueue_Ui_For_Sending(&dynamic_ui_info[5]);
	//拨弹盘模式切换
	if(chassis.feed_frequency == 1){dynamic_ui_info[6].ui_config.start_x = 1740;dynamic_ui_info[6].ui_config.end_x = 1800;}
	else if(chassis.feed_frequency ==2){dynamic_ui_info[6].ui_config.start_x = 1800;dynamic_ui_info[6].ui_config.end_x = 1860;}
	Enqueue_Ui_For_Sending(&dynamic_ui_info[6]);
	//剩余能量值图显
	//int16_t super_energy;
//	if(super_cap.Voltage<=1000)
//	{
//		super_energyl = 0;
//	}
//	else if(super_cap.Voltage>=2300)
//	{
//		super_energyl = 100;
//	}
//	else
//	{
//		super_energyl = super_cap.Voltage*0.077-77;
//	}
//	
////	super_energy=super_cap.Voltage<1400?1400:super_cap.Voltage;
////	super_energy=super_cap.Voltage>2300?2300:super_cap.Voltage;
////	int16_t super_energy=695+super_cap.Voltage*0.59-296;
//	dynamic_ui_info[0].ui_config.end_x=695+super_energyl*5.3;
//	Enqueue_Ui_For_Sending(&dynamic_ui_info[0]);
//	//剩余能量值数显
//	dynamic_ui_info[1].ui_config.int_num = super_energyl;
	//自瞄状态显示//white：未锁定/green:锁定/red:允许自动开火
	if(chassis.aim_ok_flag == 0){dynamic_ui_info[2].ui_config.color=WHITE;dynamic_ui_info[3].ui_config.color=WHITE;}
	else if(chassis.aim_ok_flag == 1){dynamic_ui_info[2].ui_config.color=GREEN;dynamic_ui_info[3].ui_config.color=GREEN;}
	else if(chassis.aim_ok_flag == 2){dynamic_ui_info[2].ui_config.color=RED_BLUE;dynamic_ui_info[3].ui_config.color=RED_BLUE;}
	Enqueue_Ui_For_Sending(&dynamic_ui_info[2]);
	Enqueue_Ui_For_Sending(&dynamic_ui_info[3]);
	//小陀螺状态显示
	dynamic_ui_info[7].ui_config.end_angel = ((int)(222+YAW_Motor.ecd_angle))%360;
	dynamic_ui_info[7].ui_config.start_angel = ((int)(162+YAW_Motor.ecd_angle))%360;
	Enqueue_Ui_For_Sending(&dynamic_ui_info[7]);
	dynamic_ui_info[8].ui_config.int_num = robot_state.robot_level;
	Enqueue_Ui_For_Sending(&dynamic_ui_info[8]);
}



