//#include "chassis_power_control.h"
//#include "referee.h"
//#include "arm_math.h"
////#include "detect_task.h"
//#include "remote_control.h"
//#include "pid.h"
//#include "voltage_task.h"

//extern cap_measure_t cap_measure; // capacitor data structure
//extern RC_ctrl_t rc_ctrl;
//uint8_t cap_state = 0;
//void chassis_power_control(chassis_move_t *chassis_power_control)
//{

//	uint16_t max_power_limit = 40;
//	fp32 chassis_max_power = 0;
//	float input_power = 0;		 // input power from battery (referee system)
//	float initial_give_power[4]; // initial power from PID calculation
//	float initial_total_power = 0;
//	fp32 scaled_give_power[4];

//	fp32 chassis_power = 0.0f;
//	fp32 chassis_power_buffer = 0.0f;

//	fp32 toque_coefficient = 1.99688994e-6f; // (20/16384)*(0.3)*(187/3591)/9.55
//	fp32 a = 1.23e-07;						 // k1
//	fp32 k2 = 1.453e-07;					 // k2
//	fp32 constant = 4.081f;

//	get_chassis_power_and_buffer(&chassis_power, &chassis_power_buffer);
//	PID_calc(&chassis_power_control->buffer_pid, chassis_power_buffer, 30);
//	get_chassis_max_power(&max_power_limit);
//	input_power = max_power_limit - chassis_power_control->buffer_pid.out; // Input power floating at maximum power

//	CAN_CMD_CAP(input_power); // set the input power of capacitor controller

//	if (rc_ctrl.key.v & KEY_PRESSED_OFFSET_E)
//	{
//		cap_state = 0;
//	}
//	if (rc_ctrl.key.v & KEY_PRESSED_OFFSET_Q)
//	{
//		cap_state = 1;
//	}

//	if (cap_measure.cap_percent > 5)
//	{
//		if (cap_state == 0)
//		{
//			chassis_max_power = input_power + 5; // Slightly greater than the maximum power, avoiding the capacitor being full all the time and improving energy utilization
//		}
//		else
//		{
//			chassis_max_power = input_power + 200;
//		}
//	}
//	else
//	{
//		chassis_max_power = input_power;
//	}

//	for (uint8_t i = 0; i < 4; i++) // first get all the initial motor power and total motor power
//	{
//		initial_give_power[i] = chassis_power_control->motor_speed_pid[i].out * toque_coefficient * chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm +
//								k2 * chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm * chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm +
//								a * chassis_power_control->motor_speed_pid[i].out * chassis_power_control->motor_speed_pid[i].out + constant;

//		if (initial_give_power < 0) // negative power not included (transitory)
//			continue;
//		initial_total_power += initial_give_power[i];
//	}

//	if (initial_total_power > chassis_max_power) // determine if larger than max power
//	{
//		fp32 power_scale = chassis_max_power / initial_total_power;
//		for (uint8_t i = 0; i < 4; i++)
//		{
//			scaled_give_power[i] = initial_give_power[i] * power_scale; // get scaled power
//			if (scaled_give_power[i] < 0)
//			{
//				continue;
//			}

//			fp32 b = toque_coefficient * chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm;
//			fp32 c = k2 * chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm * chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm - scaled_give_power[i] + constant;

//			if (chassis_power_control->motor_speed_pid[i].out > 0) // Selection of the calculation formula according to the direction of the original moment
//			{
//				fp32 temp = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
//				if (temp > 16000)
//				{
//					chassis_power_control->motor_speed_pid[i].out = 16000;
//				}
//				else
//					chassis_power_control->motor_speed_pid[i].out = temp;
//			}
//			else
//			{
//				fp32 temp = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
//				if (temp < -16000)
//				{
//					chassis_power_control->motor_speed_pid[i].out = -16000;
//				}
//				else
//					chassis_power_control->motor_speed_pid[i].out = temp;
//			}
//		}
//	}
//}
//#include "chassis_power_control.h"
//#include "referee.h"
//#include "arm_math.h"
////#include "detect_task.h"
//#include "remote_control.h"
//#include "bsp_pid.h"
//#include "referee.h"
////#include "voltage_task.h"

///* 外部变量声明 */
//cap_measure_t cap_measure;   // 超级电容测量数据结构
//extern RC_ctrl_t rc_ctrl;           // 遥控器控制数据结构
//extern ext_robot_status_t robot_state;
//extern ext_power_heat_data_t power_heat_data;
//uint8_t cap_state = 0;              // 电容能量释放状态：0-正常模式，1-主动释放模式

///**
// * @brief 底盘功率控制主函数
// * @param chassis_power_control 底盘运动控制结构体指针
// * @note 实现功能：
// * 1. 动态功率限制：根据超级电容状态调整底盘最大功率
// * 2. 功率再分配：通过二次方程求解各电机力矩限制值
// * 3. 兼容裁判系统功率限制，防止超功率
// */
//void power_ctrl_init()
//{
//	 /* 初始化参数 */
//    uint16_t max_power_limit = robot_state.chassis_power_limit;                    // 裁判系统功率上限（W）
//    fp32 chassis_max_power = 0;                      // 动态计算的底盘最大功率
//    float input_power = 0;                           // 电池输入功率（来自裁判系统）
//    float initial_give_power[4];                     // 各电机初始需求功率
//    float initial_total_power = 0;                   // 总初始功率
//    fp32 scaled_give_power[4];                       // 缩放后的各电机功率

//    /* 功率与缓冲能量 */
//    fp32 chassis_power = 0.0f;                       // 当前底盘功率
//    fp32 chassis_power_buffer = power_heat_data.chassis_power_buffer;                // 缓冲能量（用于平滑功率限制）

//    /* 电机功率模型参数（需实验拟合）*/
//    fp32 toque_coefficient = 1.99688994e-6f;         // 力矩电流系数：(20/16384) * K_T / 9.55
//    fp32 a = 1.23e-07;                               // 转速二次项系数 k1
//    fp32 k2 = 1.453e-07;                             // 力矩二次项系数 k2
//    fp32 constant = 4.081f;                          // 常数项 a（来自模型公式）





//}
//void chassis_power_control(chassis_move_t *chassis_power_control)
//{
////    /* 初始化参数 */
////    uint16_t max_power_limit = robot_state.chassis_power_limit;                    // 裁判系统功率上限（W）
////    fp32 chassis_max_power = 0;                      // 动态计算的底盘最大功率
////    float input_power = 0;                           // 电池输入功率（来自裁判系统）
////    float initial_give_power[4];                     // 各电机初始需求功率
////    float initial_total_power = 0;                   // 总初始功率
////    fp32 scaled_give_power[4];                       // 缩放后的各电机功率

////    /* 功率与缓冲能量 */
////    fp32 chassis_power = 0.0f;                       // 当前底盘功率
////    fp32 chassis_power_buffer = power_heat_data.chassis_power_buffer;                // 缓冲能量（用于平滑功率限制）

////    /* 电机功率模型参数（需实验拟合）*/
////    fp32 toque_coefficient = 1.99688994e-6f;         // 力矩电流系数：(20/16384) * K_T / 9.55
////    fp32 a = 1.23e-07;                               // 转速二次项系数 k1
////    fp32 k2 = 1.453e-07;                             // 力矩二次项系数 k2
////    fp32 constant = 4.081f;                          // 常数项 a（来自模型公式）

//    /* Step 1: 获取当前功率状态 */
//    //get_chassis_power_and_buffer(&chassis_power, &chassis_power_buffer); // 从裁判系统获取
//    PID_calc(&chassis_power_control->buffer_pid, chassis_power_buffer, 30); // 缓冲能量PID闭环
//    //get_chassis_max_power(&max_power_limit);         // 获取裁判系统动态功率限制
//   // input_power = max_power_limit - chassis_power_control->buffer_pid.out; // 计算实际可用功率

//    /* Step 2: 超级电容控制 */
//    //CAN_CMD_CAP(input_power); // 设置电容控制器输入功率
//    
//    // 遥控器按键切换电容状态（Q/E键）
//    if (rc_ctrl.key.v & KEY_PRESSED_OFFSET_E) cap_state = 0; // E键：正常模式
//    if (rc_ctrl.key.v & KEY_PRESSED_OFFSET_Q) cap_state = 1; // Q键：主动释放模式

//    /* Step 3: 动态调整底盘最大功率 */
//    if (cap_measure.cap_percent > 5) { // 电容电量>5%时允许额外功率
//        if (cap_state == 0) {
//            chassis_max_power = input_power + 5;     // 正常模式：略微超限，避免电容满充
//        } else {
//            chassis_max_power = input_power + 200;   // 主动释放：允许短时大功率（如冲刺）
//        }
//    } else {
//        chassis_max_power = input_power;            // 电容电量不足时严格限制
//    }

//    /* Step 4: 计算各电机初始需求功率 */
//    for (uint8_t i = 0; i < 4; i++) {
//        // 公式：P_in = C_T*I_cmd*ω + k1*ω2 + k2*I_cmd2 + a （对应报告公式9）
//        initial_give_power[i] = 
//            chassis_power_control->motor_speed_pid[i].out * toque_coefficient * 
//            chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm +
//            k2 * chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm * 
//            chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm +
//            a * chassis_power_control->motor_speed_pid[i].out * 
//            chassis_power_control->motor_speed_pid[i].out + constant;

//        if (initial_give_power[i] < 0) continue;     // 负功率不计入总功率（瞬态）
//        initial_total_power += initial_give_power[i];
//    }

//    /* Step 5: 功率再分配算法 */
//    if (initial_total_power > chassis_max_power) {   // 总功率超限时缩放
//        fp32 power_scale = chassis_max_power / initial_total_power; // 缩放系数
//        
//        for (uint8_t i = 0; i < 4; i++) {
//            scaled_give_power[i] = initial_give_power[i] * power_scale; // 缩放功率
//            if (scaled_give_power[i] < 0) continue;

//            /* 解二次方程：a*τ2 + b*τ + c = 0 */
//            fp32 b = toque_coefficient * 
//                chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm;
//            fp32 c = k2 * 
//                chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm * 
//                chassis_power_control->motor_chassis[i].chassis_motor_measure->speed_rpm - 
//                scaled_give_power[i] + constant;

//            // 根据原始力矩方向选择解（报告公式7逻辑）
//            if (chassis_power_control->motor_speed_pid[i].out > 0) { 
//                // 正向力矩选正根
//                fp32 temp = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
//                chassis_power_control->motor_speed_pid[i].out = 
//                    (temp > 16000) ? 16000 : temp;   // 限幅防止溢出
//            } else { 
//                // 负向力矩选负根
//                fp32 temp = (-b - sqrt(b * b - 4 * a * c)) / (2 * a);
//                chassis_power_control->motor_speed_pid[i].out = 
//                    (temp < -16000) ? -16000 : temp; 
//            }
//        }
//    }
//}
