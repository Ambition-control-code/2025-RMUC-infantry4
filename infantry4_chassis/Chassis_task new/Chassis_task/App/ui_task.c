#include "ui_task.h"
#include "ui.h"
#include "usart.h"
#include "motor.h"
#include "priority_ui.h"
#include "ui_protocol.h"
extern UART_HandleTypeDef huart5;
extern uint8_t send_test;
extern measure_t chassis;
/**
  * @Name    StartControlTask
  * @brief   控制任务
  * @param   
  * @retval 
  * @author  
  * @Date    
**/
extern measure_t chassis;
int send=0;int upda=0;
extern ext_client_custom_graphic_seven_t graphic_tx_buffer;   // 图像发送缓存
void ui_send_task(void const * argument)
{
  /* USER CODE BEGIN ui_send_task */
  /* Infinite loop */
  for(;;)
  {
		upda+=1;
		//send_test=chassis.rc_mode;
		client_info_update();
		if(chassis.mode==8)send_test=1;
		else if(chassis.rc_mode!=8)send_test=0;
		 Ui_Info_Update();//UI界面信息更新
		//Ui_Send_Normal();
    osDelay(5);
  }
}
/**
  * @Name    StartRealTimeTask
  * @brief   实时任务更新
  * @param   
  * @retval 
  * @author  
  * @Date    
**/
void ui_update_task(void const * argument)
{
  /* USER CODE BEGIN ui_update_task */
  /* Infinite loop */
  for(;;)
  {
		Ui_Send();//发送信息
		send+=1;
		//__HAL_UART_CLEAR_FLAG(&huart5,UART_FLAG_TC);
    osDelay(50);
  }
  /* USER CODE END ui_update_task */
}
/**
  * @Name    StartHeartBeatTask
  * @brief   监控任务
  * @param   
  * @retval 
  * @author  
  * @Date    
**/
//void StartHeartBeatTask(void const * argument)
//{
//  for(;;)
//  {
//    osDelay(1);
//  }
//}



