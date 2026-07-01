#include "remote_control.h"
#include "main.h"
#include "tele_link.h"
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern VT13_ctrl_t VT13_ctrl;
RC_ctrl_t rc_ctrl;//遥控器数据结构体
int rc_ok_flag=0;
//receive data, 18 bytes one frame, but set 36 bytes 
//接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
 uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];
/**
  * @brief          遥控器初始化
  * @param[in]      none
  * @retval         none
  */
void remote_control_init(void)
{
    RC_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}
/**
  * @brief          获取遥控器数据指针
  * @param[in]      none
  * @retval         遥控器数据指针
  */
 RC_ctrl_t *get_remote_control_point(void)
{
    return &rc_ctrl;
}


//串口中断
void USART3_IRQHandler(void)
{
    if(huart3.Instance->SR & UART_FLAG_RXNE)//接收到数据
    {
        __HAL_UART_CLEAR_PEFLAG(&huart3);//清除usart奇偶校验错误标志
    }
    else if(USART3->SR & UART_FLAG_IDLE)//如果usart处于空闲状态
    {
        static uint16_t this_time_rx_len = 0;//定义静态变量存储本次接收长度

        __HAL_UART_CLEAR_PEFLAG(&huart3);

        if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)//检查dma控制标志是否被设置
        {
            /* Current memory buffer used is Memory 0 */
    
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 1
            //设定缓冲区1
            hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                sbus_to_rc(sbus_rx_buf[0], &rc_ctrl);
            }
        }
        else
        {
            /* Current memory buffer used is Memory 1 */
            //disable DMA
            //失效DMA
            __HAL_DMA_DISABLE(&hdma_usart3_rx);

            //get receive data length, length = set_data_length - remain_length
            //获取接收数据长度,长度 = 设定长度 - 剩余长度
            this_time_rx_len = SBUS_RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;

            //reset set_data_lenght
            //重新设定数据长度
            hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_NUM;

            //set memory buffer 0
            //设定缓冲区0
            DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
            
            //enable DMA
            //使能DMA
            __HAL_DMA_ENABLE(&hdma_usart3_rx);

            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //处理遥控器数据
                sbus_to_rc(sbus_rx_buf[1], &rc_ctrl);
            }
        }
    }
}
/**
  * @brief          遥控器协议解析
	* @author         Z&S
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl_ctrl: 遥控器数据指
  * @retval         none
  */
 void sbus_to_rc(uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
	
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {   
			return;
    }

			rc_ctrl->cnts+=1;
			rc_ctrl->R_x = 1024 - ((sbus_buf[0]| (sbus_buf[1] << 8)) & 0x07ff); //!< Channel 0
			rc_ctrl->R_y = 1024 - (((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff); //!< Channel 1
			rc_ctrl->L_x = 1024 - (((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) | (sbus_buf[4] << 10))& 0x07ff);//!< Channel 2
			rc_ctrl->L_y = 1024 - (((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff); //!< Channel 3
			rc_ctrl->last_sr = rc_ctrl->sr;
			rc_ctrl->sl = ((sbus_buf[5] >> 4)& 0x000C) >> 2; //!< Switch left
			rc_ctrl->sr = ((sbus_buf[5] >> 4)& 0x0003); //!< Switch left
			//rc_ctrl->last_sr = rc_ctrl->sr;
			rc_ctrl->mouse_x = (sbus_buf[6] | (sbus_buf[7] << 8)); //!< Mouse X axis
			rc_ctrl->mouse_y = (sbus_buf[8] | (sbus_buf[9] << 8)); //!< Mouse Y axis
			rc_ctrl->mouse_z = sbus_buf[10] | (sbus_buf[11] << 8); //!< Mouse Z axis
			rc_ctrl->mouse_l = (sbus_buf[12]); //!< Mouse Left Is Press ?
			rc_ctrl->mouse_r = (sbus_buf[13]); //!< Mouse Right Is Press ?
			rc_ctrl->key = sbus_buf[14] | (sbus_buf[15] << 8); //!< KeyBoard value
			rc_ctrl->lun = -((sbus_buf[16] | (sbus_buf[17] << 8))-1024); //拨轮
			//rc_ctrl->last_lun = rc_ctrl->lun;
			rc_ctrl->W = ((rc_ctrl->key&0x01)>>0);
			rc_ctrl->S = ((rc_ctrl->key&0x02)>>1);
			rc_ctrl->A = ((rc_ctrl->key&0x04)>>2);
			rc_ctrl->D = ((rc_ctrl->key&0x08)>>3);
			rc_ctrl->last_Shift = rc_ctrl->Shift;
			rc_ctrl->Shift = ((rc_ctrl->key&0x10)>>4);
			rc_ctrl->last_Ctrl = rc_ctrl->Ctrl;
			rc_ctrl->Ctrl = (rc_ctrl->key&0x20)>>5;
			rc_ctrl->last_Q = rc_ctrl->Q;
			rc_ctrl->Q = ((rc_ctrl->key&0x40)>>6);
			rc_ctrl->last_E = rc_ctrl->E;
			rc_ctrl->E  = ((rc_ctrl->key&0x80)>>7);
			rc_ctrl->last_R = rc_ctrl->R;
			rc_ctrl->R = ((rc_ctrl->key&0x100)>>8);
			rc_ctrl->last_F = rc_ctrl->F;
			rc_ctrl->F = ((rc_ctrl->key&0x200)>>9);
			rc_ctrl->last_G = rc_ctrl->G;
			rc_ctrl->G = ((rc_ctrl->key&0x400)>>10);
			rc_ctrl->Z = ((rc_ctrl->key&0x800)>>11);
			rc_ctrl->X = ((rc_ctrl->key&0x1000)>>12);
			rc_ctrl->C = ((rc_ctrl->key&0x2000)>>13);
			rc_ctrl->V = ((rc_ctrl->key&0x4000)>>14);
			rc_ctrl->last_B=rc_ctrl->B; 
			rc_ctrl->B  = ((rc_ctrl->key&0x8000)>>15);
		//遥控器值抛出异常
			rc_ctrl->R_x = rc_ctrl->R_x > 660 ? 0 : rc_ctrl->R_x;
			rc_ctrl->R_x = rc_ctrl->R_x < -660 ? 0 : rc_ctrl->R_x;
			
			rc_ctrl->R_y = rc_ctrl->R_y > 660 ? 0 : rc_ctrl->R_y;
			rc_ctrl->R_y = rc_ctrl->R_y < -660 ? 0 : rc_ctrl->R_y;
			
			rc_ctrl->L_x = rc_ctrl->L_x > 660 ? 0 : rc_ctrl->L_x;
			rc_ctrl->L_x = rc_ctrl->L_x < -660 ? 0 : rc_ctrl->L_x;
			
			rc_ctrl->L_y = rc_ctrl->L_y > 660 ? 0 : rc_ctrl->L_y;
			rc_ctrl->L_y = rc_ctrl->L_y < -660 ? 0 : rc_ctrl->L_y;
			
			rc_ctrl->mouse_x = rc_ctrl->mouse_x > 1024 ? 1024 : rc_ctrl->mouse_x;
			rc_ctrl->mouse_x = rc_ctrl->mouse_x < -1024 ? -1024 : rc_ctrl->mouse_x;
			rc_ctrl->mouse_y = rc_ctrl->mouse_y > 1024 ? 1024 : rc_ctrl->mouse_y;
			rc_ctrl->mouse_y = rc_ctrl->mouse_y < -1024 ? -1024 : rc_ctrl->mouse_y;
			
			
	}

	
	
	
