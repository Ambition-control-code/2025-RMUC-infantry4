#include "bsp_uart.h"
#include "referee.h"
#include "usart.h"
#include "main.h"
#include "ui_protocol.h"
extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern uint8_t client_tx_buf[128];
#define USART_RX_BUF_LENGHT 200
#define FrameSize 128
uint8_t usart5_buf[2][USART_RX_BUF_LENGHT];
uint8_t Frame  [FrameSize];
uint8_t  Speed1,Speed2,Speed3,Speed4;
void usart5_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{

    //enable the DMA transfer for the receiver and tramsmit request
    //使能DMA串口接收和发送
    SET_BIT(huart5.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart5.Instance->CR3, USART_CR3_DMAT);

    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);



    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_uart5_rx);
    
    while(hdma_uart5_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_uart5_rx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_uart5_rx, DMA_LISR_TCIF1);

    hdma_uart5_rx.Instance->PAR = (uint32_t) & (UART5->DR);
    //memory buffer 1
    //内存缓冲区1
    hdma_uart5_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //内存缓冲区2
    hdma_uart5_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    //数据长度
    __HAL_DMA_SET_COUNTER(&hdma_uart5_rx, dma_buf_num);

    //enable double memory buffer
    //使能双缓冲区
    SET_BIT(hdma_uart5_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_uart5_rx);
		// 禁用发送DMA并确保停止
}


  void uart5_init_task()
	{
   usart5_init(usart5_buf[0], usart5_buf[1], USART_RX_BUF_LENGHT);
	}


void UART5_IRQHandler(void)
{
    static volatile uint8_t res;
	//__HAL_UART_CLEAR_FLAG(&huart5,UART_FLAG_TC);
    if(UART5->SR & UART_FLAG_IDLE)
    {
        __HAL_UART_CLEAR_PEFLAG(&huart5);

        static uint16_t this_time_rx_len = 0;

        if ((huart5.hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
        {
            __HAL_DMA_DISABLE(huart5.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart5.hdmarx);
            __HAL_DMA_SET_COUNTER(huart5.hdmarx, USART_RX_BUF_LENGHT);
            huart5.hdmarx->Instance->CR |= DMA_SxCR_CT;
            __HAL_DMA_ENABLE(huart5.hdmarx);
					 //HAL_UART_Receive_DMA(&huart5, (uint8_t*)usart5_buf[0], this_time_rx_len);         
            judge_read_data(usart5_buf[0]);
        }
        else
        {
            __HAL_DMA_DISABLE(huart5.hdmarx);
            this_time_rx_len = USART_RX_BUF_LENGHT - __HAL_DMA_GET_COUNTER(huart5.hdmarx);
            __HAL_DMA_SET_COUNTER(huart5.hdmarx, USART_RX_BUF_LENGHT);
            huart5.hdmarx->Instance->CR &= ~(DMA_SxCR_CT);
            __HAL_DMA_ENABLE(huart5.hdmarx);
					 //HAL_UART_Receive_DMA(&huart5, (uint8_t*)usart5_buf[0], this_time_rx_len);         
            judge_read_data(usart5_buf[1]);
        }
    }
		else
		{
        __HAL_UART_CLEAR_FLAG(&huart5,UART_FLAG_TC);		
		}
}



