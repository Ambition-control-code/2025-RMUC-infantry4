#ifndef BSP_USART_H
#define BSP_USART_H
#include "main.h"


extern void usart5_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
void uart5_init_task();
void refree_data_task(uint8_t *rx1_buf);
uint8_t uart_send_data(uint8_t *txbuf, uint16_t length);
#endif
