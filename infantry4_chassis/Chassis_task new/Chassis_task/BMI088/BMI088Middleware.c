#include "main.h"
void BMI088_com_init(void)
{


}
void BMI088_GPIO_init(void)
{

}
void BMI088_delay_us(uint16_t us)
{

    uint32_t ticks = 0;
    uint32_t told = 0;
    uint32_t tnow = 0;
    uint32_t tcnt = 0;
    uint32_t reload = 0;
    reload = SysTick->LOAD;
    ticks = us * 168;
    told = SysTick->VAL;
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks)
            {
                break;
            }
        }
    }


}
void BMI088_delay_ms(uint16_t ms)
{
    while(ms--)
    {
        BMI088_delay_us(1000);
    }
}


void BMI088_ACCEL_NS_L(void)//取消gyro  spi 片选

{
	HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
}
void BMI088_ACCEL_NS_H(void)//片选中acccel，读取accel数据
{ 
  HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
}

void BMI088_GYRO_NS_L(void)//取消gyro spi 片选
{
  HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
}
void BMI088_GYRO_NS_H(void)//片选中gyro,读取gyro数据
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
}

int spi_11 = 0;
int spi_get = 0;
int i = 0;
//uint8_t BMI088_read_write_byte(uint8_t txdata)
//{
//	
//    uint8_t rx_data;
//	   i++;
//	 while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET){}  //等待发送区为空
//	  SPI_I2S_SendData(SPI1,txdata);
//		 spi_11 = 1; 
//		 
//	 while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET){}
//	  rx_data = SPI_I2S_ReceiveData(SPI1);
//		 if(i==3)
//	  spi_get = rx_data;
//    //HAL_SPI_TransmitReceive(&hspi1, &txdata, &rx_data, 1, 1000);
//    return rx_data;
//}
extern SPI_HandleTypeDef hspi2;
uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&hspi2, &txdata, &rx_data, 1, 1000);
    return rx_data;
}





