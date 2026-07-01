#ifndef ulitility_tools_H
#define ulitility_tools_H
#include "main.h"

#define LIMIT_MAX_MIN(x, max, min)	(((x) <= (min)) ? (min):(((x) >= (max)) ? (max) : (x)))

/**
 * @brief typedef structure that contains the information for the second order lowpass filter.
 */
typedef struct 
{
    uint8_t Initialized;  /*!< init flag */
    float input;       /*!< input value */
    float output[3];   /*!< output value */
    float alpha[3];    /*!< filter coefficient */
}LowPassFilter2p_Info_TypeDef;


#define UNITEE_TX() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET)
#define UNITEE_RX() HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_RESET)

//void  judege_station(int16_t a,int16_t b,Gimbal_hero *HERO_AMB);
//float gimbal_pitch_replay(float now_pitch,float *a);
//void  OLS_Init(Ordinary_Least_Squares_t *OLS, uint16_t order);
//float OLS_Smooth(Ordinary_Least_Squares_t *OLS, float deltax, float y);
//void vmc(float set_hight,float *TL,float *TR,float ROLL,float kp,float kd,float angle_left,float angle_right,float speed_l,float speed_r);
//void unitee_8016_send(u8 id,float angle,int32_t tor,float kp,float kd);     
void LowPassFilter2p_Init(LowPassFilter2p_Info_TypeDef *lpf,float alpha[3]);
float LowPassFilter2p_Update(LowPassFilter2p_Info_TypeDef *lpf,float input);
float sign_tool(float x);
float fliter_for_jump(float data);
float *Norm3d(float *v);
float NormOf3d(float *v);
void Cross3d(float *v1, float *v2, float *res);
float Dot3d(float *v1, float *v2);

#endif








