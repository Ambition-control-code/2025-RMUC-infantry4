#include "usbd_cdc_if.h"
#include "ulitility_tools.h"
#include "stdlib.h"
#include "string.h"
#include "math.h"
#include "usart.h"
#include "bsp_dwt.h"
#define long_of_leg 2*0.09f
#define zero_angle  18.03/57.3f
#define half_of_wide  0.055
#define F_Y 1
#define M_of_car 2.1f
#define g_ln 9.8f

void vmc(float set_hight,float *TL,float *TR,float ROLL,float kp,float kd,float angle_left,float angle_right,float speed_l,float speed_r)
{
float now_hight,set_angle_l,set_angle_r,replay_roll,F_virtual_l,F_virtual_r;
float gravity_compensation,inertia_compensation;
gravity_compensation=0.5f*(M_of_car)*g_ln;
replay_roll=tanf(ROLL/57.3f)*half_of_wide;
set_angle_l=asinf((set_hight+replay_roll)/long_of_leg)-zero_angle;
set_angle_r=asinf((set_hight-replay_roll)/long_of_leg)-zero_angle;
F_virtual_l=kp*(set_angle_l-angle_left) +kd*(0-speed_l)+gravity_compensation;
F_virtual_r=kp*(set_angle_r-angle_right)+kd*(0-speed_r)+gravity_compensation;
*TL=F_virtual_l*F_Y;
*TR=F_virtual_r*F_Y;	
}


void station_check(float pitch)
{


}

/**
  * @brief Initializes the Second order lowpass filter according to the specified parameters in the
  *         LowPassFilter2p_Info_TypeDef.
  * @param lpf: pointer to an LowPassFilter2p_Info_TypeDef structure that
  *         contains the information  for the Second order lowpass filter.
  * @param alpha: the floating-point array of filter coefficient
  * @param frame_period: frame perood
  * @retval none
  */
void LowPassFilter2p_Init(LowPassFilter2p_Info_TypeDef *lpf,float alpha[3])
{
	lpf->alpha[0] = alpha[0];
	lpf->alpha[1] = alpha[1];
	lpf->alpha[2] = alpha[2];

  lpf->input = 0;
	
	lpf->output[0] = 0;
	lpf->output[1] = 0;
	lpf->output[2] = 0;
}
//------------------------------------------------------------------------------


/**
  * @brief Update the Second order lowpass filter according to the specified parameters in the
  *         LowPassFilter2p_Info_TypeDef.
  * @param kf: pointer to an LowPassFilter2p_Info_TypeDef structure that
  *         contains the information  for the Second order lowpass filter.
  * @param input: the filter input
  * @retval lowpass filter output
  */
float LowPassFilter2p_Update(LowPassFilter2p_Info_TypeDef *lpf,float input)
{
	lpf->input = input;
  
  if(lpf->Initialized == 0)
  {
    lpf->output[0] = lpf->input;
    lpf->output[1] = lpf->input;
    lpf->output[2] = lpf->input;
    lpf->Initialized = 1;
  }
  
	lpf->output[0] = lpf->output[1];
	lpf->output[1] = lpf->output[2];
  lpf->output[2] = lpf->alpha[1] * lpf->output[0] + lpf->alpha[0] * lpf->output[1] + lpf->alpha[2] * lpf->input;

	return lpf->output[2];
}
//------------------------------------------------------------------------------

float sign_tool(float x)
{
	if(x>0)
		return 1;
	else if(x<0)
		return -1;
	else
		return 0;
}

float now_change,largest;
float maopao[6];

float fliter_for_jump(float data)
{
static float sample_space_last,last,rps,sample_space;
static int i;
rps=-(last-DWT_GetTimeline_ms());
last=DWT_GetTimeline_ms();
now_change=(data-sample_space_last)/(rps/1000);
sample_space_last=data;
if(ABS( now_change)>6) {return data;}
else return sample_space_last;
//for(int i=0;i<6;i++)
//{
//	(sample_space[i]>largest)?(largest=sample_space[i]):(largest=largest);
//}
}

float *Norm3d(float *v)
{
    float len = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    v[0] /= len;
    v[1] /= len;
    v[2] /= len;
    return v;
}

float NormOf3d(float *v)
{
    return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

void Cross3d(float *v1, float *v2, float *res)
{
    res[0] = v1[1] * v2[2] - v1[2] * v2[1];
    res[1] = v1[2] * v2[0] - v1[0] * v2[2];
    res[2] = v1[0] * v2[1] - v1[1] * v2[0];
}

float Dot3d(float *v1, float *v2)
{
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}



