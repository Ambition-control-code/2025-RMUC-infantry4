#include "ADRC_core.h"
#include "math.h"

float ABS(float X)  
{if(X<0) X=-X;}  

Fhan_Data ADRC_Pitch_Controller;
Fhan_Data ADRC_Roll_Controller;
float ADRC_Unit[3][15]=
{
/*TD跟踪微分器   改进最速TD,h0=N*h      扩张状态观测器ESO           扰动补偿     非线性组合*/
/*  r     h      N                  beta_01   beta_02    beta_03     b0       beta_0  beta_1  beta_2   N1     C    alpha1  alpha2*/
 {300000 ,0.005 , 2,               100,      1000,      2000,     0.001,    0.002,   1.0,      0.0005,    5,    5,    0.8,   1.5,    50},
 {300000 ,0.005 , 2,               100,      1000,      2000,     0.001,    0.002,   1.0,      0.0005,    5,    5,    0.8,   1.5,    50},
 {50000  ,0.005 , 30,              100,      2000,      10000,    5    ,    0.002,   10,        0.001,    5,    5,    0.5,   1.05,   50},
};


float Constrain_Float(float amt, float low, float high){
  return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}

int16_t Sign_ADRC(float Input)
{
    int16_t output=0;
    if(Input>1E-6) output=1;
    else if(Input<-1E-6) output=-1;
    else output=0;
    return output;
}

int16_t Fsg_ADRC(float x,float d)
{
  int16_t output=0;
  output=(Sign_ADRC(x+d)-Sign_ADRC(x-d))/2;
  return output;
}


void ADRC_Init(Fhan_Data *fhan_Input1,float ADRC_Unit[15])
{
  fhan_Input1->r=ADRC_Unit[0];
  fhan_Input1->h=ADRC_Unit[1];
  fhan_Input1->N0=(uint16)(ADRC_Unit[2]);
  fhan_Input1->beta_01=ADRC_Unit[3];
  fhan_Input1->beta_02=ADRC_Unit[4];
  fhan_Input1->beta_03=ADRC_Unit[5];
  fhan_Input1->b0=ADRC_Unit[6];
  fhan_Input1->beta_0=ADRC_Unit[7];
  fhan_Input1->beta_1=ADRC_Unit[8];
  fhan_Input1->beta_2=ADRC_Unit[9];
  fhan_Input1->N1=(uint16)(ADRC_Unit[10]);
  fhan_Input1->c=ADRC_Unit[11];

  fhan_Input1->alpha1=ADRC_Unit[12];
  fhan_Input1->alpha2=ADRC_Unit[13];
  fhan_Input1->zeta=ADRC_Unit[14];
}

//ADRC最速跟踪微分器TD，改进的算法fhan
float TD_PID(Fhan_Data *fhan_Input,float expect)//安排ADRC过度过程
{
  float d=0,a0=0,y=0,a1=0,a2=0,a=0;
  float x1_delta=0;//ADRC状态跟踪误差项
  x1_delta=fhan_Input->x1-expect;//用x1-v(k)替代x1得到离散更新公式
  fhan_Input->h0=fhan_Input->N0*fhan_Input->h;//用h0替代h，解决最速跟踪微分器速度超调问题
  d=fhan_Input->r*fhan_Input->h0*fhan_Input->h0;//d=rh^2;
  a0=fhan_Input->h0*fhan_Input->x2;//a0=h*x2
  y=x1_delta+a0;//y=x1+a0
  a1=sqrt(d*(d+8*ABS(y)));//a1=sqrt(d*(d+8*ABS(y))])
  a2=a0+Sign_ADRC(y)*(a1-d)/2;//a2=a0+sign(y)*(a1-d)/2;
  a=(a0+y)*Fsg_ADRC(y,d)+a2*(1-Fsg_ADRC(y,d));
  fhan_Input->fh=-fhan_Input->r*(a/d)*Fsg_ADRC(a,d)
                  -fhan_Input->r*Sign_ADRC(a)*(1-Fsg_ADRC(a,d));//得到最速微分加速度跟踪量
  fhan_Input->x1+=fhan_Input->h*fhan_Input->x2;//跟新最速跟踪状态量x1
  fhan_Input->x2+=fhan_Input->h*fhan_Input->fh;//跟新最速跟踪状态量微分x2
}

//ADRC最速跟踪微分器TD，改进的算法fhan
void Fhan_ADRC(Fhan_Data *fhan_Input,float expect_ADRC)//安排ADRC过度过程
{
  float d=0,a0=0,y=0,a1=0,a2=0,a=0;
  float x1_delta=0;//ADRC状态跟踪误差项
  x1_delta=fhan_Input->x1-expect_ADRC;//用x1-v(k)替代x1得到离散更新公式
  fhan_Input->h0=fhan_Input->N0*fhan_Input->h;//用h0替代h，解决最速跟踪微分器速度超调问题
  d=fhan_Input->r*fhan_Input->h0*fhan_Input->h0;//d=rh^2;
  a0=fhan_Input->h0*fhan_Input->x2;//a0=h*x2
  y=x1_delta+a0;//y=x1+a0
  a1=sqrt(d*(d+8*ABS(y)));//a1=sqrt(d*(d+8*ABS(y))])
  a2=a0+Sign_ADRC(y)*(a1-d)/2;//a2=a0+sign(y)*(a1-d)/2;
  a=(a0+y)*Fsg_ADRC(y,d)+a2*(1-Fsg_ADRC(y,d));
  fhan_Input->fh=-fhan_Input->r*(a/d)*Fsg_ADRC(a,d)
                  -fhan_Input->r*Sign_ADRC(a)*(1-Fsg_ADRC(a,d));//得到最速微分加速度跟踪量
  fhan_Input->x1+=fhan_Input->h*fhan_Input->x2;//跟新最速跟踪状态量x1
  fhan_Input->x2+=fhan_Input->h*fhan_Input->fh;//跟新最速跟踪状态量微分x2
}


//原点附近有连线性段的连续幂次函数
float Fal_ADRC(float e,float alpha,float zeta)
{
int s=0;
float fal_output=0;
s=(Sign_ADRC(e+zeta)-Sign_ADRC(e-zeta))/2;
fal_output=e*s/(powf(zeta,1-alpha))+powf(ABS(e),alpha)*Sign_ADRC(e)*(1-s);
return fal_output;
}




/************扩张状态观测器********************/
//状态观测器参数beta01=1/h  beta02=1/(3*h^2)  beta03=2/(8^2*h^3) ...
void ESO_ADRC(Fhan_Data *fhan_Input)
{
  fhan_Input->e=fhan_Input->z1-fhan_Input->y;//状态误差

  fhan_Input->fe=Fal_ADRC(fhan_Input->e,0.5,fhan_Input->h);//非线性函数，提取跟踪状态与当前状态误差
  fhan_Input->fe1=Fal_ADRC(fhan_Input->e,0.25,fhan_Input->h);

  /*************扩展状态量更新**********/
  fhan_Input->z1+=fhan_Input->h*(fhan_Input->z2-fhan_Input->beta_01*fhan_Input->e);
  fhan_Input->z2+=fhan_Input->h*(fhan_Input->z3
                                 -fhan_Input->beta_02*fhan_Input->fe
                                   +fhan_Input->b0*fhan_Input->u);
 //ESO估计状态加速度信号，进行扰动补偿，传统MEMS陀螺仪漂移较大，估计会产生漂移
  fhan_Input->z3+=fhan_Input->h*(-fhan_Input->beta_03*fhan_Input->fe1);
}


/************非线性组合****************/
/*
void Nolinear_Conbination_ADRC(Fhan_Data *fhan_Input)
{
  float d=0,a0=0,y=0,a1=0,a2=0,a=0;
  float Sy=0,Sa=0;//ADRC状态跟踪误差项

  fhan_Input->h1=fhan_Input->N1*fhan_Input->h;

  d=fhan_Input->r*fhan_Input->h1*fhan_Input->h1;
  a0=fhan_Input->h1*fhan_Input->c*fhan_Input->e2;
  y=fhan_Input->e1+a0;
  a1=sqrt(d*(d+8*ABS(y)));
  a2=a0+Sign_ADRC(y)*(a1-d)/2;

  Sy=Fsg_ADRC(y,d);
  a=(a0+y-a2)*Sy+a2;
  Sa=Fsg_ADRC(a,d);
  fhan_Input->u0=-fhan_Input->r*((a/d)-Sign_ADRC(a))*Sa-fhan_Input->r*Sign_ADRC(a);

  //a=(a0+y)*Fsg_ADRC(y,d)+a2*(1-Fsg_ADRC(y,d));

  //fhan_Input->fh=-fhan_Input->r*(a/d)*Fsg_ADRC(a,d)
  //                -fhan_Input->r*Sign_ADRC(a)*(1-Fsg_ADRC(a,d));//得到最速微分加速度跟踪量
}
*/
void Nolinear_Conbination_ADRC(Fhan_Data *fhan_Input)        //非线性ADRC
{
  float temp_e2=0;
  temp_e2=Constrain_Float(fhan_Input->e2,-3000,3000);
  fhan_Input->u0=fhan_Input->beta_1*Fal_ADRC(fhan_Input->e1,fhan_Input->alpha1,fhan_Input->zeta)
                +fhan_Input->beta_2*Fal_ADRC(temp_e2,fhan_Input->alpha2,fhan_Input->zeta);

}


void ADRC_Control(Fhan_Data *fhan_Input,float expect_ADRC,float feedback_ADRC)
{
    /*自抗扰控制器第1步*/
      /*****
      安排过度过程，输入为期望给定，
      由TD跟踪微分器得到：
      过度期望信号x1，过度期望微分信号x2
      ******/
      Fhan_ADRC(fhan_Input,expect_ADRC);

    /*自抗扰控制器第2步*/
      /************系统输出值为反馈量，状态反馈，ESO扩张状态观测器的输入*********/
      fhan_Input->y=feedback_ADRC;
      /*****
      扩张状态观测器，得到反馈信号的扩张状态：
      1、状态信号z1；
      2、状态速度信号z2；
      3、状态加速度信号z3。
      其中z1、z2用于作为状态反馈与TD微分跟踪器得到的x1,x2做差后，
      经过非线性函数映射，乘以beta系数后，
      组合得到未加入状态加速度估计扰动补偿的原始控制量u
      *********/
      ESO_ADRC(fhan_Input);//低成本MEMS会产生漂移，扩展出来的z3此项会漂移，目前暂时未想到办法解决，未用到z3
    /*自抗扰控制器第3步*/
      /********状态误差反馈率***/
      fhan_Input->e0+=fhan_Input->e1*fhan_Input->h;//状态积分项
      fhan_Input->e1= fhan_Input->x1-fhan_Input->z1;//状态偏差项
      fhan_Input->e2= fhan_Input->x2-fhan_Input->z2;//状态微分项，
      /********线性组合*******/
     /*
      fhan_Input->u0=//fhan_Input->beta_0*fhan_Input->e0
                    +fhan_Input->beta_1*fhan_Input->e1
                    +fhan_Input->beta_2*fhan_Input->e2;
    */
      Nolinear_Conbination_ADRC(fhan_Input);
      /**********扰动补偿*******/
      //fhan_Input->u=fhan_Input->u0
      //             -fhan_Input->z3/fhan_Input->b0;
      //由于MEMS传感器漂移比较严重，当beta_03取值比较大时，长时间z3漂移比较大，目前不加入扰动补偿控制量
      fhan_Input->u=Constrain_Float(fhan_Input->u0,-10000,10000);
}

////////////////////////////////////////////////////////////////
//2024 12 23
//LADRC
//
void LADRC_Init(LADRC_NUM* LADRC_TYPE1,float h,float r,float wc,float w0,float b0) 
{
    LADRC_TYPE1->h = h;   // 定时时间及时间步长      0.001
    LADRC_TYPE1->r = r;   // 跟踪速度参数            50
    LADRC_TYPE1->wc = wc;  // 观测器带宽             25
    LADRC_TYPE1->w0 = w0;  // 状态误差反馈率带宽     25
    LADRC_TYPE1->b0 = b0;  // 系统参数               125
}

void LADRC_REST(LADRC_NUM* LADRC_TYPE1) {
    LADRC_TYPE1->z1 = 0;  // 定时时间及时间步长
    LADRC_TYPE1->z2 = 0;  // 跟踪速度参数
    LADRC_TYPE1->z3 = 0;  // 观测器带宽
}

void LADRC_TD(LADRC_NUM* LADRC_TYPE1, float Expect) 
{
    float fh = -LADRC_TYPE1->r * LADRC_TYPE1->r * (LADRC_TYPE1->v1 - Expect) -
               2 * LADRC_TYPE1->r * LADRC_TYPE1->v2;
    LADRC_TYPE1->v1 += LADRC_TYPE1->v2 * LADRC_TYPE1->h;
    LADRC_TYPE1->v2 += fh * LADRC_TYPE1->h;
}

void LADRC_ESO(LADRC_NUM* LADRC_TYPE1, float FeedBack) {
    float Beita_01 = 3 * LADRC_TYPE1->w0;
    float Beita_02 = 3 * LADRC_TYPE1->w0 * LADRC_TYPE1->w0;
    float Beita_03 = LADRC_TYPE1->w0 * LADRC_TYPE1->w0 * LADRC_TYPE1->w0;

    float e = LADRC_TYPE1->z1 - FeedBack;
    LADRC_TYPE1->z1 += (LADRC_TYPE1->z2 - Beita_01 * e) * LADRC_TYPE1->h;
    LADRC_TYPE1->z2 +=
        (LADRC_TYPE1->z3 - Beita_02 * e + LADRC_TYPE1->b0 * LADRC_TYPE1->u) *
        LADRC_TYPE1->h;
    LADRC_TYPE1->z3 += -Beita_03 * e * LADRC_TYPE1->h;
}

void LADRC_LF(LADRC_NUM* LADRC_TYPE1,float limit_out) {
    float Kp = LADRC_TYPE1->wc * LADRC_TYPE1->wc;
    float Kd = 2 * LADRC_TYPE1->wc;
    float e1 = LADRC_TYPE1->v1 - LADRC_TYPE1->z1;
    float e2 = LADRC_TYPE1->v2 - LADRC_TYPE1->z2;
    float u0 = Kp * e1 + Kd * e2;
    LADRC_TYPE1->u = (u0 - LADRC_TYPE1->z3) / LADRC_TYPE1->b0;
    if (LADRC_TYPE1->u > limit_out)
        LADRC_TYPE1->u = limit_out;
    else if (LADRC_TYPE1->u < -limit_out)
        LADRC_TYPE1->u = -limit_out;
}

void LADRC_Loop(LADRC_NUM* LADRC_TYPE1, float Expect, float RealTimeOut,float limit) 
{
    LADRC_TD(LADRC_TYPE1, Expect);
    LADRC_ESO(LADRC_TYPE1, RealTimeOut);
    LADRC_LF(LADRC_TYPE1,limit);
}

