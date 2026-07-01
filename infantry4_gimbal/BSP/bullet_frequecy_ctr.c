#include "bullet_frequency_ctr.h"

float Q_max;//热量上限
float Q_cd;//冷却速度
float Q_warn;//热量预警//拨弹盘降速//200
float Q_satu;//热量饱和阈值//拨弹频率与冷却速度相同//50
float Q_stop;//热量限制阈值//停止拨弹//20
float Q_cur;
float Q_res;
float Q_res = Q_max - Q_cur;
float bullet_fre;
//拨盘频率=（角度环/60/36/）*7
void bullet_fre_ctrl(void)
{
    if(Q_res < Q_warn)
    {
        if(Q_res <= Q_satu)
        {
            if(Q_res <= Q_stop)
            {
                bullet_fre = 0;//停止拨弹
            }
            else
            {
                bullet_fre = (Q_cd/7)*36*60;
            }
        }
        else
        {
             bullet_fre = (Q_cd/7)*36*60;
        }
    }
    else
    {
        bullet_fre = Q_res*(15-Q_cd)/150 + (15 + 1.34*Q_cd)/2000
    }
}
