#include "bsp_bull_ctr.h"
#include "bsp_tr.h"
float Q_max;//热量上限
float Q_cd;//冷却速度
float Q_warn;//热量预警//拨弹盘降速//200
float Q_satu;//热量饱和阈值//拨弹频率与冷却速度相同//50
float Q_stop;//热量限制阈值//停止拨弹//20
float Q_cur;
float Q_res;
float Q_res;
float bullet_fre;
extern measure_chassis chassis_data;
//拨盘频率=（角度环/60/36/）*7
//热量控制算法，自动调整拨弹频率，适用自瞄
void bullet_fre_ctrl(void)
{
	
	
	Q_res = chassis_data.Q_max - chassis_data.Q_now;
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
                bullet_fre = (chassis_data.Q_cd/7)*36*60;
            }
        }
        else
        {
             bullet_fre = (chassis_data.Q_cd/7)*36*60;
        }
    }
    else
    {
        bullet_fre = (Q_res*(15-chassis_data.Q_cd)/150 + (15 + 1.34*chassis_data.Q_cd)/2000)*36*60;
    }
}
