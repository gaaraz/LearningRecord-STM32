#include "driver_ir_receiver.h"
#include "tim.h"
#include <string.h>


static int state = 0;
static uint8_t data[4]= {0, 0, 0, 0};
static uint8_t cnt = 0;


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
    if(htim->Instance == TIM1){
        // 下降沿捕获
        if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
            if (state == 0)
            {
                return;
            }
            
            if(state == 1){
                uint32_t ccr1 = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_1);
                if (ccr1 > 3500 && ccr1 < 5500)
                {
                    state = 2;
                }else{
                    state = 0;
                }
                __HAL_TIM_SET_COUNTER(htim, 0);   
                return;
            }else if(state == 2){
                uint32_t ccr1 = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_1);
                uint32_t ccr2 = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_2);
                uint32_t pulse_width = ccr1 - ccr2;
                if (pulse_width > 460 && pulse_width < 660)
                {
                    
                }else if(pulse_width > 1580 && pulse_width < 1780){
                    data[cnt/8] |= 1<<(cnt%8);
                }else{
                    state = 0;
                }
                if (cnt < 31)
                {
                    cnt++;
                }
                
                __HAL_TIM_SET_COUNTER(htim, 0);   
                return;
            }
        }
        // 上升沿捕获
        if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){
            // HAL_TIM_IC_Stop_IT(htim, TIM_CHANNEL_2);
            uint32_t ccr1 = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_1);
            uint32_t ccr2 = __HAL_TIM_GET_COMPARE(htim, TIM_CHANNEL_2);
            uint32_t pulse_width = ccr2 - ccr1;
            if(pulse_width > 0){
                if(pulse_width > 100000){
                    state = 0;
                    __HAL_TIM_SET_COUNTER(htim, 0);
                    return;
                }
                if(state == 0){
                    if (pulse_width > 8000 && pulse_width < 10000)
                    {
                        state = 1;
                    }else{
                        state = 0;
                    }
                    __HAL_TIM_SET_COUNTER(htim, 0);
                    return;
                }
            }
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
    if(htim->Instance == TIM1){
        // HAL_TIM_Base_Stop_IT(htim);
        state = 0;
        cnt = 0;
        memset(data, 0, sizeof(data));
    }
}