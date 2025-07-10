#include "App_Communication.h"


static uint8_t check_conn_cnt = 200;
static uint8_t thr_max_duration = 0;
static uint8_t thr_min_duration = 0;
static ComRemoteStatus remote_status = THR_FREE;
static uint32_t low_duration = 0;


/* 通信模块初始化 */
void App_Communication_Start(void){
    debug_printfln("App_Communication_Start");

    /* 初始化SI24R1 */
    Inf_SI24R1_Init();
    while (Inf_SI24R1_Check()){
        HAL_Delay(500);
    }

    Inf_SI24R1_RX_Mode();
    debug_printfln("App_Communication_Start finished");
}


/* 通信接收摇杆和n按键数据 */
Com_Status App_Communication_ReceiveJoyStickData(void){
    taskENTER_CRITICAL();
    Com_Status r = Inf_SI24R1_RxPacket(RX_BUF);
    taskEXIT_CRITICAL();
    if ( r == 1) return COM_FAIL;

    if(RX_BUF[0] != FRAME_0 || 
        RX_BUF[1] != FRAME_1 ||
        RX_BUF[2] != FRAME_2)
    {
        return COM_FAIL;
    }

    joyStick.THR = (RX_BUF[4] << 8) | RX_BUF[5];
    joyStick.YAW = (RX_BUF[6] << 8) | RX_BUF[7];
    joyStick.PIT = (RX_BUF[8] << 8) | RX_BUF[9];
    joyStick.ROL = (RX_BUF[10] << 8) | RX_BUF[11];

    joyStick.isPowerOff = RX_BUF[12];

    if (RX_BUF[13])
    {
        joyStick.isFixHeight = !joyStick.isFixHeight;
    }

    return COM_OK;
}


/* 检查通信是否连接 */
Com_Status App_Communication_CheckConnection(Com_Status isReceiveData){
    if (isReceiveData == COM_OK) {
        check_conn_cnt = 0;
        return COM_OK;
    }
    
    check_conn_cnt++;
    if (check_conn_cnt > 200) {
        check_conn_cnt = 200;
        return COM_FAIL;
    }
    return COM_OK;
}


/* 解锁遥控器 */
Com_Status App_Communication_RemoteUnlock(void){
    if (isRemoteConnected != COM_OK) {
        remote_status = THR_FREE;
        return COM_FAIL;
    }
    
    switch (remote_status)
    {
        case THR_FREE:
            thr_max_duration = 0;
            thr_min_duration = 0;
            if (joyStick.THR >= 960) {
                remote_status = THR_MAX;
            }
            break;

        case THR_MAX:
            if (joyStick.THR >= 960) {
                thr_max_duration++;
                if (thr_max_duration > 200 ) {
                    remote_status = THR_MAX_LEAVE;
                }
            } else {
                remote_status = THR_FREE;
            }
            break;

        case THR_MAX_LEAVE:
            if (joyStick.THR <= 20) {
                remote_status = THR_MIN;
            }
            break;

        case THR_MIN:
            if (joyStick.THR <= 20) {
                thr_min_duration++;
                if (thr_min_duration > 200 ) {
                    remote_status = THR_UNLOCK;
                }
            } else {
                remote_status = THR_FREE;
            }
            break;

        case THR_UNLOCK:
            if (joyStick.THR <= 20)
            {
                low_duration++;
                if (low_duration > 200 * 50) {
                    low_duration = 0;
                    remote_status = THR_FREE;
                }
            }else{
                low_duration = 0;
            }
            
            return COM_OK;
        default:
            break;
    }
}

