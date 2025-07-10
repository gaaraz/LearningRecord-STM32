#include "App_Flight.h"

/*  6个PID结构体*/
/* 俯仰 */
PIDStruct pitchPID = {.kp = -7.0f, .ki = 0.0f, .kd = 0.0f};      //加速度PID 外环
PIDStruct gyroYPID = {.kp = 2.0f, .ki = 0.0f, .kd = 0.1f};      //角速度PID 内环

/* 横滚 */
PIDStruct rollPID = {.kp = -7.0f, .ki = 0.0f, .kd = 0.0f};       //加速度PID
PIDStruct gyroXPID = {.kp = -2.0f, .ki = 0.0f, .kd = -0.1f};      //角速度PID

/* 偏航 */
PIDStruct yawPID = {.kp = -2.2f, .ki = 0.0f, .kd = 0.0f};        //加速度PID
PIDStruct gyroZPID = {.kp = -1.5f, .ki = 0.0f, .kd = 0.0f};      //角速度PID



void App_Flight_Start(void)
{
    /* 初始化motor */
    Inf_Motor_Init();

    Inf_MPU6050_Init();
    Inf_MPU6050_GyroAndAccelDataCalibrate();
}

/**
 * 对六轴数据做滤波
 * 角速度：一阶低通滤波
 * 加速度：卡尔曼滤波
 */
void App_Flight_GetGyroAccelWithFilter(void)
{
    taskENTER_CRITICAL();
    Inf_MPU6050_ReadCalibratedGyroAndAccelData();
    taskEXIT_CRITICAL();

    static int16_t lastDatas[3] = {0};
    // Com_Config_PrintMPU6050("1", &gyroAndAccelData);
    // Com_Config_PrintFilter(gyroAndAccelData.gyroX, 
    //     gyroAndAccelData.gyroY, 
    //     gyroAndAccelData.gyroZ,
    //     Com_Filter_LowPassFilter(gyroAndAccelData.gyroX, lastDatas[0]),
    //     Com_Filter_LowPassFilter(gyroAndAccelData.gyroY, lastDatas[1]),
    //     Com_Filter_LowPassFilter(gyroAndAccelData.gyroZ, lastDatas[2])
    // );
    gyroAndAccelData.gyroX = Com_Filter_LowPassFilter(gyroAndAccelData.gyroX, lastDatas[0]);
    gyroAndAccelData.gyroY = Com_Filter_LowPassFilter(gyroAndAccelData.gyroY, lastDatas[1]);
    gyroAndAccelData.gyroZ = Com_Filter_LowPassFilter(gyroAndAccelData.gyroZ, lastDatas[2]);
    
    lastDatas[0] = gyroAndAccelData.gyroX;
    lastDatas[1] = gyroAndAccelData.gyroY;
    lastDatas[2] = gyroAndAccelData.gyroZ;

    // Com_Config_PrintFilter(gyroAndAccelData.accelX, 
    //     gyroAndAccelData.accelY, 
    //     gyroAndAccelData.accelZ,
    //     Com_Filter_KalmanFilter(&kfs[0], gyroAndAccelData.accelX),
    //     Com_Filter_KalmanFilter(&kfs[1], gyroAndAccelData.accelY),
    //     Com_Filter_KalmanFilter(&kfs[2], gyroAndAccelData.accelZ)
    // );
    gyroAndAccelData.accelX = Com_Filter_KalmanFilter(&kfs[0], gyroAndAccelData.accelX);
    gyroAndAccelData.accelY = Com_Filter_KalmanFilter(&kfs[1], gyroAndAccelData.accelY);
    gyroAndAccelData.accelZ = Com_Filter_KalmanFilter(&kfs[2], gyroAndAccelData.accelZ);
    // Com_Config_PrintMPU6050("2", &gyroAndAccelData);
    // Com_Config_PrintFilter(&gyroAndAccelData);
}

/**
 * 获取欧拉角
 */
void App_Flight_GetEulerAngle(ComGyroAndAccelData *gyroAndAccelData,
                                ComEulerAngleStruct *eulerAngle,
                                float dt)
{
    Com_IMU_GetEulerAgle(gyroAndAccelData, eulerAngle, dt);
}

/**
 * 计算姿态pid
 */
void App_Flight_PIDPosture(ComGyroAndAccelData *gyroAndAccelData,
                            ComEulerAngleStruct *eulerAngle,
                            float dt)
{
    /* 俯仰 */
    pitchPID.dt  = dt;
    pitchPID.desire = (joyStick.PIT - 500) * 0.04;
    pitchPID.measure = eulerAngle->pitch;

    gyroYPID.dt  = dt;
    gyroYPID.measure = gyroAndAccelData->gyroY * Gyro_G;

    Com_PID_CascadePID(&pitchPID, &gyroYPID);

    /* 横滚 */
    rollPID.dt  = dt;
    rollPID.desire = (joyStick.ROL - 500) * 0.04;
    rollPID.measure = eulerAngle->roll;

    gyroXPID.dt  = dt;
    gyroXPID.measure = gyroAndAccelData->gyroX * Gyro_G;

    Com_PID_CascadePID(&rollPID, &gyroXPID);

    /* 偏航 */
    yawPID.dt  = dt;
    yawPID.desire = (joyStick.YAW - 500) * 0.04;
    yawPID.measure = eulerAngle->yaw;

    gyroZPID.dt  = dt;
    gyroZPID.measure = gyroAndAccelData->gyroZ * Gyro_G;

    Com_PID_CascadePID(&yawPID, &gyroZPID);
}

/**
 * 姿态PID作用到电机上
 */
void App_Flight_MotorWithPosturePID(void)
{
    if (isRemoteUnlocked != COM_OK) return;

    /**
     * leftTop  leftBottom  rightTop  rightBottom
     * 
     * 横滚：gyroXPID.result
     *      leftTop + leftBottom VS rightTop + rightBottom
     * 俯仰：gyroYPID.result
     *      leftTop + rightTop VS leftBottom + rightBottom
     * 偏航：gyroZPID.result
     *      leftTop + rightBottom VS leftBottom + rightTop
     */

     int16_t speed = joyStick.THR * 0.7;
     motorLeftTop.speed     = speed + gyroXPID.result + gyroYPID.result + gyroZPID.result;
     motorLeftBottom.speed  = speed + gyroXPID.result - gyroYPID.result - gyroZPID.result;
     motorRightTop.speed    = speed - gyroXPID.result + gyroYPID.result - gyroZPID.result;
     motorRightBottom.speed = speed - gyroXPID.result - gyroYPID.result + gyroZPID.result;
}


/* 飞行器工作*/
void App_Flight_Work(void)
{
    if (isRemoteUnlocked != COM_OK || joyStick.THR <= 30)
    {
        motorLeftTop.speed = 
            motorLeftBottom.speed = 
                motorRightTop.speed = 
                    motorRightBottom.speed = 0;
    }
        
    Inf_Motor_AllMotorsWork();
}

/**
 * 如何调节pid：
 *      内环 p
 *          确定极性：
 *              错误：加速下跌
 *              正确：缓慢平滑的下跌
 *          大小：
 *              下跌的时候要平滑
 *      外环 p
 *          确定极性：
 *              正确：回到初始位置
 *              错误：旋转
 *      内环 d
 *          抑制震荡
 *          确定极性：
 *              错误：震荡加剧
 *              正确：震荡减弱
 *          大小：
 *              既能快速回到初始位置，还没有震荡
 *      外环 d（可选）
 *          如果内环的d抑制不了震荡，再调节外环的d
 *      内环 i（可选）
 *      外环 i（可选）
 */