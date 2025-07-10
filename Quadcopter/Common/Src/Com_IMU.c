#include "Com_IMU.h"
#include "math.h"

/* 计算欧拉角用到的3个参数 */
float RtA = 57.2957795f;                        //弧度->度
/* 陀螺仪初始化量程+-2000度/秒  */
float Gyro_G = 4000.0/65536;                    // 度/s
float Gyro_Gr = 4000.0/65536/180*3.1415926;     // 弧度/s
#define squa(Sq)  ((float)Sq) * ((float)Sq)     // 平方

static float Q_rsqrt(float number)
{
    long i;
    float x2, y;
    const float threehalfs = 1.5F;

    x2 = number * 0.5F;
    y  = number;
    i  = * (long *) &y;
    i  = 0x5f3759df - (i >> 1);
    y  = * (float *) &i;
    y  = y * (threehalfs - (x2 * y * y));   // 1st iteration c第一牛顿迭代
    return y;
}

static double normAccz;     // z轴上的加速度
/**
 * 根据mpu的6轴数据，获取表征姿态的欧拉角
 * ComGyroAndAccelData *gyroAndAccelData mpu6轴数据
 * ComEulerAngleStruct *eulerAngle 欧拉角数据
 * dt 采样周期（s）
 */
void Com_IMU_GetEulerAgle(ComGyroAndAccelData *gyroAndAccelData,
                            ComEulerAngleStruct *eulerAngle,
                            float dt)
{
    volatile struct V
    {
        float x;
        float y;
        float z;
    }Gravity, Acc, Gyro, AccGravity;
    
    static struct V GyroIntegError = {0};
    static float KpDef = 0.8f;
    static float KiDef = 0.0003f;
    static QuaternionStruct NumQ = {1, 0, 0, 0};
    float  q0_t, q1_t, q2_t, q3_t;
    float  NormQuat;
    float  HalfTime = dt * 0.5f;

    // 提取等效旋转矩阵中的重力分量
    Gravity.x = 2 * (NumQ.q1 * NumQ.q3 - NumQ.q0 * NumQ.q2);
    Gravity.y = 2 * (NumQ.q0 * NumQ.q1 + NumQ.q2 * NumQ.q3);
    Gravity.z = 1 - 2 * (NumQ.q1 * NumQ.q1 + NumQ.q2 * NumQ.q2);

    // 加速度归一化
    NormQuat = Q_rsqrt(squa(gyroAndAccelData->accelX)
                        +squa(gyroAndAccelData->accelY)
                        +squa(gyroAndAccelData->accelZ));
    
                        
    Acc.x = gyroAndAccelData->accelX * NormQuat;
    Acc.y = gyroAndAccelData->accelY * NormQuat;
    Acc.z = gyroAndAccelData->accelZ * NormQuat;

    // 向量差乘得出的值
    AccGravity.x = (Acc.y * Gravity.z - Acc.z * Gravity.y);
    AccGravity.y = (Acc.z * Gravity.x - Acc.x * Gravity.z);
    AccGravity.z = (Acc.x * Gravity.y - Acc.y * Gravity.x);

    // 再做加速度积分补偿角速度的补偿值
    GyroIntegError.x += AccGravity.x * KiDef;
    GyroIntegError.y += AccGravity.y * KiDef;
    GyroIntegError.z += AccGravity.z * KiDef;

    // 角速度融合加速度积分补偿值
    Gyro.x = gyroAndAccelData->gyroX * Gyro_Gr + KpDef * AccGravity.x + GyroIntegError.x;
    Gyro.y = gyroAndAccelData->gyroY * Gyro_Gr + KpDef * AccGravity.y + GyroIntegError.y;
    Gyro.z = gyroAndAccelData->gyroZ * Gyro_Gr + KpDef * AccGravity.z + GyroIntegError.z;

    // 一阶龙格库塔法，更新四元数
    q0_t = (-NumQ.q1*Gyro.x - NumQ.q2*Gyro.y - NumQ.q3*Gyro.z) * HalfTime;
    q1_t = ( NumQ.q0*Gyro.x - NumQ.q3*Gyro.y + NumQ.q2*Gyro.z) * HalfTime;
    q2_t = ( NumQ.q3*Gyro.x + NumQ.q0*Gyro.y - NumQ.q1*Gyro.z) * HalfTime;
    q3_t = (-NumQ.q2*Gyro.x + NumQ.q1*Gyro.y + NumQ.q0*Gyro.z) * HalfTime;

    NumQ.q0 += q0_t;
    NumQ.q1 += q1_t;
    NumQ.q2 += q2_t;
    NumQ.q3 += q3_t;

    // 四元数归一化
    NormQuat = Q_rsqrt(squa(NumQ.q0) + squa(NumQ.q1) + squa(NumQ.q2) + squa(NumQ.q3));
    NumQ.q0 *= NormQuat;
    NumQ.q1 *= NormQuat;
    NumQ.q2 *= NormQuat;
    NumQ.q3 *= NormQuat;

    // 机体坐标系下的Z方向向量
    float vecxZ = 2*NumQ.q0*NumQ.q2 - 2*NumQ.q1*NumQ.q3;    //矩阵(3,1)项
    float vecyZ = 2*NumQ.q2*NumQ.q3 + 2*NumQ.q0*NumQ.q1;    //矩阵(3,2)项
    float veczZ = 1 - 2*squa(NumQ.q1) - 2*squa(NumQ.q2);    //矩阵(3,3)项

    // 将Z轴角速度l陀螺仪值 转化为Z角度/秒  Gyro_G/* 陀螺仪初始化量程+-2000度/秒  */
    float yaw_G = gyroAndAccelData->accelZ * Gyro_G;  
    if ((yaw_G > 0.5f) || (yaw_G < -0.5f))
    {
        eulerAngle->yaw += yaw_G * dt;  // 角速度积分成偏航角
    }
    
    eulerAngle->pitch = asin(vecxZ) * RtA;          //俯仰角
    eulerAngle->roll = atan2f(vecyZ, veczZ) * RtA;  //横滚角

    // Z轴垂直方向上的加速度，此值涵盖了倾斜时在Z轴角速度的向量和，不是单纯重力感应得出的值
    normAccz = gyroAndAccelData->accelX * vecxZ 
                + gyroAndAccelData->accelY * vecyZ 
                + gyroAndAccelData->accelZ * veczZ;
}

float Com_IMU_GetNormAccZ(void){
    return normAccz;
}
