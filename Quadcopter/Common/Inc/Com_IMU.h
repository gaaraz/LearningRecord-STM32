#ifndef __COM_IMU_H__
#define __COM_IMU_H__


/* Includes */
#include "main.h"
#include "Com_Debug.h"
#include "Com_Config.h"

/* 计算欧拉角用到的3个参数 */
extern float RtA;                          //弧度->度
/* 陀螺仪初始化量程+-2000度/秒  */
extern float Gyro_G;                       // 度/s
extern float Gyro_Gr;                      // 弧度/s

void Com_IMU_GetEulerAgle(ComGyroAndAccelData *gyroAndAccelData,
                            ComEulerAngleStruct *eulerAngle,
                            float dt);

float Com_IMU_GetNormAccZ(void);

#endif