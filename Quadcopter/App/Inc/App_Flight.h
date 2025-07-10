#ifndef __APP_FLIGHT_H__
#define __APP_FLIGHT_H__

#include "FreeRTOS.h"
#include "task.h"

#include "Inf_Motor.h"
#include "Inf_MPU6050.h"
#include "Com_Debug.h"
#include "Com_Filter.h"
#include "Com_IMU.h"
#include "Com_PID.h"

void App_Flight_Start(void);

void App_Flight_Work(void);

void App_Flight_GetGyroAccelWithFilter(void);

void App_Flight_GetEulerAngle(ComGyroAndAccelData *gyroAndAccelData,
                                ComEulerAngleStruct *eulerAngle,
                                float dt);

void App_Flight_PIDPosture(ComGyroAndAccelData *gyroAndAccelData,
                            ComEulerAngleStruct *eulerAngle,
                            float dt);

void App_Flight_MotorWithPosturePID(void);

#endif