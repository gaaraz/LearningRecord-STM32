#include "Com_PID.h"

/* 计算PID */
void Com_PID_ComputePID(PIDStruct *pid)
{
    /* 计算误差 */
    float error = pid->measure - pid->desire;
    /* 比例项 */
    float pV = pid->kp * error;
    /* 积分项 */
    pid->integral += pid->ki * error * pid->dt;
    /* 微分项 */
    float dV = pid->kd * (error - pid->lastError) / pid->dt;
    pid->lastError = error;

    /* 计算结果 */
    pid->result = pV + pid->integral + dV;
}


/* 计算级联PID */
void Com_PID_CascadePID(PIDStruct *out, PIDStruct *in)
{
    /* 计算外环PID */
    Com_PID_ComputePID(out);
    /* 把外环的输出值设置内环目标值 */
    in->desire = out->result;
    /* 计算内环PID */
    Com_PID_ComputePID(in);
}