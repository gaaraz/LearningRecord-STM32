#include "Com_Filter.h"

/* 一阶低通滤波 */
#define ALPHA 0.8
int16_t Com_Filter_LowPassFilter(int16_t newData, int16_t lastData)
{
    return ALPHA * lastData + (1 - ALPHA) * newData;
}

/* 卡尔曼滤波 */
KalmaFilterStruct kfs[3] = {
    {0.02, 0, 0, 0, 0.001, 0.543},
    {0.02, 0, 0, 0, 0.001, 0.543},
    {0.02, 0, 0, 0, 0.001, 0.543}
};
double Com_Filter_KalmanFilter(KalmaFilterStruct *kf, double input)
{
    kf->NowP = kf->LastP + kf->Q;
    kf->Kg = kf->NowP / (kf->NowP + kf->R);
    kf->out = kf->out + kf->Kg * (input - kf->out);
    kf->LastP = (1 - kf->Kg) * kf->NowP;
    return kf->out;
}