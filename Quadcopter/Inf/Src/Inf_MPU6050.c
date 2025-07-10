#include "Inf_MPU6050.h"

static int Inf_MPU6050_WriteReg(uint8_t reg, uint8_t data)
{
    uint8_t data_to_send[2];
    data_to_send[0] = reg;
    data_to_send[1] = data;

    return HAL_I2C_Master_Transmit(&hi2c1, MPU6050_ADDRESS, data_to_send, 2, MPU6050_TIMEOUT);
}

static int Inf_MPU6050_ReadReg(uint8_t reg, uint8_t *data)
{
    return HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADDRESS, reg, 1, data, 1, MPU6050_TIMEOUT);
}
void Inf_MPU6050_Init(void){
    // 电源管理寄存器1，取消睡眠模式，选择时钟源为X轴锁相环
    Inf_MPU6050_WriteReg(MPU6050_REG_PWR_MGMT_1, 0x01);
    // 电源管理寄存器2，所有轴均不待机
    Inf_MPU6050_WriteReg(MPU6050_REG_PWR_MGMT_2, 0x00);
    /* 设置陀螺仪采样率和低通滤波器
        低通滤波器：数据输出速率是 1kHZ
        采样频率 = 陀螺仪输出频率/(1+SMPLRT_DIV)
        500Hz = 1000Hz/(1+1)
    */
    Inf_MPU6050_WriteReg(MPU6050_REG_SMPLRT_DIV, 0x01);
    Inf_MPU6050_WriteReg(MPU6050_REG_CONFIG, 0x01);
    // 角速度量程：±2000°/秒
    Inf_MPU6050_WriteReg(MPU6050_REG_GYRO_CONFIG, 0x18);
    // 加速度量程：±2g
    Inf_MPU6050_WriteReg(MPU6050_REG_ACCEL_CONFIG, 0x00);
}

int Inf_MPU6050_Read(ComGyroAndAccelData *data){
    uint8_t DataH,DataL;
    int err = 0;

    err |= Inf_MPU6050_ReadReg(MPU6050_REG_ACCEL_XOUT_H, &DataH);
    err |= Inf_MPU6050_ReadReg(MPU6050_REG_ACCEL_XOUT_L, &DataL);
    data->accelX = (DataH << 8) | DataL;

    err |= Inf_MPU6050_ReadReg(MPU6050_REG_ACCEL_YOUT_H, &DataH);
    err |= Inf_MPU6050_ReadReg(MPU6050_REG_ACCEL_YOUT_L, &DataL);
    data->accelY = (DataH << 8) | DataL;

    err |= Inf_MPU6050_ReadReg(MPU6050_REG_ACCEL_ZOUT_H, &DataH);
    err |= Inf_MPU6050_ReadReg(MPU6050_REG_ACCEL_ZOUT_L, &DataL);
    data->accelZ = (DataH << 8) | DataL;

    err |= Inf_MPU6050_ReadReg(MPU6050_REG_GYRO_XOUT_H, &DataH);
    err |= Inf_MPU6050_ReadReg(MPU6050_REG_GYRO_XOUT_L, &DataL);
    data->gyroX = (DataH << 8) | DataL;

    err |= Inf_MPU6050_ReadReg(MPU6050_REG_GYRO_YOUT_H, &DataH);
    err |= Inf_MPU6050_ReadReg(MPU6050_REG_GYRO_YOUT_L, &DataL);
    data->gyroY = (DataH << 8) | DataL;

    err |= Inf_MPU6050_ReadReg(MPU6050_REG_GYRO_ZOUT_H, &DataH);
    err |= Inf_MPU6050_ReadReg(MPU6050_REG_GYRO_ZOUT_L, &DataL);
    data->gyroZ = (DataH << 8) | DataL;

    return err;
}


void Inf_MPU6050_GyroAndAccelDataCalibrate(void){
    debug_printfln("MPU6050 Calibrate start");
    uint8_t cnt = 30;
    ComGyroAndAccelData current_data;
    ComGyroAndAccelData last_data;

    debug_printfln("MPU6050 Waiting for Stillness");
    while(cnt){
        Inf_MPU6050_Read(&current_data);
        // Com_Config_PrintMPU6050("Stillness", &current_data);
        if (abs(current_data.gyroX - last_data.gyroX) <= 10
            && abs(current_data.gyroY - last_data.gyroY) <= 10
            && abs(current_data.gyroZ - last_data.gyroZ) <= 10)
        {
            cnt--;
        }
        last_data = current_data;
        HAL_Delay(3);
    }
    debug_printfln("MPU6050 Stillness");

    int32_t sumBuff[6] = {0};
    for (uint16_t i = 0; i < 255; i++)
    {
        Inf_MPU6050_Read(&gyroAndAccelData);
        sumBuff[0] += gyroAndAccelData.accelX;
        sumBuff[1] += gyroAndAccelData.accelY;
        sumBuff[2] += gyroAndAccelData.accelZ - 16383;
        sumBuff[3] += gyroAndAccelData.gyroX;
        sumBuff[4] += gyroAndAccelData.gyroY;
        sumBuff[5] += gyroAndAccelData.gyroZ;
        HAL_Delay(3);
    }
    
    offsetGyroAndAccelData.accelX = sumBuff[0] / 255;
    offsetGyroAndAccelData.accelY = sumBuff[1] / 255;
    offsetGyroAndAccelData.accelZ = sumBuff[2] / 255;
    offsetGyroAndAccelData.gyroX = sumBuff[3] / 255;
    offsetGyroAndAccelData.gyroY = sumBuff[4] / 255;
    offsetGyroAndAccelData.gyroZ = sumBuff[5] / 255;

    // Com_Config_PrintMPU6050("offset", &offsetGyroAndAccelData);
    debug_printfln("MPU6050 Calibrate finish");
}


void Inf_MPU6050_ReadCalibratedGyroAndAccelData(void){
    Inf_MPU6050_Read(&gyroAndAccelData);

    gyroAndAccelData.accelX -= offsetGyroAndAccelData.accelX;
    gyroAndAccelData.accelY -= offsetGyroAndAccelData.accelY;
    gyroAndAccelData.accelZ -= offsetGyroAndAccelData.accelZ;
    gyroAndAccelData.gyroX -= offsetGyroAndAccelData.gyroX;
    gyroAndAccelData.gyroY -= offsetGyroAndAccelData.gyroY;
    gyroAndAccelData.gyroZ -= offsetGyroAndAccelData.gyroZ;
}

