#include "driver_W25Q64.h"
#include "spi.h"

uint8_t write_enable[] = {0x06};
uint8_t read_status_register1[] = {0x05};
uint8_t jedec_id[] = {0x9F};

void ReadID(uint8_t *mid, uint16_t *did){
    //片选信号，低电平开启
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

    // 读ID
    HAL_SPI_Transmit(&hspi1, jedec_id, 1, HAL_MAX_DELAY);

    HAL_SPI_Receive(&hspi1, mid, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, (uint8_t*)did, 2, HAL_MAX_DELAY);

    //片选信号，低电平开启
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

void WriteEnable(void){
    //片选信号，低电平开启
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

    // 写使能
    HAL_SPI_Transmit(&hspi1, write_enable, 1, HAL_MAX_DELAY);

    //片选信号，低电平开启
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

uint8_t WaitBusy(){
    uint32_t timeout;
    //片选信号，低电平开启
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

    uint8_t status_register1;
    HAL_SPI_Transmit(&hspi1, read_status_register1, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi1, &status_register1, 1, HAL_MAX_DELAY);
   
    timeout = 100000;
    while (status_register1 & 0x01)
    {
        timeout--;
        if (timeout == 0)
        {
            break;
        }
        HAL_SPI_Receive(&hspi1, &status_register1, 1, HAL_MAX_DELAY);
    }
    
    //片选信号，低电平开启
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

}



void SectorErase(uint8_t *addr){
    uint8_t sector_erase[4];
    sector_erase[0] = 0x20;
    for (uint8_t i = 0; i < 3; i++)
    {
        sector_erase[i + 1] = addr[i];
    }

    WriteEnable();

    //片选信号，低电平开启
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

    // 扇区擦除
    HAL_SPI_Transmit(&hspi1, sector_erase, 4, HAL_MAX_DELAY);

    //片选信号，关闭
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    WaitBusy();
}

void PageProgram(uint8_t *addr, uint8_t *data, uint16_t data_size){
    uint8_t command[4];
    command[0] = 0x02;
    for (uint8_t i = 0; i < 3; i++)
    {
        command[i + 1] = addr[i];
    }

    WriteEnable();

    //片选信号，低电平开启
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);

    HAL_SPI_Transmit(&hspi1, command, 4, HAL_MAX_DELAY);

    // 页编程
    HAL_SPI_Transmit(&hspi1, data, data_size, HAL_MAX_DELAY);

    //片选信号，关闭
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
    WaitBusy();
}

void ReadData(uint8_t *addr, uint8_t *data, uint16_t count){
    uint8_t command[4];
    command[0] = 0x03;
    for (uint8_t i = 0; i < 3; i++)
    {
        command[i + 1] = addr[i];
    }
    
    //片选信号，低电平开启
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
    
    HAL_SPI_Transmit(&hspi1, command, 4, HAL_MAX_DELAY);

    // 读数据
    HAL_SPI_Receive(&hspi1, data, count, HAL_MAX_DELAY);


    //片选信号，关闭
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}
