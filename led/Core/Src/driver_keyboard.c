#include "driver_keyboard.h"

uint8_t Keyboard_GetNum(void){
    uint8_t KeyNum = 0;
    if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_RESET)
    {
        HAL_Delay(20);
        while (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_RESET);
        HAL_Delay(20);
        KeyNum = 1;
    }

    return KeyNum;
}