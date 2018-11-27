#ifndef __Led_H__
#define __Led_H__
#include "stdint.h"
#include "gpio.h"
#define LedON   1
#define LedOFF  0
#define Led_ON                       HAL_GPIO_WritePin(Led_1_GPIO_Port,Led_1_Pin,GPIO_PIN_SET)    // 输出高电平
#define Led_OFF                      HAL_GPIO_WritePin(Led_1_GPIO_Port,Led_1_Pin,GPIO_PIN_RESET)  // 输出低电平
#define LED_TOGGLE                   HAL_GPIO_TogglePin(Led_1_GPIO_Port,Led_1_Pin)                // 输出反转


void Led_Set(uint8_t Led, uint8_t Status);

#endif
