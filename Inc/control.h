#ifndef __CONTROL_H__
#define __CONTROL_H__
#include "gpio.h"

#define InsideOrOutsideStatus_ON    HAL_GPIO_WritePin(InOrOutStatus_GPIO_Port,InOrOutStatus_Pin,GPIO_PIN_SET)                   // 输出高电平
#define InsideOrOutsideStatus_OFF   HAL_GPIO_WritePin(InOrOutStatus_GPIO_Port,InOrOutStatus_Pin,GPIO_PIN_RESET)                 // 输出低电平

#define OutputStatus_Brake_ON       HAL_GPIO_WritePin(OutputStatus_Brake_GPIO_Port,OutputStatus_Brake_Pin,GPIO_PIN_SET)         // 输出高电平
#define OutputStatus_Brake_OFF      HAL_GPIO_WritePin(OutputStatus_Brake_GPIO_Port,OutputStatus_Brake_Pin,GPIO_PIN_RESET)       // 输出低电平

#define OutputStatus_Forward_ON     HAL_GPIO_WritePin(OutputStatus_Forward_GPIO_Port,OutputStatus_Forward_Pin,GPIO_PIN_SET)     // 输出高电平
#define OutputStatus_Forward_OFF    HAL_GPIO_WritePin(OutputStatus_Forward_GPIO_Port,OutputStatus_Forward_Pin,GPIO_PIN_RESET)   // 输出低电平

#define OutputStatus_Backward_ON    HAL_GPIO_WritePin(OutputStatus_Backward_GPIO_Port,OutputStatus_Backward_Pin,GPIO_PIN_SET)   // 输出高电平
#define OutputStatus_Backward__OFF  HAL_GPIO_WritePin(OutputStatus_Backward_GPIO_Port,OutputStatus_Backward_Pin,GPIO_PIN_RESET) // 输出低电平

#define OutputStatus_Led_ON         HAL_GPIO_WritePin(OutputStatus_Led_GPIO_Port,OutputStatus_Led_Pin,GPIO_PIN_SET)             // 输出高电平
#define OutputStatus_Led__OFF       HAL_GPIO_WritePin(OutputStatus_Led_GPIO_Port,OutputStatus_Led_Pin,GPIO_PIN_RESET)           // 输出低电平

#define BarrierStatus               HAL_GPIO_ReadPin(BarrierStatus_GPIO_Port,BarrierStatus_Pin)                            //障碍物状态检测


void barrierScan(void);

#endif
