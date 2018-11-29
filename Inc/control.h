#ifndef __CONTROL_H__
#define __CONTROL_H__
#include "gpio.h"

#define HeigthStd 200             //判断隧道内外的阈值，单位Cm
#define UnitWheelLength 20        //单位轮子周长，单位Cm /*注：如果精度不够可在轮子上等分的多装几个磁块*/
/*
* 运行速度计算：单位轮子周长X霍尔传感器返回的频率
*/
/*
* 运行里程计算：单位时间X单位速度 求和
*/
#define InsideOrOutsideStatus_ON    HAL_GPIO_WritePin(InsideOrOutsideStatus_GPIO_Port,InsideOrOutsideStatus_Pin,GPIO_PIN_SET)   // 输出高电平
#define InsideOrOutsideStatus_OFF   HAL_GPIO_WritePin(InsideOrOutsideStatus_GPIO_Port,InsideOrOutsideStatus_Pin,GPIO_PIN_RESET) // 输出低电平

#define OutputStatus_Brake_OFF      HAL_GPIO_WritePin(OutputStatus_Brake_GPIO_Port,OutputStatus_Brake_Pin,GPIO_PIN_SET)         // 输出高电平
#define OutputStatus_Brake_ON       HAL_GPIO_WritePin(OutputStatus_Brake_GPIO_Port,OutputStatus_Brake_Pin,GPIO_PIN_RESET)       // 输出低电平

#define OutputStatus_Forward_OFF    HAL_GPIO_WritePin(OutputStatus_Forward_GPIO_Port,OutputStatus_Forward_Pin,GPIO_PIN_SET)     // 输出高电平
#define OutputStatus_Forward_ON     HAL_GPIO_WritePin(OutputStatus_Forward_GPIO_Port,OutputStatus_Forward_Pin,GPIO_PIN_RESET)   // 输出低电平

#define OutputStatus_Backward_OFF   HAL_GPIO_WritePin(OutputStatus_Backward_GPIO_Port,OutputStatus_Backward_Pin,GPIO_PIN_SET)   // 输出高电平
#define OutputStatus_Backward_ON    HAL_GPIO_WritePin(OutputStatus_Backward_GPIO_Port,OutputStatus_Backward_Pin,GPIO_PIN_RESET) // 输出低电平

#define OutputStatus_Led_OFF        HAL_GPIO_WritePin(OutputStatus_Led_GPIO_Port,OutputStatus_Led_Pin,GPIO_PIN_SET)             // 输出高电平
#define OutputStatus_Led_ON         HAL_GPIO_WritePin(OutputStatus_Led_GPIO_Port,OutputStatus_Led_Pin,GPIO_PIN_RESET)           // 输出低电平

#define OutputStatus_Camera_OFF     HAL_GPIO_WritePin(OutputStatus_Camera_GPIO_Port,OutputStatus_Camera_Pin,GPIO_PIN_SET)             // 输出高电平
#define OutputStatus_Camera_ON      HAL_GPIO_WritePin(OutputStatus_Camera_GPIO_Port,OutputStatus_Camera_Pin,GPIO_PIN_RESET)

#define BarrierStatus               HAL_GPIO_ReadPin(BarrierStatus_GPIO_Port,BarrierStatus_Pin)                                 //障碍物状态检测


void barrierScan(void);
uint32_t filter(uint16_t *Array);
void ifInsideOrOutside(void);
void sendEnd(void);
void updateData(void);

#endif
