#ifndef __CONTROL_H__
#define __CONTROL_H__
#include "gpio.h"

#define HeigthStd 200             //�ж�����������ֵ����λCm
#define UnitWheelLength 20        //��λ�����ܳ�����λCm /*ע��������Ȳ������������ϵȷֵĶ�װ�����ſ�*/
/*
* �����ٶȼ��㣺��λ�����ܳ�X�������������ص�Ƶ��
*/
/*
* ������̼��㣺��λʱ��X��λ�ٶ� ���
*/
#define InsideOrOutsideStatus_ON    HAL_GPIO_WritePin(InsideOrOutsideStatus_GPIO_Port,InsideOrOutsideStatus_Pin,GPIO_PIN_SET)   // ����ߵ�ƽ
#define InsideOrOutsideStatus_OFF   HAL_GPIO_WritePin(InsideOrOutsideStatus_GPIO_Port,InsideOrOutsideStatus_Pin,GPIO_PIN_RESET) // ����͵�ƽ

#define OutputStatus_Brake_OFF      HAL_GPIO_WritePin(OutputStatus_Brake_GPIO_Port,OutputStatus_Brake_Pin,GPIO_PIN_SET)         // ����ߵ�ƽ
#define OutputStatus_Brake_ON       HAL_GPIO_WritePin(OutputStatus_Brake_GPIO_Port,OutputStatus_Brake_Pin,GPIO_PIN_RESET)       // ����͵�ƽ

#define OutputStatus_Forward_OFF    HAL_GPIO_WritePin(OutputStatus_Forward_GPIO_Port,OutputStatus_Forward_Pin,GPIO_PIN_SET)     // ����ߵ�ƽ
#define OutputStatus_Forward_ON     HAL_GPIO_WritePin(OutputStatus_Forward_GPIO_Port,OutputStatus_Forward_Pin,GPIO_PIN_RESET)   // ����͵�ƽ

#define OutputStatus_Backward_OFF   HAL_GPIO_WritePin(OutputStatus_Backward_GPIO_Port,OutputStatus_Backward_Pin,GPIO_PIN_SET)   // ����ߵ�ƽ
#define OutputStatus_Backward_ON    HAL_GPIO_WritePin(OutputStatus_Backward_GPIO_Port,OutputStatus_Backward_Pin,GPIO_PIN_RESET) // ����͵�ƽ

#define OutputStatus_Led_OFF        HAL_GPIO_WritePin(OutputStatus_Led_GPIO_Port,OutputStatus_Led_Pin,GPIO_PIN_SET)             // ����ߵ�ƽ
#define OutputStatus_Led_ON         HAL_GPIO_WritePin(OutputStatus_Led_GPIO_Port,OutputStatus_Led_Pin,GPIO_PIN_RESET)           // ����͵�ƽ

#define OutputStatus_Camera_OFF     HAL_GPIO_WritePin(OutputStatus_Camera_GPIO_Port,OutputStatus_Camera_Pin,GPIO_PIN_SET)             // ����ߵ�ƽ
#define OutputStatus_Camera_ON      HAL_GPIO_WritePin(OutputStatus_Camera_GPIO_Port,OutputStatus_Camera_Pin,GPIO_PIN_RESET)

#define BarrierStatus               HAL_GPIO_ReadPin(BarrierStatus_GPIO_Port,BarrierStatus_Pin)                                 //�ϰ���״̬���


void barrierScan(void);
uint32_t filter(uint16_t *Array);
void ifInsideOrOutside(void);
void sendEnd(void);
void updateData(void);

#endif
