#include "control.h" 
#include "stdio.h"
#include "gpio.h"

void barrierScan()
{
  if(BarrierStatus)
  {
    HAL_Delay(20);               //��ʱ����
    if(BarrierStatus)
    {
      printf("��⵽�ϰ���");//�ϰ����ⴥ��
    }
  }
}
