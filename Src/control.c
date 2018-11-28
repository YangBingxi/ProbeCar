#include "control.h" 
#include "stdio.h"
#include "gpio.h"

void barrierScan()
{
  if(BarrierStatus)
  {
    HAL_Delay(20);               //延时消抖
    if(BarrierStatus)
    {
      printf("检测到障碍物");//障碍物检测触发
    }
  }
}
