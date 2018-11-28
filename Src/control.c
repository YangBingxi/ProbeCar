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

uint8_t j;
uint32_t temp;
uint32_t filter(uint16_t *Array)
{
  uint32_t temp = 0;
  for(j=0;j<12;j++)
  {
    temp+=Array[j];
  }  
  return temp/12;
}
uint16_t HeightArray[12];
void ifInsideOrOutside(void)
{
  uint16_t Heigth_temp;
  Heigth_temp = filter(HeightArray);
  printf("\r\nHeigth_temp=%d\n",Heigth_temp);
  if(Heigth_temp>HeigthStd)
  {
    InsideOrOutsideStatus_ON;   //判断在隧道外
    printf("\r\n隧道外");
  }else
  {
    InsideOrOutsideStatus_OFF;  //判断在隧道内
    printf("\r\n隧道内");
  }
  
}
