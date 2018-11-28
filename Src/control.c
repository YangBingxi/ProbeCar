#include "control.h" 
#include "stdio.h"
#include "gpio.h"

extern uint8_t probeStatus;                      //检测状态,默认无障碍

void barrierScan()
{
  if(BarrierStatus)
  {
    HAL_Delay(20);               //延时消抖
    if(BarrierStatus)
    {
      probeStatus = 1;                      //检测状态,有障碍
      printf("检测到障碍物");//障碍物检测触发
    }
  }
  else
  probeStatus = 0;                      //检测状态,无障碍
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
uint8_t page = 0;
void ifInsideOrOutside(void)
{
  uint16_t Heigth_temp;
  Heigth_temp = filter(HeightArray);
  //printf("\r\nHeigth_temp=%d\n",Heigth_temp);
  if(Heigth_temp>HeigthStd)
  {
    InsideOrOutsideStatus_ON;   //判断在隧道外
    printf("\r\n隧道外");
    if(page!=1)
    {
      page = 1;
      sendEnd();
      printf("page %d",page);
      sendEnd();    
    }

  }else
  {
    InsideOrOutsideStatus_OFF;  //判断在隧道内
    printf("\r\n隧道内");
    if(page!=2)
    {
      page = 2;
      sendEnd();
      printf("page %d",page);
      sendEnd();    
    }
  }
  
}
void sendEnd(void)
{
  printf("%c",0xFF);printf("%c",0xFF);printf("%c",0xFF);
}

extern uint32_t Time_Sec;   //定义时间相关变量
extern uint16_t Height;
extern uint8_t carStatus;
void updateData(void)
{
  sendEnd();
  printf("t17.txt=\"%d\"",Time_Sec);
  sendEnd();
  printf("t14.txt=\"%d\"",Height);
  sendEnd();
  if(carStatus==0)
  {
    sendEnd();
    printf("t2.txt=\"停止\"");
    sendEnd();
    
  }else if(carStatus==1)
  {
    sendEnd();
    printf("t2.txt=\"前进\"");
    sendEnd();
  }else if(carStatus==2)
  {
    sendEnd();
    printf("t2.txt=\"后退\"");
    sendEnd();
  }else if(carStatus==3)
  {
    sendEnd();
    printf("t2.txt=\"刹车\"");
    sendEnd();
  }
  if(probeStatus==1)
  {
    sendEnd();
    printf("t5.pco=63488");
    sendEnd();
    printf("t5.txt=\"有障碍\"");
    sendEnd();
  } 
  else
  {
    sendEnd();
    printf("t5.pco=2016");
    sendEnd();
    printf("t5.txt=\"无障碍\"");
    sendEnd();  
  }
}
