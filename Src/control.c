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
    InsideOrOutsideStatus_ON;   //�ж��������
    printf("\r\n�����");
  }else
  {
    InsideOrOutsideStatus_OFF;  //�ж��������
    printf("\r\n�����");
  }
  
}
