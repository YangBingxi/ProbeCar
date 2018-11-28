#include "control.h" 
#include "stdio.h"
#include "gpio.h"

extern uint8_t probeStatus;                      //���״̬,Ĭ�����ϰ�

void barrierScan()
{
  if(BarrierStatus)
  {
    HAL_Delay(20);               //��ʱ����
    if(BarrierStatus)
    {
      probeStatus = 1;                      //���״̬,���ϰ�
      printf("��⵽�ϰ���");//�ϰ����ⴥ��
    }
  }
  else
  probeStatus = 0;                      //���״̬,���ϰ�
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
    InsideOrOutsideStatus_ON;   //�ж��������
    printf("\r\n�����");
    if(page!=1)
    {
      page = 1;
      sendEnd();
      printf("page %d",page);
      sendEnd();    
    }

  }else
  {
    InsideOrOutsideStatus_OFF;  //�ж��������
    printf("\r\n�����");
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

extern uint32_t Time_Sec;   //����ʱ����ر���
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
    printf("t2.txt=\"ֹͣ\"");
    sendEnd();
    
  }else if(carStatus==1)
  {
    sendEnd();
    printf("t2.txt=\"ǰ��\"");
    sendEnd();
  }else if(carStatus==2)
  {
    sendEnd();
    printf("t2.txt=\"����\"");
    sendEnd();
  }else if(carStatus==3)
  {
    sendEnd();
    printf("t2.txt=\"ɲ��\"");
    sendEnd();
  }
  if(probeStatus==1)
  {
    sendEnd();
    printf("t5.pco=63488");
    sendEnd();
    printf("t5.txt=\"���ϰ�\"");
    sendEnd();
  } 
  else
  {
    sendEnd();
    printf("t5.pco=2016");
    sendEnd();
    printf("t5.txt=\"���ϰ�\"");
    sendEnd();  
  }
}
