#include "control.h" 
#include "stdio.h"
#include "gpio.h"
/**
  * �� �� ��: barrierScan
  * ��������: ����״�����Ŀ����� 
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  *   By Sw Young 
  *   2018.11.27
  */
extern uint32_t carSpeed,carDistance;              //����С���ٶȡ�С�����
uint8_t sendListCounter = 0;
extern uint16_t List[160];
extern uint8_t probeStatus;                      //���״̬,Ĭ�����ϰ�
void barrierScan()
{
  if(BarrierStatus)
  {
    HAL_Delay(20);                          //��ʱ����
    if(BarrierStatus)
    {
      probeStatus = 1;                      //���״̬,���ϰ�
      OutputStatus_Led_ON;                  //�򿪱�����
      OutputStatus_Camera_ON;               //��������
      printf("��⵽�ϰ���");               //���ڷ�����Ϣ
      if(sendListCounter<160)
        List[sendListCounter]=carDistance;
      sendListCounter++;
    }while(BarrierStatus);
  }
  else
  {
    probeStatus = 0;                        //���״̬,���ϰ�
    OutputStatus_Led_OFF;                   //�رձ�����
    OutputStatus_Camera_ON;                 //�رմ�������
  }
    
}
/**
  * �� �� ��: filter
  * ��������: ��ֵ�˲� 
  * �������: uint16_t *Array
  * �� �� ֵ: �˲����ֵ
  * ˵    ��: ��
  *   By Sw Young 
  *   2018.11.27
  */
uint8_t j;
uint32_t temp;
uint32_t filter(uint16_t *Array)
{
  uint32_t temp = 0;
  for(j=0;j<5;j++)
  {
    temp+=Array[j];
  }  
  return temp/5;
}
/**
  * �� �� ��: ifInsideOrOutside
  * ��������: �ж��Ƿ�������� 
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  *   By Sw Young 
  *   2018.11.27
  */

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
/**
  * �� �� ��: sendEnd
  * ��������: ���ͽ����� 
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  *   By Sw Young 
  *   2018.11.27
  */
void sendEnd(void)
{
  printf("%c",0xFF);printf("%c",0xFF);printf("%c",0xFF);
}

/**
  * �� �� ��: updateData
  * ��������: �������ݺ��� 
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  *   By Sw Young 
  *   2018.11.27
  */
extern uint32_t Time_Sec;   //����ʱ����ر���
extern uint16_t Height;
extern uint8_t carStatus;
extern uint8_t sendListFlag,sendListPage;
void updateData(void)
{
  sendEnd();
  printf("t17.txt=\"%d\"",Time_Sec);
  sendEnd();
  printf("t14.txt=\"%d\"",Height);
  sendEnd();
  printf("t7.txt=\"%d\"",carSpeed);
  sendEnd();
  printf("t9.txt=\"%d\"",carDistance);
  sendEnd();
  if(carStatus==0)
  {
    sendEnd();
    printf("t2.txt=\"ֹͣ\"");
    sendEnd();
    
  }if(carStatus==1)
  {
    sendEnd();
    printf("t2.txt=\"ǰ��\"");
    sendEnd();
    OutputStatus_Brake_OFF;
    OutputStatus_Forward_ON;
    OutputStatus_Backward_OFF;
  }else if(carStatus==2)
  {
    sendEnd();
    printf("t2.txt=\"����\"");
    sendEnd();
    OutputStatus_Brake_OFF;
    OutputStatus_Forward_OFF;
    OutputStatus_Backward_ON;
  }else if(carStatus==3)
  {
    sendEnd();
    printf("t2.txt=\"ɲ��\"");
    sendEnd();
    OutputStatus_Brake_ON;
    OutputStatus_Forward_OFF;
    OutputStatus_Backward_OFF;
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
  if(sendListFlag==1)
  {
    sendListFlag=0;
    sendEnd();
    printf("t200.txt=\"%d\"",List[sendListPage*16+0]); sendEnd();
    printf("t201.txt=\"%d\"",List[sendListPage*16+1]); sendEnd();
    printf("t202.txt=\"%d\"",List[sendListPage*16+2]); sendEnd();
    printf("t203.txt=\"%d\"",List[sendListPage*16+3]); sendEnd();
    printf("t204.txt=\"%d\"",List[sendListPage*16+4]); sendEnd();
    printf("t205.txt=\"%d\"",List[sendListPage*16+5]); sendEnd();
    printf("t206.txt=\"%d\"",List[sendListPage*16+6]); sendEnd();
    printf("t207.txt=\"%d\"",List[sendListPage*16+7]); sendEnd();
    printf("t208.txt=\"%d\"",List[sendListPage*16+8]); sendEnd();
    printf("t209.txt=\"%d\"",List[sendListPage*16+9]); sendEnd();
    printf("t210.txt=\"%d\"",List[sendListPage*16+10]);sendEnd();
    printf("t211.txt=\"%d\"",List[sendListPage*16+11]);sendEnd();
    printf("t212.txt=\"%d\"",List[sendListPage*16+12]);sendEnd();
    printf("t213.txt=\"%d\"",List[sendListPage*16+13]);sendEnd();
    printf("t214.txt=\"%d\"",List[sendListPage*16+14]);sendEnd();
    printf("t215.txt=\"%d\"",List[sendListPage*16+15]);sendEnd();
  }
}
