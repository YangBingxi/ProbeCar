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
extern float carSpeed,carDistance;          //����С���ٶȡ�С�����
uint32_t sendListCounter = 0;
extern uint16_t List[160];                  //�ϰ���λ�ô洢�б�
extern uint8_t probeStatus;                 //���״̬,Ĭ�����ϰ�
extern uint8_t SD_SendData[20];             //д��SD���ļ��б�
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
      //printf("��⵽�ϰ���");             //���ڷ�����Ϣ
      sendEnd();
      printf("t5.pco=63488");
      sendEnd();
      printf("t5.txt=\"���ϰ�\"");
      sendEnd();
      if(sendListCounter<1600000)
        List[sendListCounter]=carDistance;   //�洢�ϰ���λ��
      writeToSD(sendListCounter,(int)carDistance,SD_SendData); //���ϰ����λ��д��sd��
      sendListCounter++;
    }while(BarrierStatus);
  }
  else
  {
    sendEnd();
    printf("t5.pco=2016");
    sendEnd();
    printf("t5.txt=\"���ϰ�\"");
    sendEnd(); 
    probeStatus = 0;                        //���״̬,���ϰ�
    OutputStatus_Led_OFF;                   //�رձ�����
    OutputStatus_Camera_OFF;                //�رմ�������
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
    InsideOrOutsideStatus_ON;   //�������
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
    InsideOrOutsideStatus_OFF;  //�������
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
  * ��������: ���´��������ݺ��� 
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
  printf("t17.txt=\"%d\"",Time_Sec);         //��������ʱ��
  sendEnd();
  printf("t14.txt=\"%d\"",Height);           //���¼��߶�
  sendEnd();
  printf("t7.txt=\"%.2f\"",carSpeed);        //����С���ٶ�
  sendEnd();
  printf("t9.txt=\"%.2f\"",carDistance);     //����С����ʻ���
  sendEnd();
  if(carStatus==0)                           //����С������״̬
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

  if(sendListFlag==1)                        //�����ϰ����б�
  {
    sendListFlag=0;
    sendEnd();
    printf("t300.txt=\"%d/1000\"",sendListPage); sendEnd();
    
    printf("t200.txt=\"1: %d\"",List[sendListPage*16+0]); sendEnd();
    printf("t201.txt=\"2: %d\"",List[sendListPage*16+1]); sendEnd();
    printf("t202.txt=\"3: %d\"",List[sendListPage*16+2]); sendEnd();
    printf("t203.txt=\"4: %d\"",List[sendListPage*16+3]); sendEnd();
    printf("t204.txt=\"5: %d\"",List[sendListPage*16+4]); sendEnd();
    printf("t205.txt=\"6: %d\"",List[sendListPage*16+5]); sendEnd();
    printf("t206.txt=\"7: %d\"",List[sendListPage*16+6]); sendEnd();
    printf("t207.txt=\"8: %d\"",List[sendListPage*16+7]); sendEnd();
    printf("t208.txt=\"9: %d\"",List[sendListPage*16+8]); sendEnd();
    printf("t209.txt=\"10: %d\"",List[sendListPage*16+9]); sendEnd();
    printf("t210.txt=\"11: %d\"",List[sendListPage*16+10]);sendEnd();
    printf("t211.txt=\"12: %d\"",List[sendListPage*16+11]);sendEnd();
    printf("t212.txt=\"13: %d\"",List[sendListPage*16+12]);sendEnd();
    printf("t213.txt=\"14: %d\"",List[sendListPage*16+13]);sendEnd();
    printf("t214.txt=\"15: %d\"",List[sendListPage*16+14]);sendEnd();
    printf("t215.txt=\"16: %d\"",List[sendListPage*16+15]);sendEnd();
  }
}
