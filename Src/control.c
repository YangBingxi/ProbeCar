#include "control.h" 
#include "stdio.h"
#include "gpio.h"
/**
  * 函 数 名: barrierScan
  * 函数功能: 检测雷达输出的开关量 
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  *   By Sw Young 
  *   2018.11.27
  */
extern float carSpeed,carDistance;          //定义小车速度、小车里程
uint32_t sendListCounter = 0;
extern uint16_t List[160];                  //障碍物位置存储列表
extern uint8_t probeStatus;                 //检测状态,默认无障碍
extern uint8_t SD_SendData[20];             //写入SD卡文件列表
void barrierScan()
{
  if(BarrierStatus)
  {
    HAL_Delay(20);                          //延时消抖
    if(BarrierStatus)
    {
      probeStatus = 1;                      //检测状态,有障碍
      OutputStatus_Led_ON;                  //打开报警灯
      OutputStatus_Camera_ON;               //触发拍照
      //printf("检测到障碍物");             //串口发送信息
      sendEnd();
      printf("t5.pco=63488");
      sendEnd();
      printf("t5.txt=\"有障碍\"");
      sendEnd();
      if(sendListCounter<1600000)
        List[sendListCounter]=carDistance;   //存储障碍物位置
      writeToSD(sendListCounter,(int)carDistance,SD_SendData); //将障碍物的位置写入sd卡
      sendListCounter++;
    }while(BarrierStatus);
  }
  else
  {
    sendEnd();
    printf("t5.pco=2016");
    sendEnd();
    printf("t5.txt=\"无障碍\"");
    sendEnd(); 
    probeStatus = 0;                        //检测状态,无障碍
    OutputStatus_Led_OFF;                   //关闭报警灯
    OutputStatus_Camera_OFF;                //关闭触发拍照
  }
    
}
/**
  * 函 数 名: filter
  * 函数功能: 均值滤波 
  * 输入参数: uint16_t *Array
  * 返 回 值: 滤波后的值
  * 说    明: 无
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
  * 函 数 名: ifInsideOrOutside
  * 函数功能: 判断是否在隧道内 
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
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
    InsideOrOutsideStatus_ON;   //在隧道外
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
    InsideOrOutsideStatus_OFF;  //在隧道内
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
/**
  * 函 数 名: sendEnd
  * 函数功能: 发送结束符 
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  *   By Sw Young 
  *   2018.11.27
  */
void sendEnd(void)
{
  printf("%c",0xFF);printf("%c",0xFF);printf("%c",0xFF);
}

/**
  * 函 数 名: updateData
  * 函数功能: 更新串口屏数据函数 
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  *   By Sw Young 
  *   2018.11.27
  */
extern uint32_t Time_Sec;   //定义时间相关变量
extern uint16_t Height;
extern uint8_t carStatus;
extern uint8_t sendListFlag,sendListPage;
void updateData(void)
{
  sendEnd();
  printf("t17.txt=\"%d\"",Time_Sec);         //更新运行时间
  sendEnd();
  printf("t14.txt=\"%d\"",Height);           //更新监测高度
  sendEnd();
  printf("t7.txt=\"%.2f\"",carSpeed);        //更新小车速度
  sendEnd();
  printf("t9.txt=\"%.2f\"",carDistance);     //更新小车行驶里程
  sendEnd();
  if(carStatus==0)                           //更新小车运行状态
  {
    sendEnd();
    printf("t2.txt=\"停止\"");
    sendEnd();
    
  }if(carStatus==1)
  {
    sendEnd();
    printf("t2.txt=\"前进\"");
    sendEnd();
    OutputStatus_Brake_OFF;
    OutputStatus_Forward_ON;
    OutputStatus_Backward_OFF;
  }else if(carStatus==2)
  {
    sendEnd();
    printf("t2.txt=\"后退\"");
    sendEnd();
    OutputStatus_Brake_OFF;
    OutputStatus_Forward_OFF;
    OutputStatus_Backward_ON;
  }else if(carStatus==3)
  {
    sendEnd();
    printf("t2.txt=\"刹车\"");
    sendEnd();
    OutputStatus_Brake_ON;
    OutputStatus_Forward_OFF;
    OutputStatus_Backward_OFF;
  }

  if(sendListFlag==1)                        //更新障碍物列表
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
