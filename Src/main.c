
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  *
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "dma.h"
#include "fatfs.h"
#include "sdio.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "led.h"
#include "stdio.h"
#include "control.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint8_t aRxBuffer_1;			                  //接收中断缓冲
uint8_t Uart1_RxBuff[256];		              //接收缓冲
uint8_t Uart1_Rx_Cnt = 0;		                //接收缓冲计数
uint8_t aRxBuffer_2;			                  //接收中断缓冲
uint8_t Uart2_RxBuff[256];		              //接收缓冲
uint8_t Uart2_Rx_Cnt = 0;		                //接收缓冲计数
uint8_t aRxBuffer_3;			                  //接收中断缓冲
uint8_t Uart3_RxBuff[256];		              //接收缓冲
uint8_t Uart3_Rx_Cnt = 0;		                //接收缓冲计数

STRUCT_CAPTURE strCapture = { 0, 0, 0 };


uint32_t TimeCounter = 0,Time_Sec = 0;      //定义时间相关变量
uint16_t Height = 0;   
float carSpeed,carSpeedLast,carDistance;    //定义小车速度、小车里程
uint8_t carStatus = 0;                      //小车状态,默认停止
uint8_t probeStatus = 0;                    //检测状态,默认无障碍
uint8_t sendListFlag = 0;
uint8_t sendListPage = 0;
uint16_t List[160];

uint8_t timeFlag = 0;


typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;
/* 私有宏定义 ----------------------------------------------------------------*/
#define BLOCK_SIZE            512         // SD卡块大小     
#define NUMBER_OF_BLOCKS      8           // 测试块数量(小于15)
#define WRITE_READ_ADDRESS    0x00002000  // 测试读写地址
 
/* 私有变量 ------------------------------------------------------------------*/
__align(4) uint32_t Buffer_Block_Tx[BLOCK_SIZE*NUMBER_OF_BLOCKS]; // 写数据缓存
__align(4) uint32_t Buffer_Block_Rx[BLOCK_SIZE*NUMBER_OF_BLOCKS]; // 读数据缓存
HAL_StatusTypeDef sd_status;    // HAL库函数操作SD卡函数返回值：操作结果
TestStatus test_status;           // 数据测试结果

FATFS fs;                 // Work area (file system object) for logical drive
FIL fil;                  // file objects
uint32_t byteswritten;                /* File write counts */
uint32_t bytesread;                   /* File read counts */
uint8_t wtext[] = "障碍物检测位置列表"; /* File write buffer */
uint8_t SD_SendData[20] = {'\r','\n',' ',' ',' ',' ',' ',' ',':',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' '};
/*---------------------------0----1---2---3---4---5---6---7---8---9--10--11--12--13--14--15--16--17--18--19--*/
uint8_t rtext[100];                     /* File read buffers */
char filename[] = "障碍物检测位置列表.txt";
extern DMA_HandleTypeDef hdma_sdio;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void SD_EraseTest(void);
void SD_Write_Read_Test(void);
void Fatfs_RW_test(void);
void writeToSD(uint32_t Counter,uint32_t Distance,uint8_t Array[]);
HAL_StatusTypeDef SD_DMAConfigRx(SD_HandleTypeDef *hsd);
HAL_StatusTypeDef SD_DMAConfigTx(SD_HandleTypeDef *hsd);


/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  uint32_t ulTmrClk, ulTime,t;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM5_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  /* USER CODE BEGIN 2 */
	//HAL_UART_Receive_IT(&huart1, (uint8_t *)&aRxBuffer_1, 1);   //开启串口接收中断
	HAL_UART_Receive_IT(&huart2, (uint8_t *)&aRxBuffer_2, 1);     //开启串口接收中断
	HAL_UART_Receive_IT(&huart3, (uint8_t *)&aRxBuffer_3, 1);     //开启串口接收中断



  HAL_TIM_Base_Start_IT(&htim6);                                //使能TIM6定时器中断
  
  /* 获取定时器时钟周期 */	
	ulTmrClk = HAL_RCC_GetHCLKFreq()/GENERAL_TIM_PRESCALER;    
  /* 启动定时器 */
  HAL_TIM_Base_Start_IT(&htim5);  
  /* 启动定时器通道输入捕获并开启中断 */
  HAL_TIM_IC_Start_IT(&htim5,GENERAL_TIM_CHANNELx);
  
  printf("System is Ok!\n");
  sendEnd();
  printf("page 0");                                             //初始化串口屏
  sendEnd();
  
//  SD_EraseTest();
//  SD_Write_Read_Test();  3
    Fatfs_RW_test();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    if(strCapture.ucFinishFlag == 1 )
		{
      /* 计算高电平计数值 */
			ulTime = strCapture .usPeriod * 0xFFFF + strCapture .usCtr;
			/* 打印高电平脉宽时间 */
			printf ( ">>测得高电平脉宽时间：%d.%d s\n", ulTime / ulTmrClk, ulTime % ulTmrClk ); 
			carSpeed = UnitWheelLength/(ulTime / ulTmrClk+ulTime % ulTmrClk/1000000.0);
      carSpeed/=100.0;                                          //将速度转换为m/s 
      if(carSpeed-carSpeedLast>2||carSpeedLast-carSpeed>2)      //限幅滤波
          carSpeed = carSpeedLast; 
      carSpeedLast = carSpeed;  
      printf ( ">>测得小车速度：%f\n", carSpeed);
      strCapture .ucFinishFlag = 0;			
		}
    barrierScan();                                              //障碍物状态检测
    updateData();
    LED_TOGGLE;                                                 //led翻转
    HAL_Delay(200);                                             //系统延时
    
    
    t++;
    writeToSD(t,t,SD_SendData);
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */
/**
  * 函数功能: 定时器输入捕获中断回调函数
  * 输入参数: htim：定时器句柄
  * 返 回 值: 无
  * 说    明: 无
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  TIM_IC_InitTypeDef sConfigIC;
  
  if ( strCapture .ucStartFlag == 0 )
  {
    __HAL_TIM_SET_COUNTER(htim,0); // 清零定时器计数
    strCapture .usPeriod = 0;			
    strCapture .usCtr = 0;
    
    // 配置输入捕获参数，主要是修改触发电平
    sConfigIC.ICPolarity = GENERAL_TIM_STRAT_ICPolarity;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0;
    HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, GENERAL_TIM_CHANNELx);
    // 清除中断标志位
    __HAL_TIM_CLEAR_IT(htim, GENERAL_TIM_IT_CCx);
    // 启动输入捕获并开启中断
    HAL_TIM_IC_Start_IT(&htim5,GENERAL_TIM_CHANNELx);    
    strCapture .ucStartFlag = 1;			
  }		
  
  else
  {
    // 获取定时器计数值
    strCapture .usCtr = HAL_TIM_ReadCapturedValue(&htim5,GENERAL_TIM_CHANNELx);
    // 配置输入捕获参数，主要是修改触发电平
    sConfigIC.ICPolarity = GENERAL_TIM_STRAT_ICPolarity;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0;
    HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, GENERAL_TIM_CHANNELx);
    
    // 清除中断标志位
    __HAL_TIM_CLEAR_IT(htim, GENERAL_TIM_IT_CCx); 
    // 启动输入捕获并开启中断
    HAL_TIM_IC_Start_IT(&htim5,GENERAL_TIM_CHANNELx);    
    strCapture .ucStartFlag = 0;			
    strCapture .ucFinishFlag = 1;    
  }
}
 //定时器TIM6溢出中断处理回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(TIM5 == htim->Instance)
    {
      strCapture .usPeriod ++;
    }
    
    
    if(TIM6 == htim->Instance)
    {
      if(timeFlag>0)
      {
        TimeCounter++;                                          //计时变量++
                
        if(TimeCounter==1000)
        {
            carDistance+=carSpeed;
            Time_Sec++;                                         //越界清零
            if(Time_Sec>UINT32_MAX-5)
              Time_Sec=0;
            TimeCounter = 0;
            printf("\r\nTime=%d\n",Time_Sec);          
          }
        }
     } 
} 
/**
  * 函数功能: 检查缓冲区的数据是否为0xff或0
  * 输入参数: pBuffer：要比较的缓冲区的指针
  *           BufferLength：缓冲区长度
  * 返 回 值: PASSED：缓冲区的数据全为0xff或0
  *           FAILED：缓冲区的数据至少有一个不为0xff或0 
  * 说    明: 无
  */
TestStatus eBuffercmp(uint32_t* pBuffer, uint32_t BufferLength)
{
  while (BufferLength--)
  {
    /* SD卡擦除后的可能值为0xff或0 */
    if ((*pBuffer != 0xFFFFFFFF) && (*pBuffer != 0))
    {
      return FAILED;
    }
    pBuffer++;
  }
  return PASSED;
}
 
/**
  * 函数功能: SD卡擦除测试
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void SD_EraseTest(void)
{
	/* 第1个参数为SD卡句柄，第2个参数为擦除起始地址，第3个参数为擦除结束地址 */
  sd_status=HAL_SD_Erase(&hsd,WRITE_READ_ADDRESS,WRITE_READ_ADDRESS+NUMBER_OF_BLOCKS*4);
   printf("erase status:%d\r\n",sd_status);
 
	HAL_Delay(500);
  if (sd_status == HAL_OK)
  {	
    /* 读取刚刚擦除的区域 */
    sd_status = HAL_SD_ReadBlocks_DMA(&hsd,(uint8_t *)Buffer_Block_Rx,WRITE_READ_ADDRESS,NUMBER_OF_BLOCKS);
    printf("erase read status:%d\r\n",sd_status);
    /* 把擦除区域读出来对比 */
    test_status = eBuffercmp(Buffer_Block_Rx,BLOCK_SIZE*NUMBER_OF_BLOCKS);
 
    if(test_status == PASSED)
      printf("》擦除测试成功！\r\n" ); 
    else	  
      printf("》擦除不成功，数据出错！\r\n" );      
  }
  else
  {
    printf("》擦除测试失败！部分SD不支持擦除，只要读写测试通过即可\r\n" );
  }
}
 
/**
  * 函数功能: 在缓冲区中填写数据
  * 输入参数: pBuffer：要填充的缓冲区
  *           BufferLength：要填充的大小
  *           Offset：填在缓冲区的第一个值 
  * 返 回 值: 无
  * 说    明: 无
  */
void Fill_Buffer(uint32_t *pBuffer, uint32_t BufferLength, uint32_t Offset)
{
  uint32_t index = 0;
  /* 填充数据 */
  for (index = 0; index < BufferLength; index++ )
  {
    pBuffer[index] = index + Offset;
  }
}
 
/**
  * 函数功能: 比较两个缓冲区中的数据是否相等
  * 输入参数: pBuffer1：要比较的缓冲区1的指针
  *           pBuffer2：要比较的缓冲区2的指针
  *           BufferLength：缓冲区长度
  * 返 回 值: PASSED：相等
  *           FAILED：不等
  * 说    明: 无
  */
TestStatus Buffercmp(uint32_t* pBuffer1, uint32_t* pBuffer2, uint32_t BufferLength)
{
  while (BufferLength--)
  {
    if(BufferLength%50==0)
    {
      printf("buf:0x%08X - 0x%08X\r\n",*pBuffer1,*pBuffer2);
    }
    if (*pBuffer1 != *pBuffer2)
    {
      return FAILED;
    }
    pBuffer1++;
    pBuffer2++;
  }
  return PASSED;
}
 
/**
  * 函数功能: SD卡读写测试
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void SD_Write_Read_Test(void)
{  
	int i,j = 0;
  /* 填充数据到写缓存 */
  Fill_Buffer(Buffer_Block_Tx,BLOCK_SIZE*NUMBER_OF_BLOCKS, 0x6666);
  
  /* 往SD卡写入数据 */
  sd_status = HAL_SD_WriteBlocks_DMA(&hsd,(uint8_t *)Buffer_Block_Tx,WRITE_READ_ADDRESS,NUMBER_OF_BLOCKS);
  printf("write status:%d\r\n",sd_status);
			
  HAL_Delay(500);
  /* 从SD卡读取数据 */
  sd_status = HAL_SD_ReadBlocks_DMA(&hsd,(uint8_t *)Buffer_Block_Rx,WRITE_READ_ADDRESS,NUMBER_OF_BLOCKS);
  printf("read status:%d\r\n",sd_status);
  
  /* 比较数据 */
  test_status = Buffercmp(Buffer_Block_Tx, Buffer_Block_Rx, BLOCK_SIZE*NUMBER_OF_BLOCKS/4);	//比较
  if(test_status == PASSED)
	{
    printf("》读写测试成功！\r\n" );
		
		for(i=0;i<BLOCK_SIZE*NUMBER_OF_BLOCKS/4;i++)
		{
			if(j==8)
			{
				printf("\r\n");
				j=0;
			}
			
			printf("%08x   ",Buffer_Block_Rx[i]);
			j++;
		}
		printf("\r\n");
	}
  else  
  	printf("》读写测试失败！\r\n " );  
}
/*
*Fatfs文件系统读写测试
*/
void Fatfs_RW_test(void)
{
	  printf("\r\n ****** FatFs Example ******\r\n\r\n");
 
    /*##-1- Register the file system object to the FatFs module ##############*/
    retSD = f_mount(&fs, "", 1);
    if(retSD)
    {
        printf(" mount error : %d \r\n",retSD);
        Error_Handler();
    }
    else
        printf(" mount sucess!!! \r\n");
     
    /*##-2- Create and Open new text file objects with write access ######*/
    retSD = f_open(&fil, filename, FA_CREATE_ALWAYS | FA_WRITE);
    if(retSD)
        printf(" open file error : %d\r\n",retSD);
    else
        printf(" open file sucess!!! \r\n");
     
    /*##-3- Write data to the text files ###############################*/
    retSD = f_write(&fil, wtext, sizeof(wtext), (void *)&byteswritten);
    if(retSD)
        printf(" write file error : %d\r\n",retSD);
    else
    {
        printf(" write file sucess!!! \r\n");
        printf(" write Data : %s\r\n",wtext);
    }
     
    /*##-4- Close the open text files ################################*/
    retSD = f_close(&fil);
    if(retSD)
        printf(" close error : %d\r\n",retSD);
    else
        printf(" close sucess!!! \r\n");
     
//    /*##-5- Open the text files object with read access ##############*/
//    retSD = f_open(&fil, filename, FA_READ);
//    if(retSD)
//        printf(" open file error : %d\r\n",retSD);
//    else
//        printf(" open file sucess!!! \r\n");
//     
//    /*##-6- Read data from the text files ##########################*/
//    retSD = f_read(&fil, rtext, sizeof(rtext), (UINT*)&bytesread);
//    if(retSD)
//        printf(" read error!!! %d\r\n",retSD);
//    else
//    {
//        printf(" read sucess!!! \r\n");
//        printf(" read Data : %s\r\n",rtext);
//    }
//     
//    /*##-7- Close the open text files ############################*/
//    retSD = f_close(&fil);
//    if(retSD)  
//        printf(" close error!!! %d\r\n",retSD);
//    else
//        printf(" close sucess!!! \r\n");
//     
//    /*##-8- Compare read data with the expected data ############*/
//    if(bytesread == byteswritten)
//    { 
//        printf(" FatFs is working well!!!\r\n");
//    }
}
/**
  * @brief Configure the DMA to receive data from the SD card
  * @retval
  *  HAL_ERROR or HAL_OK
  */
HAL_StatusTypeDef SD_DMAConfigRx(SD_HandleTypeDef *hsd)
{
  HAL_StatusTypeDef status = HAL_ERROR;
  
  /* Configure DMA Rx parameters */
  hdma_sdio.Instance = DMA2_Channel4;
	hdma_sdio.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_sdio.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_sdio.Init.MemInc = DMA_MINC_ENABLE;
	hdma_sdio.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
	hdma_sdio.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
	hdma_sdio.Init.Mode = DMA_NORMAL;
  hdma_sdio.Init.Priority = DMA_PRIORITY_LOW;
 
  /* Associate the DMA handle */
  __HAL_LINKDMA(hsd,hdmarx,hdma_sdio);
 
  /* Stop any ongoing transfer and reset the state*/
  HAL_DMA_Abort(&hdma_sdio);
  
  /* Deinitialize the Channel for new transfer */
  HAL_DMA_DeInit(&hdma_sdio);//注意这里！！！DeInit的是另一个通道！！！
 
  /* Configure the DMA Channel */
  status = HAL_DMA_Init(&hdma_sdio);
    
  return (status);
}
 
/**
  * @brief Configure the DMA to transmit data to the SD card
  * @retval
  *  HAL_ERROR or HAL_OK
  */
HAL_StatusTypeDef SD_DMAConfigTx(SD_HandleTypeDef *hsd)
{
  HAL_StatusTypeDef status;
  
  /* SDMMC1_TX Init */
	hdma_sdio.Instance = DMA2_Channel4;
	hdma_sdio.Init.Direction = DMA_MEMORY_TO_PERIPH;
	hdma_sdio.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_sdio.Init.MemInc = DMA_MINC_ENABLE;
	hdma_sdio.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
	hdma_sdio.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
	hdma_sdio.Init.Mode = DMA_NORMAL;//这里NORMAL或其他都可以，无所谓
	hdma_sdio.Init.Priority = DMA_PRIORITY_LOW;
 
  /* Associate the DMA handle */
  __HAL_LINKDMA(hsd, hdmatx, hdma_sdio);
  
  /* Stop any ongoing transfer and reset the state*/
  HAL_DMA_Abort(&hdma_sdio);
  
  /* Deinitialize the Channel for new transfer */
  HAL_DMA_DeInit(&hdma_sdio);  //注意这里！！！DeInit的是另一个通道！！！
  
  /* Configure the DMA Channel */
  status = HAL_DMA_Init(&hdma_sdio); 
 
  return (status);
}

void writeToSD(uint32_t Counter,uint32_t Distance,uint8_t Array[])
{
	Array[7]=Counter%10+48;
  Array[6]=Counter/10%10+48;
	Array[5]=Counter/100%10+48;
	Array[4]=Counter/1000%10+48;
	Array[3]=Counter/10000%10+48;
	Array[2]=Counter/100000%10+48;
	
	Array[14]=Distance%10+48;
    Array[13]=Distance/10%10+48;
	Array[12]=Distance/100%10+48;
	Array[11]=Distance/1000%10+48;
	Array[10]=Distance/10000%10+48;
	Array[9]=Distance/100000%10+48;
	
    retSD = f_open(&fil, filename, FA_WRITE);
    f_lseek(&fil,f_size(&fil));
    if(retSD)
        printf(" open file error : %d\r\n",retSD);
    else
        printf(" open file sucess!!! \r\n");
     
    /*##-3- Write data to the text files ###############################*/
    retSD = f_write(&fil, Array, 20, (void *)&byteswritten);
    if(retSD)
        printf(" write file error : %d\r\n",retSD);
    else
    {
        printf(" write file sucess!!! \r\n");
        printf(" write Data : %s\r\n",Array);
    }
     
    /*##-4- Close the open text files ################################*/
    retSD = f_close(&fil);
    if(retSD)
        printf(" close error : %d\r\n",retSD);
    else
        printf(" close sucess!!! \r\n");
}
/*
*printf函数输出重定向
*/
//加入以下代码,支持printf函数，使用printf函数从串口输出。
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 
 
FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	USART1->DR=(uint8_t)ch;      
	return ch;
}
#endif 
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
