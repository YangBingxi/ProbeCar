
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
uint8_t aRxBuffer_1;			                  //�����жϻ���
uint8_t Uart1_RxBuff[256];		              //���ջ���
uint8_t Uart1_Rx_Cnt = 0;		                //���ջ������
uint8_t aRxBuffer_2;			                  //�����жϻ���
uint8_t Uart2_RxBuff[256];		              //���ջ���
uint8_t Uart2_Rx_Cnt = 0;		                //���ջ������
uint8_t aRxBuffer_3;			                  //�����жϻ���
uint8_t Uart3_RxBuff[256];		              //���ջ���
uint8_t Uart3_Rx_Cnt = 0;		                //���ջ������

STRUCT_CAPTURE strCapture = { 0, 0, 0 };


uint32_t TimeCounter = 0,Time_Sec = 0;      //����ʱ����ر���
uint16_t Height = 0;   
float carSpeed,carSpeedLast,carDistance;    //����С���ٶȡ�С�����
uint8_t carStatus = 0;                      //С��״̬,Ĭ��ֹͣ
uint8_t probeStatus = 0;                    //���״̬,Ĭ�����ϰ�
uint8_t sendListFlag = 0;
uint8_t sendListPage = 0;
uint16_t List[160];

uint8_t timeFlag = 0;


typedef enum {FAILED = 0, PASSED = !FAILED} TestStatus;
/* ˽�к궨�� ----------------------------------------------------------------*/
#define BLOCK_SIZE            512         // SD�����С     
#define NUMBER_OF_BLOCKS      8           // ���Կ�����(С��15)
#define WRITE_READ_ADDRESS    0x00002000  // ���Զ�д��ַ
 
/* ˽�б��� ------------------------------------------------------------------*/
__align(4) uint32_t Buffer_Block_Tx[BLOCK_SIZE*NUMBER_OF_BLOCKS]; // д���ݻ���
__align(4) uint32_t Buffer_Block_Rx[BLOCK_SIZE*NUMBER_OF_BLOCKS]; // �����ݻ���
HAL_StatusTypeDef sd_status;    // HAL�⺯������SD����������ֵ���������
TestStatus test_status;           // ���ݲ��Խ��

FATFS fs;                 // Work area (file system object) for logical drive
FIL fil;                  // file objects
uint32_t byteswritten;                /* File write counts */
uint32_t bytesread;                   /* File read counts */
uint8_t wtext[] = "�ϰ�����λ���б�"; /* File write buffer */
uint8_t SD_SendData[20] = {'\r','\n',' ',' ',' ',' ',' ',' ',':',' ',' ',' ',' ',' ',' ',' ',' ',' ',' ',' '};
/*---------------------------0----1---2---3---4---5---6---7---8---9--10--11--12--13--14--15--16--17--18--19--*/
uint8_t rtext[100];                     /* File read buffers */
char filename[] = "�ϰ�����λ���б�.txt";
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
	//HAL_UART_Receive_IT(&huart1, (uint8_t *)&aRxBuffer_1, 1);   //�������ڽ����ж�
	HAL_UART_Receive_IT(&huart2, (uint8_t *)&aRxBuffer_2, 1);     //�������ڽ����ж�
	HAL_UART_Receive_IT(&huart3, (uint8_t *)&aRxBuffer_3, 1);     //�������ڽ����ж�



  HAL_TIM_Base_Start_IT(&htim6);                                //ʹ��TIM6��ʱ���ж�
  
  /* ��ȡ��ʱ��ʱ������ */	
	ulTmrClk = HAL_RCC_GetHCLKFreq()/GENERAL_TIM_PRESCALER;    
  /* ������ʱ�� */
  HAL_TIM_Base_Start_IT(&htim5);  
  /* ������ʱ��ͨ�����벶�񲢿����ж� */
  HAL_TIM_IC_Start_IT(&htim5,GENERAL_TIM_CHANNELx);
  
  printf("System is Ok!\n");
  sendEnd();
  printf("page 0");                                             //��ʼ��������
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
      /* ����ߵ�ƽ����ֵ */
			ulTime = strCapture .usPeriod * 0xFFFF + strCapture .usCtr;
			/* ��ӡ�ߵ�ƽ����ʱ�� */
			printf ( ">>��øߵ�ƽ����ʱ�䣺%d.%d s\n", ulTime / ulTmrClk, ulTime % ulTmrClk ); 
			carSpeed = UnitWheelLength/(ulTime / ulTmrClk+ulTime % ulTmrClk/1000000.0);
      carSpeed/=100.0;                                          //���ٶ�ת��Ϊm/s 
      if(carSpeed-carSpeedLast>2||carSpeedLast-carSpeed>2)      //�޷��˲�
          carSpeed = carSpeedLast; 
      carSpeedLast = carSpeed;  
      printf ( ">>���С���ٶȣ�%f\n", carSpeed);
      strCapture .ucFinishFlag = 0;			
		}
    barrierScan();                                              //�ϰ���״̬���
    updateData();
    LED_TOGGLE;                                                 //led��ת
    HAL_Delay(200);                                             //ϵͳ��ʱ
    
    
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
  * ��������: ��ʱ�����벶���жϻص�����
  * �������: htim����ʱ�����
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
  TIM_IC_InitTypeDef sConfigIC;
  
  if ( strCapture .ucStartFlag == 0 )
  {
    __HAL_TIM_SET_COUNTER(htim,0); // ���㶨ʱ������
    strCapture .usPeriod = 0;			
    strCapture .usCtr = 0;
    
    // �������벶���������Ҫ���޸Ĵ�����ƽ
    sConfigIC.ICPolarity = GENERAL_TIM_STRAT_ICPolarity;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0;
    HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, GENERAL_TIM_CHANNELx);
    // ����жϱ�־λ
    __HAL_TIM_CLEAR_IT(htim, GENERAL_TIM_IT_CCx);
    // �������벶�񲢿����ж�
    HAL_TIM_IC_Start_IT(&htim5,GENERAL_TIM_CHANNELx);    
    strCapture .ucStartFlag = 1;			
  }		
  
  else
  {
    // ��ȡ��ʱ������ֵ
    strCapture .usCtr = HAL_TIM_ReadCapturedValue(&htim5,GENERAL_TIM_CHANNELx);
    // �������벶���������Ҫ���޸Ĵ�����ƽ
    sConfigIC.ICPolarity = GENERAL_TIM_STRAT_ICPolarity;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 0;
    HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, GENERAL_TIM_CHANNELx);
    
    // ����жϱ�־λ
    __HAL_TIM_CLEAR_IT(htim, GENERAL_TIM_IT_CCx); 
    // �������벶�񲢿����ж�
    HAL_TIM_IC_Start_IT(&htim5,GENERAL_TIM_CHANNELx);    
    strCapture .ucStartFlag = 0;			
    strCapture .ucFinishFlag = 1;    
  }
}
 //��ʱ��TIM6����жϴ���ص�����
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
        TimeCounter++;                                          //��ʱ����++
                
        if(TimeCounter==1000)
        {
            carDistance+=carSpeed;
            Time_Sec++;                                         //Խ������
            if(Time_Sec>UINT32_MAX-5)
              Time_Sec=0;
            TimeCounter = 0;
            printf("\r\nTime=%d\n",Time_Sec);          
          }
        }
     } 
} 
/**
  * ��������: ��黺�����������Ƿ�Ϊ0xff��0
  * �������: pBuffer��Ҫ�ȽϵĻ�������ָ��
  *           BufferLength������������
  * �� �� ֵ: PASSED��������������ȫΪ0xff��0
  *           FAILED��������������������һ����Ϊ0xff��0 
  * ˵    ��: ��
  */
TestStatus eBuffercmp(uint32_t* pBuffer, uint32_t BufferLength)
{
  while (BufferLength--)
  {
    /* SD��������Ŀ���ֵΪ0xff��0 */
    if ((*pBuffer != 0xFFFFFFFF) && (*pBuffer != 0))
    {
      return FAILED;
    }
    pBuffer++;
  }
  return PASSED;
}
 
/**
  * ��������: SD����������
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void SD_EraseTest(void)
{
	/* ��1������ΪSD���������2������Ϊ������ʼ��ַ����3������Ϊ����������ַ */
  sd_status=HAL_SD_Erase(&hsd,WRITE_READ_ADDRESS,WRITE_READ_ADDRESS+NUMBER_OF_BLOCKS*4);
   printf("erase status:%d\r\n",sd_status);
 
	HAL_Delay(500);
  if (sd_status == HAL_OK)
  {	
    /* ��ȡ�ող��������� */
    sd_status = HAL_SD_ReadBlocks_DMA(&hsd,(uint8_t *)Buffer_Block_Rx,WRITE_READ_ADDRESS,NUMBER_OF_BLOCKS);
    printf("erase read status:%d\r\n",sd_status);
    /* �Ѳ�������������Ա� */
    test_status = eBuffercmp(Buffer_Block_Rx,BLOCK_SIZE*NUMBER_OF_BLOCKS);
 
    if(test_status == PASSED)
      printf("���������Գɹ���\r\n" ); 
    else	  
      printf("���������ɹ������ݳ���\r\n" );      
  }
  else
  {
    printf("����������ʧ�ܣ�����SD��֧�ֲ�����ֻҪ��д����ͨ������\r\n" );
  }
}
 
/**
  * ��������: �ڻ���������д����
  * �������: pBuffer��Ҫ���Ļ�����
  *           BufferLength��Ҫ���Ĵ�С
  *           Offset�����ڻ������ĵ�һ��ֵ 
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void Fill_Buffer(uint32_t *pBuffer, uint32_t BufferLength, uint32_t Offset)
{
  uint32_t index = 0;
  /* ������� */
  for (index = 0; index < BufferLength; index++ )
  {
    pBuffer[index] = index + Offset;
  }
}
 
/**
  * ��������: �Ƚ������������е������Ƿ����
  * �������: pBuffer1��Ҫ�ȽϵĻ�����1��ָ��
  *           pBuffer2��Ҫ�ȽϵĻ�����2��ָ��
  *           BufferLength������������
  * �� �� ֵ: PASSED�����
  *           FAILED������
  * ˵    ��: ��
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
  * ��������: SD����д����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void SD_Write_Read_Test(void)
{  
	int i,j = 0;
  /* ������ݵ�д���� */
  Fill_Buffer(Buffer_Block_Tx,BLOCK_SIZE*NUMBER_OF_BLOCKS, 0x6666);
  
  /* ��SD��д������ */
  sd_status = HAL_SD_WriteBlocks_DMA(&hsd,(uint8_t *)Buffer_Block_Tx,WRITE_READ_ADDRESS,NUMBER_OF_BLOCKS);
  printf("write status:%d\r\n",sd_status);
			
  HAL_Delay(500);
  /* ��SD����ȡ���� */
  sd_status = HAL_SD_ReadBlocks_DMA(&hsd,(uint8_t *)Buffer_Block_Rx,WRITE_READ_ADDRESS,NUMBER_OF_BLOCKS);
  printf("read status:%d\r\n",sd_status);
  
  /* �Ƚ����� */
  test_status = Buffercmp(Buffer_Block_Tx, Buffer_Block_Rx, BLOCK_SIZE*NUMBER_OF_BLOCKS/4);	//�Ƚ�
  if(test_status == PASSED)
	{
    printf("����д���Գɹ���\r\n" );
		
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
  	printf("����д����ʧ�ܣ�\r\n " );  
}
/*
*Fatfs�ļ�ϵͳ��д����
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
  HAL_DMA_DeInit(&hdma_sdio);//ע���������DeInit������һ��ͨ��������
 
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
	hdma_sdio.Init.Mode = DMA_NORMAL;//����NORMAL�����������ԣ�����ν
	hdma_sdio.Init.Priority = DMA_PRIORITY_LOW;
 
  /* Associate the DMA handle */
  __HAL_LINKDMA(hsd, hdmatx, hdma_sdio);
  
  /* Stop any ongoing transfer and reset the state*/
  HAL_DMA_Abort(&hdma_sdio);
  
  /* Deinitialize the Channel for new transfer */
  HAL_DMA_DeInit(&hdma_sdio);  //ע���������DeInit������һ��ͨ��������
  
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
*printf��������ض���
*/
//�������´���,֧��printf������ʹ��printf�����Ӵ��������
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 
 
FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
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
