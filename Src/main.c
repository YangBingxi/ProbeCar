
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "sdio.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "led.h"
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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

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
  uint32_t ulTmrClk, ulTime;
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
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM5_Init();
  MX_SDIO_SD_Init();
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

int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart1, (uint8_t*)&ch ,1, 0xffff);
      return ch;
}
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
