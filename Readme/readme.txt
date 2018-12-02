注意：
如果重新用Cube生成过代码需要做以下操作
打开bsp_driver_sd.c文件，到BSP_SD_ReadBlocks_DMA函数中（186行附近），在
/*********************************************************************************
if (HAL_SD_ReadBlocks_DMA(&hsd1, (uint8_t *)pData, ReadAddr, NumOfBlocks) != HAL_OK)
{
  sd_state = MSD_ERROR;
}
/***********************************************************************************
上方添加以下代码：

if(SD_DMAConfigRx(&hsd1) != HAL_OK)
{
		return MSD_ERROR;
}
在BSP_SD_WriteBlocks_DMA（214行附近），在
/***********************************************************************************
if (HAL_SD_WriteBlocks_DMA(&hsd1, (uint8_t *)pData, WriteAddr, NumOfBlocks) != HAL_OK)
{
  sd_state = MSD_ERROR;
}
************************************************************************************/
上方添加以下代码：

if(SD_DMAConfigTx(&hsd1) != HAL_OK)
{
		return MSD_ERROR;
}
/*-----------------------------------------------------------------------------------------------------------------------------------*/
硬件接口表：
MCU-IO		外设			属性			备注
PA0		霍尔传感器Do		TIM5_CH1		霍尔传感器检测到磁体时会有一个高电平
PA2					USART2-TX	
PA3		串口屏TX			USART2-RX	
PA4		控制灯带的继电器口					上拉模式，当检测到有障碍物会跳变到低电平
PA5		小车控制前进的继电器口				上拉模式，接收到前进指令时会跳变到低电平
PA6		小车控制后退的继电器口				上拉模式，当接收到后退指令时会跳变到低电平
PA7		小车控制刹车的继电器口				上拉模式，当接收到刹车指令时会跳变到低电平
PA8		控制摄像头的继电器口				上拉模式，当检测到有障碍物时会跳变到低电平
PA9		串口屏RX			USART1-TX	
PA10					USART1-RX	
PB10					USART3X	
PB11		机关测距模块的TX		USART3RX		对应橙色的杜邦线
PA11		激光雷达的开关量输出口				下拉模式，相应外界的高电平触发
PA12		监测隧道内外的外设接口				下拉模式，当判断在隧道外时输出高电平、隧道内时输出低电平
PC8					SDIO_D0			SDIO数据口D0	
PC9					SDIO_D1			SDIO数据口D1	
PC10					SDIO_D2			SDIO数据口D2	
PC11					SDIO_D3			SDIO数据口D3	
PC12					SDIO_CK			SDIO时钟	
PD2					SDIO_CMD		SDIO命令口	
