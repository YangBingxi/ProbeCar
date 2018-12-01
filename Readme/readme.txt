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