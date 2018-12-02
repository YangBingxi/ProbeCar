# �����ϰ���̽�⳵

## *ע�⣺*
���������Cube���ɹ�������Ҫ�����²���
��bsp_driver_sd.c�ļ�����BSP_SD_ReadBlocks_DMA�����У�186�и���������
```
if (HAL_SD_ReadBlocks_DMA(&hsd1, (uint8_t *)pData, ReadAddr, NumOfBlocks) != HAL_OK)
{
	sd_state = MSD_ERROR;
}
```
�Ϸ��������´��룺
```
if(SD_DMAConfigRx(&hsd1) != HAL_OK)
{
	return MSD_ERROR;
}
```
��BSP_SD_WriteBlocks_DMA��214�и���������
```
if (HAL_SD_WriteBlocks_DMA(&hsd1, (uint8_t *)pData, WriteAddr, NumOfBlocks) != HAL_OK)
{
	sd_state = MSD_ERROR;
}
```
�Ϸ��������´��룺
```
if(SD_DMAConfigTx(&hsd1) != HAL_OK)
{
	return MSD_ERROR;
}
```
---

## Ӳ���ӿڱ���

MCU-IO|����|����|��ע
---|---|---|---
PA0|	����������Do		|TIM5_CH1		|������������⵽����ʱ����һ���ߵ�ƽ
PA2		|					|USART2-TX		|	
PA3		|������TX			|USART2-RX		|
PA4		|���Ƶƴ��ļ̵�����	|				|����ģʽ������⵽���ϰ�������䵽�͵�ƽ
PA5		|С������ǰ���ļ̵�����|			|����ģʽ�����յ�ǰ��ָ��ʱ�����䵽�͵�ƽ
PA6		|С�����ƺ��˵ļ̵�����|			|����ģʽ�������յ�����ָ��ʱ�����䵽�͵�ƽ
PA7		|С������ɲ���ļ̵�����|			|����ģʽ�������յ�ɲ��ָ��ʱ�����䵽�͵�ƽ
PA8		|��������ͷ�ļ̵�����				|����ģʽ������⵽���ϰ���ʱ�����䵽�͵�ƽ
PA9		|������RX			|USART1-TX		|
PA10	|					|USART1-RX		|
PB10	|					|USART3X		|
PB11	|���ز��ģ���TX   |USART3RX		|��Ӧ��ɫ�ĶŰ���
PA11	|�����״�Ŀ����������|			|����ģʽ����Ӧ���ĸߵ�ƽ����
PA12	|����������������ӿ�|			|����ģʽ�����ж���������ʱ����ߵ�ƽ��������ʱ����͵�ƽ
PC8		|					|SDIO_D0		|SDIO���ݿ�D0	
PC9		|					|SDIO_D1		|SDIO���ݿ�D1	
PC10	|					|SDIO_D2		|SDIO���ݿ�D2	
PC11	|					|SDIO_D3		|SDIO���ݿ�D3	
PC12	|					|SDIO_CK		|SDIOʱ��	
PD2		|					|SDIO_CMD		|SDIO�����	