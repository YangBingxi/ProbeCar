#include "Led.h" 
#include "gpio.h"

void Led_Set(uint8_t Led, uint8_t Status)
{
    if(Led==0)
    {
      if(Status==LedON)
          HAL_GPIO_WritePin(Led_1_GPIO_Port, Led_1_Pin, GPIO_PIN_RESET);
      else if(Status==LedOFF)
          HAL_GPIO_WritePin(Led_1_GPIO_Port, Led_1_Pin, GPIO_PIN_SET);    
    }
    
}
