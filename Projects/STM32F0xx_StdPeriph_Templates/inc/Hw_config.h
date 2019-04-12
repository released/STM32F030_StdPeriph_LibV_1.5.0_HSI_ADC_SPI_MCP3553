/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HW_CONFIG_H
#define __HW_CONFIG_H

/* Platform config -----------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include "Macro.h"
#include "stm32f0xx.h"
#include <stdio.h>

#include "Custom_ADC.h"
#include "Custom_SPI_MCP3553.h"

/* Define config -------------------------------------------------------------*/

extern uint8_t BitFlag;
#define BitFlag_ON(flag)						(BitFlag|=flag)
#define BitFlag_OFF(flag)						(BitFlag&=~flag)
#define BitFlag_READ(flag)					((BitFlag&flag)?1:0)


/* Macro ---------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
void Test_Function(void);

void FlashDataWrite(uint16_t idx , uint32_t data);
uint32_t FlashDataRead(uint16_t idx);

void GPIO_Config_Input(void);
void GPIO_Config_Output(void);

void TIM16_Config(void);
void TIM17_Config(void);

void USART_Test(void);
void USART_Config(void);

void LED_Config(void);
void SysTickConfig(void);
void Delay(__IO uint32_t mTime);
void TimingDelay_Decrement(void);
/* Exported constants --------------------------------------------------------*/

#endif  /* __HW_CONFIG_H */

