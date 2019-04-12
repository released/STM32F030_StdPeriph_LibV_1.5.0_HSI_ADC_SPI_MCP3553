/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPI_MCP3553_H
#define __SPI_MCP3553_H

/* Platform config -----------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include "string.h"

#include "stm32f0xx.h"

/* Define config -------------------------------------------------------------*/
#define MCP3553_NCS_Low()       					GPIO_ResetBits(GPIOA, GPIO_Pin_8)
#define MCP3553_NCS_High()      					GPIO_SetBits(GPIOA, GPIO_Pin_8) 

#define 	MCP3553_NOP								((uint8_t)0x00)

/* Macro ---------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

void SPI_MCP3553_Read(uint8_t *data1,uint8_t *data2,uint8_t *data3);
void SPI_MCP3553_Write(uint8_t cmd,uint8_t data1,uint8_t data2);

void SPI_MCP3553_Convert(void);

void SPI_MCP3553_Config(void);

/* Exported constants --------------------------------------------------------*/

#endif  /* __SPI_MCP3553_H */

