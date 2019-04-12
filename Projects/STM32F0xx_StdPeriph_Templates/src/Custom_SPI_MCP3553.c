/* Includes ------------------------------------------------------------------*/
#include "Custom_SPI_MCP3553.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/*SPI variable*/
uint8_t sDummy=0x5A;
//uint8_t Rx_Buffer1 = 0;
extern void Delay(__IO uint32_t mTime);

void MCP3553_Delay(uint32_t nCount)
{
    /* Decrement nCount value */
    while (nCount != 0)
    {
        nCount--;
    }
}

uint8_t SPI_MCP3553_SendByte(uint8_t byte)
{
	/*!< Loop while DR register in not emplty */
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET);
	/*!< Send byte through the sFLASH_SPI peripheral */
	
//	SPI_I2S_SendData(sFLASH_SPI, byte);
//	SPI_I2S_SendData16(sFLASH_SPI, byte);
	SPI_SendData8(SPI1, byte);

	/*!< Wait to receive a byte */
//	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_BSY) == SET);	
	
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET);
	/*!< Return the byte read from the SPI bus */
//	return SPI_I2S_ReceiveData(sFLASH_SPI);
//	return SPI_I2S_ReceiveData16(sFLASH_SPI);
	return SPI_ReceiveData8(SPI1);
}

#if 0
void SPI_MCP3553_DMAReadBufferStart(uint32_t ReadAddr,uint16_t* pBuffer,uint16_t NumByteToWrite)
{
	DMA_InitTypeDef DMA_InitStructure;
	uint32_t Dummy = 0x5A ;	

	/*!< Send "Read from Memory " instruction */
	SPI_MCP3553_SendByte(MCP3553_NOP);

//	/*!< Send ReadAddr high nibble address byte to read from */
//	SPI_MCP3553_SendByte((ReadAddr & 0xFF0000) >> 16);
	/*!< Send ReadAddr medium nibble address byte to read from */
	SPI_MCP3553_SendByte((ReadAddr& 0xFF00) >> 8);
	/*!< Send ReadAddr low nibble address byte to read from */
	SPI_MCP3553_SendByte(ReadAddr & 0xFF);

	/* DMA configuration -------------------------------------------------------*/
	/* Deinitialize DMA Streams */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);

	DMA_DeInit(DMA1_Channel3);
	DMA_DeInit(DMA1_Channel2);

	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	
	/* Configure DMA Initialization Structure */
	DMA_InitStructure.DMA_BufferSize = NumByteToWrite ;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(SPI1->DR)) ;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
	/* Configure TX DMA */
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST ;
	DMA_InitStructure.DMA_MemoryBaseAddr =(uint32_t) &Dummy ;
	DMA_Init(DMA1_Channel3, &DMA_InitStructure);

	/* Configure DMA Initialization Structure */
	DMA_InitStructure.DMA_BufferSize = NumByteToWrite ;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(SPI1->DR)) ;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	/* Configure RX DMA */
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC ;
	DMA_InitStructure.DMA_MemoryBaseAddr =(uint32_t) pBuffer ; 
	DMA_Init(DMA1_Channel2, &DMA_InitStructure);

	/* Enable DMA SPI TX Stream */
	DMA_Cmd(DMA1_Channel3,ENABLE);
	/* Enable DMA SPI RX Stream */
	DMA_Cmd(DMA1_Channel2,ENABLE);
	/* Enable SPI DMA TX Requsts */
	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);
	/* Enable SPI DMA RX Requsts */
	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, ENABLE);
	/* The Data transfer is performed in the SPI using Direct Memory Access */

}

void SPI_MCP3553_DMAWriteBufferStart(uint32_t WriteAddr,uint16_t pBuffer,uint16_t NumByteToWrite)
{
	DMA_InitTypeDef DMA_InitStructure;

	/*!< Send "Read from Memory " instruction */
	SPI_MCP3553_SendByte(WriteAddr);

//	/*!< Send ReadAddr high nibble address byte to read from */
//	SPI_MCP3553_SendByte((ReadAddr & 0xFF0000) >> 16);
//	/*!< Send ReadAddr medium nibble address byte to read from */
//	SPI_MCP3553_SendByte((ReadAddr& 0xFF00) >> 8);
//	/*!< Send ReadAddr low nibble address byte to read from */
//	SPI_MCP3553_SendByte(ReadAddr & 0xFF);

	/* DMA configuration -------------------------------------------------------*/
	/* Deinitialize DMA Streams */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);

	DMA_DeInit(DMA1_Channel3);
	DMA_DeInit(DMA1_Channel2);

	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	
	/* Configure DMA Initialization Structure */
	DMA_InitStructure.DMA_BufferSize = NumByteToWrite ;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(SPI1->DR)) ;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
	/* Configure TX DMA */
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST ;
	DMA_InitStructure.DMA_MemoryBaseAddr =(uint32_t) &pBuffer ;
	DMA_Init(DMA1_Channel3, &DMA_InitStructure);

	/* Configure DMA Initialization Structure */
	DMA_InitStructure.DMA_BufferSize = NumByteToWrite ;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(SPI1->DR)) ;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	/* Configure RX DMA */
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC ;
	DMA_InitStructure.DMA_MemoryBaseAddr =(uint32_t)&sDummy ; 
	DMA_Init(DMA1_Channel2, &DMA_InitStructure);

	/* Enable DMA SPI TX Stream */
	DMA_Cmd(DMA1_Channel3,ENABLE);
	/* Enable DMA SPI RX Stream */
	DMA_Cmd(DMA1_Channel2,ENABLE);
	/* Enable SPI DMA TX Requsts */
	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, ENABLE);
	/* Enable SPI DMA RX Requsts */
	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, ENABLE);
	/* The Data transfer is performed in the SPI using Direct Memory Access */

}

void SPI_MCP3553_DMABufferWait(void)
{
	/* Waiting the end of Data transfer */
	while (DMA_GetFlagStatus(DMA1_FLAG_TC3)==RESET);
	while (DMA_GetFlagStatus(DMA1_FLAG_TC2)==RESET);
	
	/* Clear DMA Transfer Complete Flags */
	DMA_ClearFlag(DMA1_FLAG_GL3);//DMA1_FLAG_TC3
	DMA_ClearFlag(DMA1_FLAG_GL2);//DMA1_FLAG_TC2  
	
	/* Disable DMA SPI TX Stream */
	DMA_Cmd(DMA1_Channel3,DISABLE);
	/* Disable DMA SPI RX Stream */
	DMA_Cmd(DMA1_Channel2,DISABLE);  
	
	/* Disable SPI DMA TX Requsts */
	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Tx, DISABLE);
	/* Disable SPI DMA RX Requsts */
	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx, DISABLE);

}


void SPI_MCP3553_WriteRegister(uint8_t Address , uint16_t Data)
{
#if 1	//use DMA to transfer data
//	Delay_ms(1);
	MCP3553_NCS_Low();	
	
	SPI_MCP3553_DMAWriteBufferStart(Address,Data,2);
	SPI_MCP3553_DMABufferWait();	

//	Delay_ms(1);	
	MCP3553_NCS_High();	
	
#else	//use regular SPI method to transfer data
//	Delay_ms(1);
	MCP3553_NCS_Low();	
	
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
	{};
//	SPI_SendData8(SPI1,Data);
	SPI_SendData8(SPI1, ~(Data));
	
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)
	{};
	SPI_ReceiveData8(SPI1);

//	Delay_ms(1);	
	MCP3553_NCS_High();
#endif
}

uint16_t SPI_MCP3553_ReadRegister(uint8_t Address)
{
#if 1	//use DMA to transfer data
	uint16_t Data = 0 ;

//	Delay_ms(1);
	MCP3553_NCS_Low();	
	
	SPI_MCP3553_DMAReadBufferStart(Address,&Data,2);
	SPI_MCP3553_DMABufferWait();	

//	Delay_ms(1);	
	MCP3553_NCS_High();	

	return Data;
	
#else	//use regular SPI method to transfer data
//	Delay_ms(1);
	MCP3553_NCS_Low();	
	
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_TXE) == RESET)
	{};
//	SPI_SendData8(SPI1,Data);
	SPI_SendData8(SPI1, ~(Data));
	
	while (SPI_I2S_GetFlagStatus(SPI1, SPI_I2S_FLAG_RXNE) == RESET)
	{};
	SPI_ReceiveData8(SPI1);

//	Delay_ms(1);	
	MCP3553_NCS_High();
#endif
}


void SPI_MCP3553_Read(uint8_t *data1,uint8_t *data2,uint8_t *data3)
{
	#if 1
	MCP3553_Delay(1);
	MCP3553_NCS_High();	
	*data1=SPI_MCP3553_SendByte(MCP3553_NOP);	
	*data2=SPI_MCP3553_SendByte(MCP3553_NOP);	
	*data3=SPI_MCP3553_SendByte(MCP3553_NOP);
	
	MCP3553_NCS_Low();	
	MCP3553_Delay(1);	
	MCP3553_NCS_High();
	MCP3553_Delay(1);	
	MCP3553_NCS_Low();	
	
	#else
	MCP3553_Delay(50);
	MCP3553_NCS_Low();	
	printf("\r\n-----\r\n");
	
	*data1=SPI_MCP3553_SendByte(MCP3553_NOP);MCP3553_Delay(25);
	printf("data1=0x%2X\r\n",*data1);
	
	*data2=SPI_MCP3553_SendByte(MCP3553_NOP);MCP3553_Delay(25);
	printf("data2=0x%2X\r\n",*data2);
	
	*data3=SPI_MCP3553_SendByte(MCP3553_NOP);MCP3553_Delay(25);
	printf("data3=0x%2X\r\n",*data3);
	printf("-----\r\n");
	
	MCP3553_Delay(50);	
	MCP3553_NCS_High();	
	#endif
}

void SPI_MCP3553_Write(uint8_t cmd,uint8_t data1,uint8_t data2)
{
	#if 1
	MCP3553_Delay(1);
	MCP3553_NCS_High();	
	SPI_MCP3553_SendByte(cmd);
	SPI_MCP3553_SendByte((uint8_t)data1);
	SPI_MCP3553_SendByte((uint8_t)data2);

	MCP3553_NCS_Low();
	MCP3553_Delay(1);
	MCP3553_NCS_High();
	MCP3553_Delay(1);	
	MCP3553_NCS_Low();	
	#else
	MCP3553_Delay(50);
	MCP3553_NCS_Low();	
	SPI_MCP3553_SendByte(cmd);//MCP3553_Delay(25);
	SPI_MCP3553_SendByte((uint8_t)data1);//MCP3553_Delay(25);
	SPI_MCP3553_SendByte((uint8_t)data2);//MCP3553_Delay(25);
	MCP3553_Delay(50);
	MCP3553_NCS_High();	
	#endif
}
#endif

void SPI_MCP3553_Convert(void)
{
	uint8_t data1,data2,data3=0;
	
    MCP3553_NCS_High();
	Delay(1);
	
    MCP3553_NCS_Low();
	Delay(50);

	#if 1
//	while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6) == Bit_SET);
	
	data1 = SPI_MCP3553_SendByte(MCP3553_NOP);
	data2 = SPI_MCP3553_SendByte(MCP3553_NOP);
	data3 = SPI_MCP3553_SendByte(MCP3553_NOP);
	printf("MCP3553:0x%2X,0x%2X,0x%2X,\r\n",data1,data2,data3);

	#else
	while(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_6) != Bit_RESET);

	data1 = SPI_MCP3553_SendByte(MCP3553_NOP);
	data2 = SPI_MCP3553_SendByte(MCP3553_NOP);
	data3 = SPI_MCP3553_SendByte(MCP3553_NOP);
	printf("MCP3553:0x%2X,0x%2X,0x%2X,\r\n",data1,data2,data3);
	#endif

	Delay(1);
	MCP3553_NCS_High();
	Delay(1);
	
}

/*====================================================================
PCB					MCP3553							MCU
----------------------------------------------------------------------			
*			MR#   : master reset (active LOW)			=>VCC
SCLOCK		SHCP  : shift register clock input			=>SPI1_CLK , PA5
LATCH		STCP  : storage register clock input		=>SPI1_NSS , PA8
DIN			DS    : serial data input					=>SPI1_MOSI , PA7(NA)
SDO			DS    : serial data out						=>SPI1_MISO , PA6

====================================================================*/

void SPI_MCP3553_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	SPI_InitTypeDef  SPI_InitStructure;

	/* Enable the SPI periph */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	/* Enable SCK, MOSI, MISO and NSS GPIO clocks */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
//	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_0);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_0);
//	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_0);

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_1;

	/* SPI SCK pin configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* SPI  MOSI pin configuration */
//	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_7;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* SPI MISO pin configuration */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//NSS
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_1;
	GPIO_Init(GPIOA, &GPIO_InitStructure);	

	MCP3553_NCS_High();
	
	/* SPI configuration -------------------------------------------------------*/
	SPI_I2S_DeInit(SPI1);
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_CRCPolynomial = 7;

	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_Init(SPI1, &SPI_InitStructure);

	/* Initialize the FIFO threshold */
	SPI_RxFIFOThresholdConfig(SPI1, SPI_RxFIFOThreshold_QF);

	/* Enable the SPI peripheral */
	SPI_Cmd(SPI1, ENABLE);

}


