/* Includes ------------------------------------------------------------------*/
#include "Hw_config.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
  #define GETCHAR_PROTOTYPE int __io_getchar(void)  
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
  #define GETCHAR_PROTOTYPE int fgetc(FILE *f)  
#endif /* __GNUC__ */

// TODO: for printf function , need to confirm use USART1 or USART2
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(USART1, (uint8_t) ch);

  /* Loop until transmit data register is empty */
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
  {}

  return ch;
}

//GETCHAR_PROTOTYPE
//{
//	while(USART_GetFlagStatus(USART1, USART_FLAG_RXNE == RESET));
//	return (USART_ReceiveData(USART1));
//}

GETCHAR_PROTOTYPE
{
    int ch;
    while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET)
    {
    }
    ch = USART_ReceiveData(USART1);
   
    while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
    {
    }
    USART_SendData(USART1, (uint8_t) ch);
    return ch;
}

/* Private macro -------------------------------------------------------------*/

#define 	ADCReadyBit 						(uint8_t)(1<<0)
#define 	GPIOReadyBit 						(uint8_t)(1<<1)
#define 	UARTReadyBit 						(uint8_t)(1<<2)
#define 	AAAReadyBit 						(uint8_t)(1<<3)
#define 	BBBReadyBit 						(uint8_t)(1<<4)
#define 	CCCReadyBit 						(uint8_t)(1<<5)
#define 	DDDReadyBit 						(uint8_t)(1<<6)
#define 	EEEReadyBit 							(uint8_t)(1<<7)

//GPIO input
#define GPIO_IN1(void)						(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_3))
#define GPIO_IN2(void)						(GPIO_ReadInputDataBit(GPIOA,GPIO_Pin_4))

//GPIO output
#define 	GPIO_OUT1_ENABLE(void)			(GPIO_SetBits(GPIOB,GPIO_Pin_0))
#define GPIO_OUT1_DISABLE(void)			(GPIO_ResetBits(GPIOB,GPIO_Pin_0))

#define 	GPIO_OUT2_ENABLE(void)			(GPIO_SetBits(GPIOB,GPIO_Pin_1))
#define GPIO_OUT2_DISABLE(void)			(GPIO_ResetBits(GPIOB,GPIO_Pin_1))

/* Private variables ---------------------------------------------------------*/
static __IO uint32_t uwTimingDelay;

/*Data variable*/
#define FLASH_USER_START_ADDR   			((uint32_t)0x08007C00)
#define FlashPageSize						(0x400)

//#define StorageSize							(FlashPageSize/4)
#define StorageSize							(10)	//reduce the ram cost
uint32_t tmp[StorageSize];

/*Bit flag variable*/
uint8_t BitFlag = 0;


/* Private functions ---------------------------------------------------------*/

void Test_Function(void)
{
	//BitFlag TEST
	BitFlag_ON(ADCReadyBit);
	BitFlag_ON(GPIOReadyBit);
	BitFlag_OFF(UARTReadyBit);
	BitFlag_ON(AAAReadyBit);
	BitFlag_ON(BBBReadyBit);
	BitFlag_OFF(CCCReadyBit);
	BitFlag_ON(DDDReadyBit);
	BitFlag_OFF(EEEReadyBit);

	printf("BitFlag_Test=========start\r\n");
	printf("0x%2X\r\n",BitFlag_READ(ADCReadyBit));
	printf("0x%2X\r\n",BitFlag_READ(GPIOReadyBit));	
	printf("0x%2X\r\n",BitFlag_READ(UARTReadyBit));	
	printf("0x%2X\r\n",BitFlag_READ(AAAReadyBit));		
	printf("0x%2X\r\n",BitFlag_READ(BBBReadyBit));
	printf("0x%2X\r\n",BitFlag_READ(CCCReadyBit));	
	printf("0x%2X\r\n",BitFlag_READ(DDDReadyBit));	
	printf("0x%2X\r\n",BitFlag_READ(EEEReadyBit));	
	printf("BitFlag_Test=========end\r\n");

	//GPIO IN TEST
	printf("GPIO_IN1:0x%2X\r\n",GPIO_IN1());
	printf("GPIO_IN2:0x%2X\r\n",GPIO_IN2());	

	//GPIO OUT TEST
	GPIO_OUT1_ENABLE();
//	GPIO_OUT1_DISABLE();	
//	GPIO_OUT2_ENABLE();
	GPIO_OUT2_DISABLE();	
}

void FlashDataWrite(uint16_t idx , uint32_t data)
{
	uint16_t i = 0;

	//#0 check input idx if over range
	if (idx>StorageSize)
	{
		return ;	//ERROR
	}
	
	//#1 Backup data
	do
	{
		tmp[i] = *(__IO uint32_t*)(FLASH_USER_START_ADDR+i*4);
//		printf("0x%4X\r\n",tmp[i]);//debug
	}while (++i != StorageSize);
	
	//#2 Erase flash
	FLASH_Unlock();
	FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR); 

	if (FLASH_ErasePage(FLASH_USER_START_ADDR) != FLASH_COMPLETE)
	{
//		printf("ERASE FAIL!!!!!\r\n");//debug
	}
	else
	{
		printf("ERASE OK!!!!!\r\n");//debug
	}

	//#3 Replace data
	tmp[idx]=data;
//	for (i=0;i<size;i++)
//	{
//		printf("TEST%4d\r\n",SaveData[i]);//debug
//	}

	//#4 Write data
	i = 0;
	do
	{
		if (FLASH_ProgramWord(FLASH_USER_START_ADDR+i*4,tmp[i])!=FLASH_COMPLETE)
		{
			printf("WRITE FAIL!!!!!\r\n");//debug
		}
		else
		{
//			printf("WRITE OK(%2d,0x%4X)!!!!!\r\n",i,tmp[i]);//debug
		}
		
	}while (++i != StorageSize);
	
	FLASH_Lock();

}

uint32_t FlashDataRead(uint16_t idx)
{
	return (*(__IO uint32_t*)(FLASH_USER_START_ADDR+idx*4));
}

void GPIO_Config_Input(void)	//PA3 , PA4
{
  	GPIO_InitTypeDef    GPIO_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3|GPIO_Pin_4;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;	
	GPIO_Init(GPIOA, &GPIO_InitStructure);	
}

void GPIO_Config_Output(void)		//PB0,PB1
{
  	GPIO_InitTypeDef    GPIO_InitStructure;

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOB, ENABLE);		
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0|GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;	
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;	
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

/*
	TIMxCLK = PCLK1 = HCLK = SystemCoreClock
	TIMx counter clock = TIMxCLK /((Prescaler + 1)*(Period + 1))
	                = 48 MHz / ((3+1)*(11999+1))
	                = 1000 Hz 
     ==> TIMx counter period = 1 ms
*/
void TIM17_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	/* TIMx clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM17, ENABLE);

	/* TIMx base configuration */
	TIM_TimeBaseStructure.TIM_Period = 11999;
	TIM_TimeBaseStructure.TIM_Prescaler = 3;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM17, &TIM_TimeBaseStructure);
	
	/* TIM Interrupts enable */
	TIM_ITConfig(TIM17, TIM_IT_Update, ENABLE);

	/* TIMx enable counter */
	TIM_Cmd(TIM17, ENABLE);

	/* Enable the TIMx gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM17_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/*
	TIMxCLK = PCLK1 = HCLK = SystemCoreClock
	TIMx counter clock = TIMxCLK /((Prescaler + 1)*(Period + 1))
	                = 48 MHz / ((3+1)*(11999+1))
	                = 1000 Hz 
     ==> TIMx counter period = 1 ms
*/
void TIM16_Config(void)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	
	/* TIMx clock enable */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16, ENABLE);

	/* TIMx base configuration */
	TIM_TimeBaseStructure.TIM_Period = 11999;
	TIM_TimeBaseStructure.TIM_Prescaler = 3;
	TIM_TimeBaseStructure.TIM_ClockDivision = 0;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

	TIM_TimeBaseInit(TIM16, &TIM_TimeBaseStructure);
	
	/* TIM Interrupts enable */
	TIM_ITConfig(TIM16, TIM_IT_Update, ENABLE);

	/* TIMx enable counter */
	TIM_Cmd(TIM16, ENABLE);

	/* Enable the TIMx gloabal Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = TIM16_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void USART_Test(void)
{
//	uint32_t i=0;
//	uint32_t j=0;
//	uint32_t displaycounter=0;
	__IO uint8_t temp;
//	uint16_t Color = 0 ;	
	
	if(USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == SET)
	{
			temp = USART_ReceiveData(USART1);
			printf("temp = 0x%x \n\r",temp);

			switch (temp)
			{

				case '3':	//test flash
					FLASH_Unlock();
					FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR); 

					FLASH_ErasePage(FLASH_USER_START_ADDR);
					FLASH_ProgramHalfWord(FLASH_USER_START_ADDR+4*0,1111);
					FLASH_ProgramHalfWord(FLASH_USER_START_ADDR+4*1,2222);
					FLASH_ProgramHalfWord(FLASH_USER_START_ADDR+4*2,3333);
					FLASH_ProgramHalfWord(FLASH_USER_START_ADDR+4*3,4444);
					printf("TEST WRITE FINISH\r\n");
					break;
				case '4':
					//for test
					FlashDataWrite(0,0x2266);
					FlashDataWrite(4,0x1289);
					FlashDataWrite(10,0xDA2212CE);					
					FlashDataWrite(99,0x9527);					
					break;					
				case '5':
					printf("DataIndex: (0x%4X)\r\n",FlashDataRead(0));
					printf("DataIndex: (0x%4X)\r\n",FlashDataRead(4));	
					printf("DataIndex: (0x%4X)\r\n",FlashDataRead(10));						
					printf("DataIndex: (0x%4X)\r\n",FlashDataRead(99));					
					break;


			}
	}
}

void USART_Config(void)
{
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;

	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable GPIO clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);

	/* Enable UART clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	/* Connect PXx to USARTx_Tx*/
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_1);

	/* Connect PXx to USARTx_Rx*/
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_1);

	/* Configure USART Tx as alternate function  */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Configure USART Rx as alternate function  */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* USARTx configured as follows:
	- BaudRate = 115200 baud  
	- Word Length = 8 Bits
	- One Stop Bit
	- No parity
	- Hardware flow control disabled (RTS and CTS signals)
	- Receive and transmit enabled
	*/
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	/* USART configuration */
	USART_Init(USART1, &USART_InitStructure);

	#if 1
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); 

	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
	{}
	
	/* NVIC configuration */
	/* Enable the USARRx Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn; 
	NVIC_InitStructure.NVIC_IRQChannelPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	#endif

	/* Enable USART */
	USART_Cmd(USART1, ENABLE);

	printf("\n\rUSART Printf Example: retarget the C library printf function to the USART\n\r");

}

uint32_t Button_GetState(void)
{
  return GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0);
}

void Button_Config_EXTI(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the BUTTON Clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/* Configure Button pin as input */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* Connect Button EXTI Line to Button GPIO Pin */
    SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);

    /* Configure Button EXTI line */
    EXTI_InitStructure.EXTI_Line = EXTI_Line0;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;

	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;  
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    /* Enable and set Button EXTI Interrupt to the lowest priority */
    NVIC_InitStructure.NVIC_IRQChannel = EXTI0_1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&NVIC_InitStructure); 	
}

void Button_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
//	EXTI_InitTypeDef EXTI_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;

	/* Enable the BUTTON Clock */
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

	/* Configure Button pin as input */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
}

void LED_Config(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /* Enable the GPIO_LED Clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOC, ENABLE);

  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);
}

void SysTickConfig(void)
{
	/* This function fills the RCC_ClockFreq structure with the current
	 frequencies of different on chip clocks (for debug purpose) */	
	RCC_ClocksTypeDef RCC_Clocks;	 
	RCC_GetClocksFreq(&RCC_Clocks);

	printf("===========================\r\n");
	printf("SYSCLK_Frequency = %d Hz\n\r",RCC_Clocks.SYSCLK_Frequency);	
	printf("HCLK = %d Hz\n\r",RCC_Clocks.HCLK_Frequency);
	printf("PCLK = %d Hz\n\r",RCC_Clocks.PCLK_Frequency);
	printf("I2C1CLK = %d Hz\n\r",RCC_Clocks.I2C1CLK_Frequency);
	
	/* Setup SysTick Timer for 1ms interrupts  */
	if (SysTick_Config(SystemCoreClock / 1000))
	{
		/* Capture error */
		while (1);
	}
	
	/* Configure the SysTick handler priority */
	NVIC_SetPriority(SysTick_IRQn, 0x01);
}

/**
  * @brief  Inserts a delay time.
  * @param  nTime: specifies the delay time length, in milliseconds.
  * @retval None
  */
void Delay(__IO uint32_t mTime)
{ 
	uwTimingDelay = mTime;
	while(uwTimingDelay != 0);
}

/**
  * @brief  Decrements the TimingDelay variable.
  * @param  None
  * @retval None
  */
void TimingDelay_Decrement(void)
{
  if (uwTimingDelay != 0x00)
  { 
    uwTimingDelay--;
  }
}

//currently not use
/*

void SystemClkDelay(void)
{
	uint32_t i;
	i = 0xffff;
	while(i--);
}

void wtPutChar(uint8_t ccc)
{
	UART1_SendData8(ccc);
	// Loop until the end of transmission 
	while (UART1_GetFlagStatus(UART1_FLAG_TXE) == RESET);	
}

u16 GetAbsTime(u16 a,u16 b)
{
	u16 c;
	if(a>=b) c=(a-b);
	else c=65535-(b-a);	
	
	return c;
}
*/
uint8_t UART_GetByte(void)
{
	while ( USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET);
	{
	}
	return (uint8_t)USART_ReceiveData(USART1);
}

void UART_SendByte(uint8_t Data)
{
	USART_SendData(USART1 , (unsigned char)Data);
	while (USART_GetFlagStatus(USART1 , USART_FLAG_TC)==RESET);
	{
	}
}

void UART_SendString(uint8_t* Data,uint16_t len)
{
	#if 1
	uint16_t i=0;
	for(i=0;i<len;i++ )
	{
		UART_SendByte(Data[i]);
	}
	#else	//ignore len
    while(*Data)  
    {  
        USART_SendData(USART1, (unsigned char) *Data++);  
        /* Loop until the end of transmission */  
        while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);  //USART_FLAG_TXE
    } 
	#endif
}

void SystemClkDelay(uint32_t u32Delay)
{
	//uint32_t i;
	//i = 0xffff;
	while(u32Delay--);
}


