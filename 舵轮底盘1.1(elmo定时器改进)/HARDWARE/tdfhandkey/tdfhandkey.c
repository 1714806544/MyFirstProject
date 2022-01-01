#include "tdfhandkey.h"
#include "includes.h"




TDF_struct tdf_handkey;
RC_Ctl_t RC_CtrlData1;
 void Data_Init(void)
 {
		 	 tdf_handkey.Joystick_Rightx = 1024 ;
			 tdf_handkey.Joystick_Righty = 1024;
			
			 tdf_handkey.Joystick_Leftx = 1024;
			 tdf_handkey.Joystick_Lefty = 1024;	
			
			 tdf_handkey.Buttom_Left  = 3;
			 tdf_handkey.Buttom_Right  = 3;

			 tdf_handkey.Knob =1024;
 }
volatile unsigned char sbus_rx_buffer[18];
 
void uart2_init(u32 bound)
{
   //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;
	 
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_DMA1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3, GPIO_AF_USART2);
	//USART2_RX   GPIOA.3
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10
 
   //USART 初始化设置
	USART_DeInit(USART2);
	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_Even;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx;	//收模式
  USART_Init(USART2, &USART_InitStructure); //初始化串口
	
//	  //UART5 NVIC 配置 
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//开启串口接受中断
	
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream5_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	
	DMA_DeInit(DMA1_Stream5);
	while (DMA_GetCmdStatus(DMA1_Stream5) != DISABLE){}//等待DMA可以设置
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART2->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)sbus_rx_buffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = 18;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	
	DMA_Init(DMA1_Stream5,&DMA_InitStructure);
	DMA_ITConfig(DMA1_Stream5,DMA_IT_TC,ENABLE);
	DMA_Cmd(DMA1_Stream5,ENABLE);
	USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
	USART_Cmd(USART2, ENABLE);                    //使能串口2
}

extern RC_Ctl_t RC_CtrlData1;
extern TDF_struct tdf_handkey;

void RemoteDataProcess(volatile unsigned char *pData)
{
 if(pData == NULL)
 {
 return;
 }
 
	 RC_CtrlData1.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF; 
	 RC_CtrlData1.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
	 RC_CtrlData1.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) |((int16_t)pData[4] << 10)) & 0x07FF;
	 RC_CtrlData1.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;	 
	 RC_CtrlData1.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;
	 RC_CtrlData1.rc.s2 = ((pData[5] >> 4) & 0x0003);
 
	 RC_CtrlData1.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
	 RC_CtrlData1.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
	 RC_CtrlData1.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8); 
	 RC_CtrlData1.mouse.press_l = pData[12];
	 RC_CtrlData1.mouse.press_r = pData[13];
 
	 RC_CtrlData1.key.v = (((int16_t)pData[16])| ((int16_t)pData[17] << 8)) & 0x07FF;
 //your control code ….
}



void DMA1_Stream5_IRQHandler(void)                	//串口1中断服务程序
{
	if(DMA_GetITStatus(DMA1_Stream5, DMA_IT_TCIF5))
	{		
		DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5);
		DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);

		RemoteDataProcess(sbus_rx_buffer);
		if(RC_CtrlData1.mouse.x==0&&RC_CtrlData1.mouse.y==0&&RC_CtrlData1.mouse.z==0&&RC_CtrlData1.mouse.press_l==0&&RC_CtrlData1.mouse.press_r==0)
		{
		 	 tdf_handkey.Joystick_Rightx = RC_CtrlData1.rc.ch0 ;
			 tdf_handkey.Joystick_Righty = RC_CtrlData1.rc.ch1;
			
			 tdf_handkey.Joystick_Leftx = RC_CtrlData1.rc.ch2;
			 tdf_handkey.Joystick_Lefty = RC_CtrlData1.rc.ch3;	
			
			 tdf_handkey.Buttom_Left  = RC_CtrlData1.rc.s1;
			 tdf_handkey.Buttom_Right  = RC_CtrlData1.rc.s2;

			 tdf_handkey.Knob =RC_CtrlData1.key.v ;
//			 DMA_SetCurrDataCounter(DMA1_Stream5,18); 
		}
		else
		{
			DMA_Cmd(DMA1_Stream5, DISABLE);
			while (DMA_GetCmdStatus(DMA1_Stream5) != DISABLE){}  //确保DMA可以设置
			DMA_SetCurrDataCounter(DMA1_Stream5,18);            //数据传输量
			DMA_Cmd(DMA1_Stream5, ENABLE);                     
		}
	}
	DMA_SetCurrDataCounter(DMA1_Stream5,18);          
} 
