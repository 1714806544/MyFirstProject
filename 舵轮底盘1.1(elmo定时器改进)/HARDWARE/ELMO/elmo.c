#include "elmo.h"
#include "delay.h"

LUN_GU_STA Lun_Gu[5];
u8 Begin_Flag,Speed_Flag;

void Elmo_Can_Init(void)
{

  	GPIO_InitTypeDef GPIO_InitStructure; 
	  CAN_InitTypeDef        CAN_InitStructure;
  	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
//   	NVIC_InitTypeDef  NVIC_InitStructure;

    //使能相关时钟
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能PORTB时钟	                   											                    											 

  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟		
	
    //初始化GPIO
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8| GPIO_Pin_9;
    GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化PA11,PA12

	
	  //引脚复用映射配置
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_CAN1); //GPIOB12复用为CAN2
	  GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_CAN1); //GPIOB13复用为CAN2
  
  	//CAN单元设置
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//非时间触发通信模式   
  	CAN_InitStructure.CAN_ABOM=DISABLE;	//软件自动离线管理	  
  	CAN_InitStructure.CAN_AWUM=DISABLE;//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
  	CAN_InitStructure.CAN_NART=ENABLE;	//禁止报文自动传送 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//报文不锁定,新的覆盖旧的  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//优先级由报文标识符决定 
  	CAN_InitStructure.CAN_Mode= CAN_Mode_Normal;	 //模式设置 
  	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=CAN_BS2_6tq; //Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=CAN_BS1_7tq;//Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=6;  //分频系数(Fdiv)为brp+1	
		CAN_Init(CAN1, &CAN_InitStructure);   // 初始化CAN2
    
		//配置过滤器
   	CAN_FilterInitStructure.CAN_FilterNumber=0;	  //过滤器0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32位 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32位ID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32位MASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活过滤器0
  	CAN_FilterInit(&CAN_FilterInitStructure);//滤波器初始化
   	CAN_FilterInitStructure.CAN_FilterNumber=14;	  //过滤器0
		CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO1;
		CAN_FilterInit(&CAN_FilterInitStructure);//滤波器初始化

		CAN_ITConfig(CAN1,CAN_IT_FMP0,DISABLE);//FIFO0消息挂号中断允许.		    
		CAN_ITConfig(CAN1,CAN_IT_FMP1,DISABLE);//FIFO0消息挂号中断允许.		    
  
//  	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
//  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;     // 主优先级为2
//  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // 次优先级为0
//  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//		NVIC_Init(&NVIC_InitStructure);
//		NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX1_IRQn;
//  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;     // 主优先级为2
//  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;            // 次优先级为0
//  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//  	NVIC_Init(&NVIC_InitStructure);

}

void Elmo_Motor_Init()
{
	CanTxMsg  Message;
	u8 mbox;
	u16 k=0;
	s32	ID=0X301;

	Message.StdId=0X000;
  Message.IDE  =CAN_ID_STD ;
  Message.RTR  =CAN_RTR_DATA;                       
  Message.DLC  =2;  
  Message.Data[0]=01;             
	Message.Data[1]=00;
  mbox= CAN_Transmit(CAN1, &Message);   
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(k<0XFFF))
	{k++;}	//等待发送结束
	delay_ms(100);
	
	for(u8 n=0;n<4;n++)
	{
		//失能
		k=0;
		Message.StdId=ID;
		Message.IDE  =CAN_ID_STD ;
		Message.RTR  =CAN_RTR_DATA;                       
		Message.DLC  =8;  
		Message.Data[0]=0X4D;             
		Message.Data[1]=0X4F;
		Message.Data[2]=0X00;             
		Message.Data[3]=0X00;
		Message.Data[4]=0X00;             
		Message.Data[5]=0X00;
		Message.Data[6]=0X00;             
		Message.Data[7]=0X00;
		mbox= CAN_Transmit(CAN1, &Message);   
		while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(k<0XFFF))
		{k++;}	//等待发送结束
		delay_ms(100);
		//速度模式
		k=0;
		Message.StdId=ID;
		Message.IDE  =CAN_ID_STD ;
		Message.RTR  =CAN_RTR_DATA;                       
		Message.DLC  =8;  
		Message.Data[0]=0X55;             
		Message.Data[1]=0X4D;
		Message.Data[2]=0X00;             
		Message.Data[3]=0X00;
		Message.Data[4]=0X02;             
		Message.Data[5]=0X00;
		Message.Data[6]=0X00;             
		Message.Data[7]=0X00;
		mbox= CAN_Transmit(CAN1, &Message);   
		while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(k<0XFFF))
		{k++;}	//等待发送结束	
		delay_ms(100);	
		
		//使能
		k=0;
		Message.StdId=ID;
		Message.IDE  =CAN_ID_STD ;
		Message.RTR  =CAN_RTR_DATA;                       
		Message.DLC  =8;  
		Message.Data[0]=0X4D;             
		Message.Data[1]=0X4F;
		Message.Data[2]=0X00;             
		Message.Data[3]=0X00;
		Message.Data[4]=0X01;             
		Message.Data[5]=0X00;
		Message.Data[6]=0X00;             
		Message.Data[7]=0X00;
		mbox= CAN_Transmit(CAN1, &Message);   
		while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(k<0XFFF))
		{k++;}	//等待发送结束
			delay_ms(100);
		ID++;
	}
}

void Elmo_Set_Speed(s32 Speed,s32 ID)
{
	CanTxMsg  Message;
	u8 mbox;
	u32 k=0;
	
	Message.StdId=ID;
  Message.IDE  =CAN_ID_STD ;
  Message.RTR  =CAN_RTR_DATA;                       
  Message.DLC  =8; 
  Message.Data[0]=0X4A;             
	Message.Data[1]=0X56;
  Message.Data[2]=0X00;             
	Message.Data[3]=0X00;
  Message.Data[4]=(Speed)&0xff;             
	Message.Data[5]=(Speed>>8)&0xff;

	Message.Data[6]=(Speed>>16)&0xff;  
	Message.Data[7]=(Speed>>24)&0xff;	

  mbox= CAN_Transmit(CAN1, &Message); 
	
//	k = 0;
//	while((mbox == CAN_TxStatus_NoMailBox)&&(k < 0XFFF))
//	{
//		mbox = CAN_Transmit(CAN1, &Message);
//		k ++;
//	}
//	if(k >= 0XFFF) 
//		return;
  
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(k<0XFFF))	{k++;}	//等待发送结束
}

void Elmo_Set_Begin(s32 ID)
{
	CanTxMsg  Message;
	u8 mbox;
	u32 k=0;
	Message.StdId=ID;
  Message.IDE  =CAN_ID_STD ;
  Message.RTR  =CAN_RTR_DATA;                       
  Message.DLC  =4; 
  Message.Data[0]=0X42;             
	Message.Data[1]=0X47;
  Message.Data[2]=0X00;             
	Message.Data[3]=0X00;
  mbox= CAN_Transmit(CAN1, &Message); 

//	k = 0;
//	while((mbox == CAN_TxStatus_NoMailBox)&&(k < 0XFFF))
//	{
//		mbox = CAN_Transmit(CAN1, &Message);
//		k ++;
//	}
//	if(k >= 0XFFF) 
//		return;
  
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(k<0XFFFF))	{k++;}	//等待发送结束
}
