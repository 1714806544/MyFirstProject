#include "m3508.h"

M350x_STA M2006[8];

static int ABS(int a)
{
	return a>0?a:-a;
}

void CAN2_Configuration(u8 mode)
{

  	
  	GPIO_InitTypeDef       GPIO_InitStructure; 
	  CAN_InitTypeDef        CAN_InitStructure;
  	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
   	NVIC_InitTypeDef       NVIC_InitStructure;

    //使能相关时钟
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOB时钟	PB5 Rx  PB6  Tx
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟		
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);//使能CAN2时钟	
	
    //初始化GPIO
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5| GPIO_Pin_6;
    GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化PA11,PA12

	  //引脚复用映射配置
  	GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_CAN2); //GPIOB12复用为CAN2
	  GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_CAN2); //GPIOB13复用为CAN2
  
  	//CAN单元设置
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//非时间触发通信模式   
  	CAN_InitStructure.CAN_ABOM=DISABLE;	//软件自动离线管理	  
  	CAN_InitStructure.CAN_AWUM=DISABLE; //睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
  	CAN_InitStructure.CAN_NART=ENABLE;	//禁止报文自动传送 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//报文不锁定,新的覆盖旧的  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//优先级由报文标识符决定 
  	CAN_InitStructure.CAN_Mode= mode;	  //模式设置 
  	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=CAN_BS2_6tq; //Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=CAN_BS1_7tq;//Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=3;  //分频系数(Fdiv)为brp+1                     1Mbps	
		CAN_Init(CAN2, &CAN_InitStructure);   // 初始化CAN2 
    
		//配置过滤器
   	CAN_FilterInitStructure.CAN_FilterNumber=14;	  //过滤器0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32位 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32位ID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32位MASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活过滤器0
		CAN_FilterInit(&CAN_FilterInitStructure);//滤波器初始化
		    
		CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);//FIFO0消息挂号中断允许.		    
  
		NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // 主优先级为1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // 次优先级为0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);

}

void CAN2_RX0_IRQHandler()
{
    CanRxMsg rx_message;
    if (CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET) 
		{  
		  CAN_Receive(CAN2, CAN_FIFO0, &rx_message);
			M2006_Can_Receive(rx_message);
			
			#if M2006_0
			M3510_Angle_Calculate(M2006);
			#endif
			#if M2006_1
			M3510_Angle_Calculate(M2006+1);
			#endif
			#if M2006_2
			M3510_Angle_Calculate(M2006+2);
			#endif
			#if M2006_3
			M3510_Angle_Calculate(M2006+3);
			#endif
			#if M2006_4
			M3510_Angle_Calculate(M2006+4);
			#endif			
			#if M2006_5
			M3510_Angle_Calculate(M2006+5);
			#endif			
			#if M2006_6
			M3510_Angle_Calculate(M2006+6);
			#endif			
			#if M2006_7
			M3510_Angle_Calculate(M2006+7);
			#endif	
			
			CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);	
		 }
}



void CAN2_SetMotor_0_3(M350x_STA *M350x)
{
  CanTxMsg  Message ;
	u8 mbox;
	u16 k;

	Message.StdId=0X200;
  Message.IDE  =CAN_ID_STD ;
  Message.RTR  =CAN_RTR_DATA;                       
  Message.DLC  =8;  
	#if M2006_0	
  Message.Data[0]=(M350x[0].Out>>8)&0xff;             
	Message.Data[1]=(M350x[0].Out)&0xff;
	#endif
	#if M2006_1
  Message.Data[2]=(M350x[1].Out>>8)&0xff;             
	Message.Data[3]=(M350x[1].Out)&0xff;
	#endif
	#if M2006_2
  Message.Data[4]=(M350x[2].Out>>8)&0xff;             
	Message.Data[5]=(M350x[2].Out)&0xff;
	#endif
	#if M2006_3
  Message.Data[6]=(M350x[3].Out>>8)&0xff;             
	Message.Data[7]=(M350x[3].Out)&0xff;
	#endif
  mbox= CAN_Transmit(CAN2, &Message);   
  while((CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)&&(k<0XFFF))
	{k++;}	//等待发送结束
}

void CAN2_SetMotor_4_7(M350x_STA *M350x)
{
  CanTxMsg  Message;
	u8 mbox;
	u16 k;

	Message.StdId=0X1FF;
  Message.IDE  =CAN_ID_STD ;
  Message.RTR  =CAN_RTR_DATA;                       
  Message.DLC  =8;  
	#if M2006_4	
  Message.Data[0]=(M350x[4].Out>>8)&0xff;             
	Message.Data[1]=(M350x[4].Out)&0xff;
	#endif
	#if M2006_5
  Message.Data[2]=(M350x[5].Out>>8)&0xff;             
	Message.Data[3]=(M350x[5].Out)&0xff;
	#endif
	#if M2006_6
  Message.Data[4]=(M350x[6].Out>>8)&0xff;             
	Message.Data[5]=(M350x[6].Out)&0xff;
	#endif
	#if M2006_7
  Message.Data[6]=(M350x[7].Out>>8)&0xff;             
	Message.Data[7]=(M350x[7].Out)&0xff;
	#endif
  mbox= CAN_Transmit(CAN2, &Message);   
  
	while((CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)&&(k<0XFFF))
	{k++;}	//等待发送结束
}


void M2006_Can_Receive(CanRxMsg rx_message)
{
	int id_num;
	if((rx_message.StdId&0xff0)==0x200&&rx_message.IDE==CAN_Id_Standard)
	{
		id_num=rx_message.StdId&0x00f;
		M2006[id_num-1].Angle&=0x0000;
		M2006[id_num-1].Angle|=(rx_message.Data[0]<<8)|rx_message.Data[1];
		M2006[id_num-1].Speed&=0x0000;
		M2006[id_num-1].Speed|=(rx_message.Data[2]<<8)|rx_message.Data[3];
	}		
}

void M3510_SpeedMode(s32 Speed,M350x_STA *M350x)
{
	/*PID控制速度*/
	PID_Calculate(&(M350x->Speed_PID_STruct),M350x->Speed,Speed);
	/*限制电流*/
	if(M350x->Speed_PID_STruct.Output<-32768)
		M350x->Speed_PID_STruct.Output=-32768;
	if(M350x->Speed_PID_STruct.Output>32767)
		M350x->Speed_PID_STruct.Output=32767;
	M350x->Out=(s16)M350x->Speed_PID_STruct. Output;
}

void M3510_KeepPosition(M350x_STA *M350x,s32 Angle_Sum)
{
	/*PID控制速度*/
	PID_Calculate(&(M350x->Position_PID_STruct),M350x->dAngle_Sum,Angle_Sum);
	/*限制电流*/
	if(M350x->Position_PID_STruct.Output<-32768)
		M350x->Position_PID_STruct.Output=-32768;
	if(M350x->Position_PID_STruct.Output>32767)
		M350x->Position_PID_STruct.Output=32767;
	M350x->Out=M350x->Position_PID_STruct.Output;

}


#define M2006_ADJUST_SPEED_MAX 32767

void M3510_Position_S(M350x_STA *M350x,s32 Angle_Sum)
{
	PID_Calculate(&(M350x->Position_P_PID_STruct),M350x->dAngle_Sum,Angle_Sum);
	
	if(M350x->Position_P_PID_STruct.Output<-M2006_ADJUST_SPEED_MAX)
		M350x->Position_P_PID_STruct.Output=-M2006_ADJUST_SPEED_MAX;
	if(M350x->Position_P_PID_STruct.Output>M2006_ADJUST_SPEED_MAX)
		M350x->Position_P_PID_STruct.Output=M2006_ADJUST_SPEED_MAX;
	
	PID_Calculate(&(M350x->Position_S_PID_STruct),M350x->Speed,M350x->Position_P_PID_STruct.Output);

	if(M350x->Position_S_PID_STruct.Output<-32768)
		M350x->Position_S_PID_STruct.Output=-32768;
	if(M350x->Position_S_PID_STruct.Output>32767)
		M350x->Position_S_PID_STruct.Output=32767;	
	
	M350x->Out=(s16)M350x->Position_S_PID_STruct. Output;
}


void M3510_Angle_Calculate(M350x_STA *M350x)
{
	int dAngle1=0,dAngle2=0,delta;

	/*计算累积旋转角度*/	
	
	if(M350x->Angle>M350x->Angle_Old)
	{
		dAngle1=M350x->Angle-M350x->Angle_Old;
		dAngle2=M350x->Angle-8192-M350x->Angle_Old;
	}
	else 
	{
		dAngle1=M350x->Angle-M350x->Angle_Old;
		dAngle2=M350x->Angle+8192-M350x->Angle_Old;
	}
	if(ABS(dAngle1)<ABS(dAngle2))
	{
		delta=dAngle1;
	}
	else
	{
		delta=dAngle2;
	}
  M350x->Angle_Old=M350x->Angle;
	M350x->dAngle_Sum+=delta;
}
