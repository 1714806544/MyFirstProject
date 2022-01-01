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

    //ʹ�����ʱ��
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��PORTBʱ��	                   											                    											 

  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��		
	
    //��ʼ��GPIO
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8| GPIO_Pin_9;
    GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��PA11,PA12

	
	  //���Ÿ���ӳ������
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource8,GPIO_AF_CAN1); //GPIOB12����ΪCAN2
	  GPIO_PinAFConfig(GPIOB,GPIO_PinSource9,GPIO_AF_CAN1); //GPIOB13����ΪCAN2
  
  	//CAN��Ԫ����
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//��ʱ�䴥��ͨ��ģʽ   
  	CAN_InitStructure.CAN_ABOM=DISABLE;	//����Զ����߹���	  
  	CAN_InitStructure.CAN_AWUM=DISABLE;//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
  	CAN_InitStructure.CAN_NART=ENABLE;	//��ֹ�����Զ����� 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//���Ĳ�����,�µĸ��Ǿɵ�  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//���ȼ��ɱ��ı�ʶ������ 
  	CAN_InitStructure.CAN_Mode= CAN_Mode_Normal;	 //ģʽ���� 
  	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;	//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=CAN_BS2_6tq; //Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=CAN_BS1_7tq;//Tbs2��ΧCAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=6;  //��Ƶϵ��(Fdiv)Ϊbrp+1	
		CAN_Init(CAN1, &CAN_InitStructure);   // ��ʼ��CAN2
    
		//���ù�����
   	CAN_FilterInitStructure.CAN_FilterNumber=0;	  //������0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32λ 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32λID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32λMASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //���������0
  	CAN_FilterInit(&CAN_FilterInitStructure);//�˲�����ʼ��
   	CAN_FilterInitStructure.CAN_FilterNumber=14;	  //������0
		CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO1;
		CAN_FilterInit(&CAN_FilterInitStructure);//�˲�����ʼ��

		CAN_ITConfig(CAN1,CAN_IT_FMP0,DISABLE);//FIFO0��Ϣ�Һ��ж�����.		    
		CAN_ITConfig(CAN1,CAN_IT_FMP1,DISABLE);//FIFO0��Ϣ�Һ��ж�����.		    
  
//  	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
//  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;     // �����ȼ�Ϊ2
//  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
//  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//		NVIC_Init(&NVIC_InitStructure);
//		NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX1_IRQn;
//  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;     // �����ȼ�Ϊ2
//  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;            // �����ȼ�Ϊ0
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
	{k++;}	//�ȴ����ͽ���
	delay_ms(100);
	
	for(u8 n=0;n<4;n++)
	{
		//ʧ��
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
		{k++;}	//�ȴ����ͽ���
		delay_ms(100);
		//�ٶ�ģʽ
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
		{k++;}	//�ȴ����ͽ���	
		delay_ms(100);	
		
		//ʹ��
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
		{k++;}	//�ȴ����ͽ���
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
  
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(k<0XFFF))	{k++;}	//�ȴ����ͽ���
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
  
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(k<0XFFFF))	{k++;}	//�ȴ����ͽ���
}
