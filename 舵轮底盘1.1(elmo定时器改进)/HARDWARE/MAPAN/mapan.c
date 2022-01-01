#include "mapan.h"
#include "math.h"
#include "robot_control.h"

Action_data Action_Data;
static int ABS(int a)
{
	return a>0?a:-a;
}
float angle;
int m;
u8 flag3=0;
char start_flag=0;


#if MAPAN_USART == 1
void mapan_init(void)
{
   //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//ʹ��USART1ʱ��
 
	//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9����ΪUSART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10����ΪUSART1
	
	//USART1�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9��GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA9��PA10

   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = 115200;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART1, &USART_InitStructure); //��ʼ������1
	
  USART_Cmd(USART1, ENABLE);  //ʹ�ܴ���1 
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//��������ж�

	//Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
}
#endif

#if MAPAN_USART == 1
void USART1_IRQHandler(void)
{ 
	static float Angle_Z_last=0;//��������һʱ�̵ĽǶ�
	static float angle_Z_temp_1,angle_Z_temp_2,angle_Z_temp;//һЩ����Ƕȵ���ʱ����

	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
	{
			start_flag=1;

		m=USART_ReceiveData(USART1);
#endif
		switch(flag3)
		{
			case 0:
			{
				if(m==0x0d)
					flag3=1;
				else
					flag3=0;
			  break;
			}
			case 1:
			{
				static char i=0;
				if(m==0x0d)
				{
					i++;
					flag3=1;
					if(i==2)
					{
						flag3=0;
						i=0;
					}
				}
				else if(m==0x0a)
					flag3=2;			
				else
					flag3=0;
				break;
			}
			case 2:
			{
				static union
				{
					u8 data_receive[24];
					float act_data[6];
				}posture;
				static int j=0;
				posture.data_receive[j]=m;
				j++;
				if(j==24)
				{
					j=0;
					flag3=0;
					
					
					if(Angle_Z_last<posture.act_data[0])//һ�����
					{
						angle_Z_temp_1=posture.act_data[0]-Angle_Z_last;//��ʱ��
						angle_Z_temp_2=posture.act_data[0]-Angle_Z_last-360;
					}
					else
					{
						angle_Z_temp_1=posture.act_data[0]-Angle_Z_last;//˳ʱ��
						angle_Z_temp_2=360+posture.act_data[0]-Angle_Z_last;
					}
					//����˳ʱ��ת������ʱ��ת������ȡС���Ǹ��Ƕ�
					angle_Z_temp=(ABS(angle_Z_temp_1))>(ABS(angle_Z_temp_2))? angle_Z_temp_2:angle_Z_temp_1;
					Angle_Z_last=posture.act_data[0];
					
					Action_Data.angle_Z=Action_Data.angle_Z+angle_Z_temp;

									
					
					
					
//					Action_Data.angle_x=posture.act_data[1];
//					Action_Data.angle_y=posture.act_data[2];
//					Action_Data.y=posture.act_data[3]+fake_x;
//					Action_Data.x=-posture.act_data[4]+fake_y;
					Action_Data.w=posture.act_data[5];	
					
					Action_Data.x=-posture.act_data[4]-cos(Action_Data.angle_Z/180*3.1415926f)*175+175;
					Action_Data.y=posture.act_data[3]-sin(Action_Data.angle_Z/180*3.1415926f)*175;
/*                           ^
					                   |x
					                   |
					                   |
					                   |  
					y<---------------/\|             ��ǰ��������
					
					
					*/					
					
					
					
				}
				break;
			}	

		}
#if MAPAN_USART == 1
		USART_ClearFlag(USART1, USART_FLAG_TC);
#endif		
	}
}	
