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
   //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10复用为USART1
	
	//USART1端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = 115200;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART1, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART1, ENABLE);  //使能串口1 
	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断

	//Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
}
#endif

#if MAPAN_USART == 1
void USART1_IRQHandler(void)
{ 
	static float Angle_Z_last=0;//机器人上一时刻的角度
	static float angle_Z_temp_1,angle_Z_temp_2,angle_Z_temp;//一些计算角度的临时变量

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
					
					
					if(Angle_Z_last<posture.act_data[0])//一种情况
					{
						angle_Z_temp_1=posture.act_data[0]-Angle_Z_last;//逆时针
						angle_Z_temp_2=posture.act_data[0]-Angle_Z_last-360;
					}
					else
					{
						angle_Z_temp_1=posture.act_data[0]-Angle_Z_last;//顺时针
						angle_Z_temp_2=360+posture.act_data[0]-Angle_Z_last;
					}
					//无论顺时针转还是逆时针转，都是取小的那个角度
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
					y<---------------/\|             当前世界坐标
					
					
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
