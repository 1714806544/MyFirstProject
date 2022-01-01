#include "timer.h"
#include "stm32f4xx.h"
#include "m3508.h"
#include "wheel_control.h"
#include "robot_control.h"
#include "elmo.h"
#include "includes.h"

extern M350x_STA M2006[8];
extern wheel_Struct	wheel_final_v[5];  

void TIM2_Int_Init(u16 arr, u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  ///ʹ��TIM3ʱ��	
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);//��ʼ��TIM3
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE); //����ʱ��3�����ж�
	TIM_Cmd(TIM2,ENABLE); //ʹ�ܶ�ʱ��3
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn; //��ʱ��3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x03; //��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_Cmd(TIM2,DISABLE);
}


void TIM2_IRQHandler(void)
{
	OSIntEnter();
	static char which_elmo = 1;
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET) //����ж�
	{
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);  //����жϱ�־λ
		
		if(which_elmo >= 9)
			which_elmo = 1;
		switch (which_elmo)
		{
			case 1: Elmo_Set_Speed((s32)wheel_final_v[1].resultant_v * 200,0X301); break;//�����һ�����ֵ����ٶ�ת���ɿ��Ƶ���Ľ��ٶȵ�ϵ��
			case 2: Elmo_Set_Speed((s32)wheel_final_v[2].resultant_v * 200,0X302); break;//������һ������֮��������ӵİ뾶�Ϳ�������
			case 3: Elmo_Set_Speed((s32)wheel_final_v[3].resultant_v * 200,0X303); break;
			case 4: Elmo_Set_Speed((s32)wheel_final_v[4].resultant_v * 200,0X304); break;
			case 5: Elmo_Set_Begin(0X301); break;
			case 6: Elmo_Set_Begin(0X302); break;
			case 7: Elmo_Set_Begin(0X303); break;
			case 8: Elmo_Set_Begin(0X304);TIM_Cmd(TIM2,DISABLE); break;	
			default: which_elmo = 1; break;
		}
//		if(which_elmo % 2 == 0)
//			Robot_Wheel_Control_3508();
		
		which_elmo ++;
	}
	OSIntExit();
}


void TIM3_Int_Init(u16 arr,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///ʹ��TIM3ʱ��
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//��ʼ��TIM3
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //����ʱ��3�����ж�
	TIM_Cmd(TIM3,ENABLE); //ʹ�ܶ�ʱ��3
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //��ʱ��3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x01; //�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


//��ʱ��3�жϷ�����
void TIM3_IRQHandler(void)
{
	OSIntEnter();
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //����ж�
	{
		TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //����жϱ�־λ
		
		Robot_Wheel_Control_3508();
	}
	OSIntExit();
}

