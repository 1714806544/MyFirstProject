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

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,ENABLE);  ///使能TIM3时钟	
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM2,&TIM_TimeBaseInitStructure);//初始化TIM3
	
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE); //允许定时器3更新中断
	TIM_Cmd(TIM2,ENABLE); //使能定时器3
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM2_IRQn; //定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x03; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	TIM_Cmd(TIM2,DISABLE);
}


void TIM2_IRQHandler(void)
{
	OSIntEnter();
	static char which_elmo = 1;
	if(TIM_GetITStatus(TIM2,TIM_IT_Update)==SET) //溢出中断
	{
		TIM_ClearITPendingBit(TIM2,TIM_IT_Update);  //清除中断标志位
		
		if(which_elmo >= 9)
			which_elmo = 1;
		switch (which_elmo)
		{
			case 1: Elmo_Set_Speed((s32)wheel_final_v[1].resultant_v * 200,0X301); break;//这里乘一个把轮的线速度转换成控制电机的角速度的系数
			case 2: Elmo_Set_Speed((s32)wheel_final_v[2].resultant_v * 200,0X302); break;//先随便给一个数，之后根据轮子的半径和控制器改
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
	
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///使能TIM3时钟
	
  TIM_TimeBaseInitStructure.TIM_Period = arr; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 
	
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//初始化TIM3
	
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //允许定时器3更新中断
	TIM_Cmd(TIM3,ENABLE); //使能定时器3
	
	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x01; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x01; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}


//定时器3中断服务函数
void TIM3_IRQHandler(void)
{
	OSIntEnter();
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //溢出中断
	{
		TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //清除中断标志位
		
		Robot_Wheel_Control_3508();
	}
	OSIntExit();
}

