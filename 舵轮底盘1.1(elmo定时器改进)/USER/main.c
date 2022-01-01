#include "robot_control.h"
#include "wheel_control.h"
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "includes.h"
#include "timer.h"
#include "elmo.h"
#include "m3508.h"
#include "tdfhandkey.h"
#include "mapan.h"
#include "lj.h"

//123git test
//START 任务
//设置任务优先级
#define START_TASK_PRIO			10  ///开始任务的优先级为最低
//设置任务堆栈大小
#define START_STK_SIZE			128
//任务任务堆栈
OS_STK START_TASK_STK[START_STK_SIZE];
//任务函数
void start_task(void *pdata);

//TASK1任务
//设置任务优先级
#define TASK1_TASK_PRIO			1
//设置任务堆栈大小
#define TASK1_STK_SIZE			512
//任务堆栈
OS_STK TASK1_TASK_STK[TASK1_STK_SIZE];
//任务函数
void task1_task(void *pdata);

//TASK2任务
//设置任务优先级
#define TASK2_TASK_PRIO			2
//设置任务堆栈大小
#define TASK2_STK_SIZE			128
//任务堆栈
OS_STK TASK2_TASK_STK[TASK2_STK_SIZE];
//任务函数
void task2_task(void *pdata);

//浮点测试任务
#define TASK3_TASK_PRIO			3
//设置任务堆栈大小
#define TASK3_STK_SIZE			128
//任务堆栈
OS_STK TASK3_TASK_STK[TASK3_STK_SIZE];
//任务函数
void task3_task(void *pdata);

void All_Init(void)
{
	//外设初始化
	delay_init(168);	    	                           //延时函数初始化	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 	 //设置NVIC中断分组2:2位抢占优先级，2位响应优先级	
  Elmo_Can_Init();
	CAN2_Configuration(CAN_Mode_Normal);  //can初始化
	Elmo_Motor_Init();
	TIM3_Int_Init(1999, 71);
	TIM2_Int_Init( 199, 71);
	mapan_init();
	uart2_init(100000);
	
	delay_ms(500);
	//数据初始化
	All_PID_Init();
	Data_Init();
}

int main(void)
 {
	delay_init(168);       //延时初始化
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //中断分组配置
	All_Init();
	
	OSInit();  //UCOS初始化
	OSTaskCreate(start_task,(void*)0,(OS_STK*)&START_TASK_STK[START_STK_SIZE-1],START_TASK_PRIO); //创建开始任务
	OSStart(); //开始任务
}

//开始任务
void start_task(void *pdata)
{
	OS_CPU_SR cpu_sr=0;
	pdata=pdata;
	OSStatInit();  //开启统计任务
	
	OS_ENTER_CRITICAL();  //进入临界区(关闭中断)
	OSTaskCreate(task1_task,(void*)0,(OS_STK*)&TASK1_TASK_STK[TASK1_STK_SIZE-1],TASK1_TASK_PRIO);//创建TASK1任务
	OSTaskCreate(task2_task,(void*)0,(OS_STK*)&TASK2_TASK_STK[TASK2_STK_SIZE-1],TASK2_TASK_PRIO);//创建TASK2任务
	OSTaskCreate(task3_task,(void*)0,(OS_STK*)&TASK3_TASK_STK[TASK3_STK_SIZE-1],TASK3_TASK_PRIO);//创建TASK3任务
	OSTaskSuspend(START_TASK_PRIO);//挂起开始任务
	OS_EXIT_CRITICAL();  //退出临界区(开中断)
}

extern TDF_struct tdf_handkey;
extern coordinitioate_Struct Robot_Coordinate_system;
float Adjust_keep_x = 0, Adjust_keep_y = 0, Adjust_keep_Angle = 0;
u8 Start_Keep_Flag = 0, Start_Line_Flag = 0;
//TASK1任务
void task1_task(void *pdata)
{
	while(1)
	{
		//手动
		/*
		Robot_Coordinate_system.x = (tdf_handkey.Joystick_Righty - 1024);
		Robot_Coordinate_system.y = -(tdf_handkey.Joystick_Rightx - 1024);
		Robot_Coordinate_system.Angular_velocity = -(tdf_handkey.Joystick_Leftx - 1024);
		*/
		
		//KeepPosition
		Robot_Wheel_Control();
//		if(Start_Keep_Flag == 1)
//		{
//			Keep_Robot_Position(Adjust_keep_Angle, Adjust_keep_x, Adjust_keep_y);
//			Start_Keep_Flag = 0;
//		}
//		if(Start_Line_Flag == 2)
//		{
//			lj1_0();
//			Start_Line_Flag = 0;
//		}
//		Robot_Wait_for_Command();
//		delay_ms(5);
	}
}

//TASK2任务
void task2_task(void *pdata)
{
	while(1)
	{
		delay_ms(5);
	}
}

//TASK3任务
void task3_task(void *pdata)
{
	while(1)
	{
		delay_ms(5);
	}
}
