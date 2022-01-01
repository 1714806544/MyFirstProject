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
//START ����
//�����������ȼ�
#define START_TASK_PRIO			10  ///��ʼ��������ȼ�Ϊ���
//���������ջ��С
#define START_STK_SIZE			128
//���������ջ
OS_STK START_TASK_STK[START_STK_SIZE];
//������
void start_task(void *pdata);

//TASK1����
//�����������ȼ�
#define TASK1_TASK_PRIO			1
//���������ջ��С
#define TASK1_STK_SIZE			512
//�����ջ
OS_STK TASK1_TASK_STK[TASK1_STK_SIZE];
//������
void task1_task(void *pdata);

//TASK2����
//�����������ȼ�
#define TASK2_TASK_PRIO			2
//���������ջ��С
#define TASK2_STK_SIZE			128
//�����ջ
OS_STK TASK2_TASK_STK[TASK2_STK_SIZE];
//������
void task2_task(void *pdata);

//�����������
#define TASK3_TASK_PRIO			3
//���������ջ��С
#define TASK3_STK_SIZE			128
//�����ջ
OS_STK TASK3_TASK_STK[TASK3_STK_SIZE];
//������
void task3_task(void *pdata);

void All_Init(void)
{
	//�����ʼ��
	delay_init(168);	    	                           //��ʱ������ʼ��	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); 	 //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�	
  Elmo_Can_Init();
	CAN2_Configuration(CAN_Mode_Normal);  //can��ʼ��
	Elmo_Motor_Init();
	TIM3_Int_Init(1999, 71);
	TIM2_Int_Init( 199, 71);
	mapan_init();
	uart2_init(100000);
	
	delay_ms(500);
	//���ݳ�ʼ��
	All_PID_Init();
	Data_Init();
}

int main(void)
 {
	delay_init(168);       //��ʱ��ʼ��
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //�жϷ�������
	All_Init();
	
	OSInit();  //UCOS��ʼ��
	OSTaskCreate(start_task,(void*)0,(OS_STK*)&START_TASK_STK[START_STK_SIZE-1],START_TASK_PRIO); //������ʼ����
	OSStart(); //��ʼ����
}

//��ʼ����
void start_task(void *pdata)
{
	OS_CPU_SR cpu_sr=0;
	pdata=pdata;
	OSStatInit();  //����ͳ������
	
	OS_ENTER_CRITICAL();  //�����ٽ���(�ر��ж�)
	OSTaskCreate(task1_task,(void*)0,(OS_STK*)&TASK1_TASK_STK[TASK1_STK_SIZE-1],TASK1_TASK_PRIO);//����TASK1����
	OSTaskCreate(task2_task,(void*)0,(OS_STK*)&TASK2_TASK_STK[TASK2_STK_SIZE-1],TASK2_TASK_PRIO);//����TASK2����
	OSTaskCreate(task3_task,(void*)0,(OS_STK*)&TASK3_TASK_STK[TASK3_STK_SIZE-1],TASK3_TASK_PRIO);//����TASK3����
	OSTaskSuspend(START_TASK_PRIO);//����ʼ����
	OS_EXIT_CRITICAL();  //�˳��ٽ���(���ж�)
}

extern TDF_struct tdf_handkey;
extern coordinitioate_Struct Robot_Coordinate_system;
float Adjust_keep_x = 0, Adjust_keep_y = 0, Adjust_keep_Angle = 0;
u8 Start_Keep_Flag = 0, Start_Line_Flag = 0;
//TASK1����
void task1_task(void *pdata)
{
	while(1)
	{
		//�ֶ�
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

//TASK2����
void task2_task(void *pdata)
{
	while(1)
	{
		delay_ms(5);
	}
}

//TASK3����
void task3_task(void *pdata)
{
	while(1)
	{
		delay_ms(5);
	}
}
