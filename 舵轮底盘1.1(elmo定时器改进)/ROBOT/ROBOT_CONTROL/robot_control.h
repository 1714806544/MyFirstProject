#ifndef __MONITION_CONTROL_H
#define __MONITION_CONTROL_H	
#include "stm32f4xx.h"
#include "pid.h"

#define Robot_RouteType_Line                  0    //ֱ��
#define Robot_LineType_Cycle_Clockwise        1		//˳ʱ��
#define Robot_LineType_Cycle_Anticlockwise    2		//��ʱ��
#define Robot_LineType_Rotate_Clockwise	      3
#define Robot_LineType_Rotate_Anticlockwise	  4
#define PI 3.1415926535f

typedef struct
{
	float x;//xλ��
	float y;//yλ��
	float Angular_velocity;//������ԭ����ת���ٶ�
} coordinitioate_Struct;//����ṹ��

typedef struct 
{
	float angle_deg;//�Ƕ�
	float angle_reg;//����
	float cos;
	float sin;
	float tan;
} Angle_Struvt;//�ǶȽṹ��

typedef struct 
{
	float Rho;
	Angle_Struvt Theta;
}Polar_coordinates_Struct;//������ṹ��

typedef struct 
{
	FunctionalState NewState;							
	s16 Max_Speed;												//����ٶ�									
	s16 Max_w_Speed;					            //�����ٶ�
	coordinitioate_Struct Target;					//Ŀ��������ϵ��Ŀ��λ��											
	coordinitioate_Struct Heart;          //Բ��
	coordinitioate_Struct Start_positon; 	//����λ��
	coordinitioate_Struct Target_position;//Ŀ��λ��
	float Angle_Target;
	char Robot_type;                    
	s16 Wheel_V[4];
	float R;
	
	float Slow_angle;										//���ٽǶ���
	float Stop_angle;										//ֹͣ�Ƕ���		
	float Speedup_angle;								//���ٽǶ���
	float Start_w_Speed;								//ĩ�˽��ٶ�											
	float Target_w_Speed;								//ĩ�˽��ٶ�											
	float Slow_accelerated_w_speed;			//���ٽǼ��ٶ�										
	float Speedup_accelerated_w_speed;	//���ٽǼ��ٶ�										
	float angle_rotate_Sum;							//Ҫת�ĽǶ�
	
	float Slow_length;										//������
	float Stop_length;										//ֹͣ��												
	float Speedup_Length;									//������
	float Start_Speed;										//��ʼ�ٶ�											
	float Target_Speed;										//ĩ���ٶ�											
	float Slow_accelerated_speed;					//���ټ��ٶ�										
	float Speedup_accelerated_speed;			//���ټ��ٶ�										
	float angle_reg_Sum;
	
}ROBOT_Status_Struct;

float Robot_Control_Line(ROBOT_Status_Struct *ROBOT_Status);//ֱ��
float Robot_Control_Circle(ROBOT_Status_Struct *ROBOT_Status);//Բ��
float Robot_Control_Rotate(ROBOT_Status_Struct *ROBOT_Status);//��ת
void Keep_Robot_Position(float Angle,float X,float Y);
float tiaoPID(ROBOT_Status_Struct *ROBOT_Status);

void PID(void);
void PID_CLEAR(void);

void Robot_Wait_for_Command(void);
void Keep_PID_Clear(PID_TypeDef *pid);
#endif
