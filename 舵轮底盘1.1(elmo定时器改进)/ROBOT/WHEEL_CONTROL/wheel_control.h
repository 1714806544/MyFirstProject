#ifndef __WHEEL_CONTROL_H
#define __WHEEL_CONTROL_H
#include "sys.h"
#include "robot_control.h"


#define HASSIS_Struct_o1_length		1							
#define HASSIS_Struct_o2_length  	1 
#define HASSIS_Struct_o3_length		1								
#define HASSIS_Struct_o4_length   1
#define HASSIS_Struct_o1_angle    PI/4
#define HASSIS_Struct_o2_angle    PI/4
#define HASSIS_Struct_o3_angle    PI/4
#define HASSIS_Struct_o4_angle    PI/4

#define vector_mode 1
#define circle_mode 2
#define keep_mode 3
#define stop_mode 4

typedef struct
{
	float x;
	float y;
	float resultant_v;
	float absolute_angle;//��ǰ������н� ����ʱ���0��360����0,360��  �������ӵľ���ʽ�Ƕ�
	float angle_sum;  //    polar_angle���֣�������������       �� �����ӵ�����ʽ�Ƕ�
	float angle_gap;  //������Ŀ���ٶȽǶȵĲ�ֵ��Ҳ����Ҫת�ĽǶ�
	float v_angle; //Ŀ����ٶȵĽǶ�
} wheel_Struct;


extern u8 Wheel_mode;

void gain_absolute_angle(wheel_Struct *wheel_v,int id);
void gain_gap_angle(wheel_Struct *wheel_v);


void Robot_Wheel_Control_3508(void);
void Robot_Wheel_Control(void);


//�Լ���ӵ�
void Robot_Debounce(float dead_distance);
#endif
