#ifndef __MONITION_CONTROL_H
#define __MONITION_CONTROL_H	
#include "stm32f4xx.h"
#include "pid.h"

#define Robot_RouteType_Line                  0    //直线
#define Robot_LineType_Cycle_Clockwise        1		//顺时针
#define Robot_LineType_Cycle_Anticlockwise    2		//逆时针
#define Robot_LineType_Rotate_Clockwise	      3
#define Robot_LineType_Rotate_Anticlockwise	  4
#define PI 3.1415926535f

typedef struct
{
	float x;//x位置
	float y;//y位置
	float Angular_velocity;//绕坐标原点旋转角速度
} coordinitioate_Struct;//坐标结构体

typedef struct 
{
	float angle_deg;//角度
	float angle_reg;//弧度
	float cos;
	float sin;
	float tan;
} Angle_Struvt;//角度结构体

typedef struct 
{
	float Rho;
	Angle_Struvt Theta;
}Polar_coordinates_Struct;//极坐标结构体

typedef struct 
{
	FunctionalState NewState;							
	s16 Max_Speed;												//最大速度									
	s16 Max_w_Speed;					            //最大角速度
	coordinitioate_Struct Target;					//目标线坐标系下目标位置											
	coordinitioate_Struct Heart;          //圆心
	coordinitioate_Struct Start_positon; 	//启动位置
	coordinitioate_Struct Target_position;//目标位置
	float Angle_Target;
	char Robot_type;                    
	s16 Wheel_V[4];
	float R;
	
	float Slow_angle;										//减速角度区
	float Stop_angle;										//停止角度区		
	float Speedup_angle;								//加速角度区
	float Start_w_Speed;								//末端角速度											
	float Target_w_Speed;								//末端角速度											
	float Slow_accelerated_w_speed;			//减速角加速度										
	float Speedup_accelerated_w_speed;	//加速角加速度										
	float angle_rotate_Sum;							//要转的角度
	
	float Slow_length;										//减速区
	float Stop_length;										//停止区												
	float Speedup_Length;									//加速区
	float Start_Speed;										//起始速度											
	float Target_Speed;										//末端速度											
	float Slow_accelerated_speed;					//减速加速度										
	float Speedup_accelerated_speed;			//加速加速度										
	float angle_reg_Sum;
	
}ROBOT_Status_Struct;

float Robot_Control_Line(ROBOT_Status_Struct *ROBOT_Status);//直线
float Robot_Control_Circle(ROBOT_Status_Struct *ROBOT_Status);//圆弧
float Robot_Control_Rotate(ROBOT_Status_Struct *ROBOT_Status);//自转
void Keep_Robot_Position(float Angle,float X,float Y);
float tiaoPID(ROBOT_Status_Struct *ROBOT_Status);

void PID(void);
void PID_CLEAR(void);

void Robot_Wait_for_Command(void);
void Keep_PID_Clear(PID_TypeDef *pid);
#endif
