#include "delay.h"
#include "sys.h"
#include "math.h"
#include "wheel_control.h"
#include "robot_control.h"
#include "lj.h"
#include "math.h"
#include "m3508.h"
#include "usart.h"
#include "pid.h"
#include "mapan.h"

extern ROBOT_Status_Struct ROBOT_Status;
extern Action_data Action_Data;
extern int yanqiexian;
float circle_x;
float circle_y;
float circle_r;
extern u8 Work_Over;

extern PID_TypeDef 
				Keep_X_PID,
				Keep_Y_PID,
				Keep_W_PID,
				Keep_Angle_PID,

				Line_X_PID,
				Line_Y_PID,
				Line_w_PID,
				Line_Angle_PID,

				Cir_X_PID,
				Cir_Y_PID,
				Cir_W_PID,
				Cir_Angle_PID;

//直线（目标x,y,要转的角度，V初，V末，a加速，a减速,w最大）  15000 5000 1000
void line(float Target_x,float Target_y,float rotate_Sum,int Start_Speed,int Target_Speed,int speedup_a,int slow_a,int Max_w,int speed_max)
{
	Work_Over=1;
	ROBOT_Status.Robot_type = 0;
	ROBOT_Status.NewState=ENABLE;
	ROBOT_Status.Start_Speed=Start_Speed;
	ROBOT_Status.Max_Speed=speed_max;
	ROBOT_Status.Target_Speed=Target_Speed;
	ROBOT_Status.Start_positon.x=ROBOT_Status.Target_position.x;
	ROBOT_Status.Start_positon.y=ROBOT_Status.Target_position.y;
	ROBOT_Status.Target_position.x=Target_x;
	ROBOT_Status.Target_position.y=Target_y;
	ROBOT_Status.Stop_length=20;
	ROBOT_Status.Slow_accelerated_speed=slow_a;
	ROBOT_Status.Speedup_accelerated_speed=speedup_a;
	//下面是自转部分
	ROBOT_Status.Start_w_Speed=40;
	ROBOT_Status.Max_w_Speed=Max_w;
	ROBOT_Status.Target_w_Speed=0;
	ROBOT_Status.angle_rotate_Sum=rotate_Sum;     //自转角度   顺时针为负
	ROBOT_Status.Stop_angle=0.5;
	ROBOT_Status.Slow_accelerated_w_speed=10000;//角减速度
	ROBOT_Status.Speedup_accelerated_w_speed=100000;//角加速度
	while(Work_Over)
	{
		Robot_Control_Line(&ROBOT_Status);
		Robot_Wheel_Control();
		delay_ms(2);
	}	
}






//顺时针圆弧（目标x,y,圆心角，圆心x，y，半径，沿切线吗，V初，V末）
void circle(float Target_x,float Target_y,float angle,float c_x,float c_y,float c_r,int yanqie,int start_v,int target_v,int speed_max)
{
    Work_Over=1;
		ROBOT_Status.NewState=ENABLE;
		circle_r=c_r;
		circle_x=c_x;
		circle_y=c_y;
		ROBOT_Status.Start_Speed=start_v;
		ROBOT_Status.Max_Speed=speed_max;
		ROBOT_Status.Target_Speed=target_v;
		ROBOT_Status.Robot_type = Robot_LineType_Cycle_Clockwise;
		ROBOT_Status.angle_reg_Sum=3.1415926f*angle/180;
		ROBOT_Status.R=circle_r;
		ROBOT_Status.Heart.x=circle_x;
		ROBOT_Status.Heart.y=circle_y;
		ROBOT_Status.Start_positon.x=ROBOT_Status.Target_position.x;
		ROBOT_Status.Start_positon.y=ROBOT_Status.Target_position.y;
		ROBOT_Status.Target_position.x=Target_x;
		ROBOT_Status.Target_position.y=Target_y;
		ROBOT_Status.Stop_length=10;
		ROBOT_Status.Slow_accelerated_speed=3000;
		ROBOT_Status.Speedup_accelerated_speed=10000;
		
		yanqiexian=yanqie;
		while(Work_Over)
		{		
			Robot_Control_Circle(&ROBOT_Status);
			Robot_Wheel_Control();
			delay_ms(2);
		}
}


//逆时针圆弧（目标x,y,圆心角，圆心x，y，半径，沿切线吗，V初，V末）
void anticircle(float Target_x,float Target_y,float angle,float c_x,float c_y,float c_r,int yanqie,int start_v,int target_v,int speed_max)
{
    Work_Over=1;
		ROBOT_Status.NewState=ENABLE;
		circle_r=c_r;
		circle_x=c_x;
		circle_y=c_y;
		ROBOT_Status.Start_Speed=start_v;
		ROBOT_Status.Max_Speed=speed_max;
		ROBOT_Status.Target_Speed=target_v;
		ROBOT_Status.Robot_type = Robot_LineType_Cycle_Anticlockwise;   //逆时针
		ROBOT_Status.angle_reg_Sum=3.1415926f*angle/180;
		ROBOT_Status.R=circle_r;
		ROBOT_Status.Heart.x=circle_x;
		ROBOT_Status.Heart.y=circle_y;
		ROBOT_Status.Start_positon.x=ROBOT_Status.Target_position.x;
		ROBOT_Status.Start_positon.y=ROBOT_Status.Target_position.y;
		ROBOT_Status.Target_position.x=Target_x;
		ROBOT_Status.Target_position.y=Target_y;
		ROBOT_Status.Stop_length=10;
		ROBOT_Status.Slow_accelerated_speed=3000;
		ROBOT_Status.Speedup_accelerated_speed=10000;
		
		yanqiexian=yanqie;
		while(Work_Over)
		{		
			Robot_Control_Circle(&ROBOT_Status);
			Robot_Wheel_Control();
			delay_ms(2);
		}
}

//直线（目标x,y,要转的角度，V初，V末，a加速，a减速,w最大）  15000 5000 1000
void lj1_0(void)//远距离回到(750，-750)
{
	line(1000,1000,0,300,0,10000,10000,1000,1000);
}
void lj1_1(float acc_jia,float acc_jian,float v_max)
{
	line(0,0,0,10,0,acc_jia,acc_jian,1000,v_max);
}


//直线（目标x,y,要转的角度，V初，V末，a加速，a减速,w最大）  15000 5000 1000
//顺时针圆弧（目标x,y,圆心角，圆心x，y，半径，沿切线吗，V初，V末）
//逆时针圆弧（目标x,y,圆心角，圆心x，y，半径，沿切线吗，V初，V末）

