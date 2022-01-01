#include "robot_control.h"
#include "wheel_control.h"
#include "math.h"
#include "usart.h"
#include "m3508.h"
#include "pid.h"
#include "mapan.h"
#include "delay.h"
#include "timer.h"

//单位统一    (未完成)
//速度v   	单位：mm/s
//路程s   	单位：mm
//加速度a 	单位：mm/s2
//角度θ     单位：°   (度) 
//角速度w 	单位：rad/s (弧度每秒)
//角加速度α 单位：rad/s2(弧度每二次方秒)

ROBOT_Status_Struct ROBOT_Status;
ROBOT_Status_Struct ROBOT_Wheel;
u8 Work_Over;
int yanqiexian;//延切线

Polar_coordinates_Struct Polar_coordinates;		//拉出来的
coordinitioate_Struct  World_Coordinate_system_Position	/*世界坐标系--位置*/
					  ,World_Coordinate_system_Velocity	/*世界坐标系--速度*/
					  ,Velocity_Coordinate_system		/*速度坐标系*/
					  ,Robot_Coordinate_system			/*机器人车身坐标系*/
					  ,TargetLine_Coordinate_system		/*目标线坐标系，相当于在速度坐标系下的位置*/
					  ,ZERO_Position={.x=0,.y=0};		/*原点*/

Angle_Struvt 		   gyroIntegral						/*车身坐标系*/
					  ,SlopeAngle_LINE;					/*目标线坐标系*/

extern Action_data Action_Data;

static float ABS(float a)
{
	return a>0?a:-a;
}

//坐标转换函数 
static void coordinitioate_transformation (coordinitioate_Struct Coordinitioate_To_Convert,
										   coordinitioate_Struct *Coordinitioate_Result,
										   coordinitioate_Struct start_position,
										   Angle_Struvt* Angle)
{
	Coordinitioate_Result->x=+(Coordinitioate_To_Convert.x-start_position.x)*Angle->cos
							 -(Coordinitioate_To_Convert.y-start_position.y)*Angle->sin;
	Coordinitioate_Result->y=(Coordinitioate_To_Convert.x-start_position.x)*Angle->sin
							 +(Coordinitioate_To_Convert.y-start_position.y)*Angle->cos;
}

//根据两点计算角度大小
static void Angle_Calculate(Angle_Struvt *Arget_angle,
							coordinitioate_Struct Position_start,
							coordinitioate_Struct Position_end)
{
		if(Position_start.x<Position_end.x)
		{
			Arget_angle->tan=(Position_end.y-Position_start.y)
							/(Position_end.x-Position_start.x);
			Arget_angle->angle_reg=atan(Arget_angle->tan);
			Arget_angle->sin=sin(Arget_angle->angle_reg);
			Arget_angle->cos=cos(Arget_angle->angle_reg);
		}
		else if(Position_start.x==Position_end.x)
		{
			if(Position_start.y<=Position_end.y)
			{
				Arget_angle->angle_reg=PI/2;
				Arget_angle->tan=tan(Arget_angle->angle_reg);
				Arget_angle->sin=sin(Arget_angle->angle_reg);
				Arget_angle->cos=cos(Arget_angle->angle_reg);
			}
			else if(Position_start.y>=Position_end.y)
			{
				Arget_angle->angle_reg=3*PI/2;
				Arget_angle->tan=tan(Arget_angle->angle_reg);
				Arget_angle->sin=sin(Arget_angle->angle_reg);
				Arget_angle->cos=cos(Arget_angle->angle_reg);
			}
		}
		else if(Position_start.x>Position_end.x)
		{
			Arget_angle->tan=(Position_end.y-Position_start.y)
							/(Position_end.x-Position_start.x);
			Arget_angle->angle_reg=PI+atan(Arget_angle->tan);
			Arget_angle->sin=sin(Arget_angle->angle_reg);
			Arget_angle->cos=cos(Arget_angle->angle_reg);
		}
}
//sin和cos算角度
float get_angle(float sin_num,float cos_num)
{//极坐标
	if(sin_num>=0&&cos_num>=0)//第一象限点
		return asin(sin_num);
	else if(sin_num>=0&&cos_num<=0)//第二象限
		return PI-asin(sin_num);
	else if(sin_num<=0&&cos_num<=0)//第三象限
		return PI-asin(sin_num);
	else if(sin_num<=0&&cos_num>=0)//第四象限
		return 2*PI+asin(sin_num);
	return 0;
}


/*恒加速减速计算--计算末速度*/
static float SlowDown_VelocityCalculate(float V_0 ,float X,float a)
{
	float temp;
	temp=2*a*X+V_0*V_0;
	return sqrt(temp>0?temp:-temp);
}
/*恒加速减速计算--计算距离*/
static float SlowDown_PositionCalculate(float V_0 ,float V,float a)
{
	float temp;
	temp=(V*V-V_0*V_0)/2/a;
	return temp>0?temp:-temp;
}

//角度取负
static void Angle_invert(Angle_Struvt *Arget_angle)
{
		Arget_angle->angle_reg*=-1;
		Arget_angle->sin*=-1;
		Arget_angle->tan*=-1;
}

PID_TypeDef 
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



float Angle_offset;//剩余要转的角度
float Angle_Z_0;//初始航向角
float Angle_Aim;//目标角度值
float X0,Y0;//初始位置

/////////////////////////////////////////////////////////////////////////////////////////////////////////////直线
float Robot_Control_Line(ROBOT_Status_Struct *Status)
{
	static Angle_Struvt Slope_angle;	//速度坐标系与世界坐标系的夹角
	/*如果是第一次进入走该路径的程序,需要计算一些基本参数，并将一些参数初始化*/
	if(Status->NewState==ENABLE)
	{
		Status->NewState=DISABLE;
		//目标连线的角度
		Angle_Calculate(&(SlopeAngle_LINE),	Status->Start_positon, Status->Target_position);
		Angle_invert(&(SlopeAngle_LINE));
		//速度坐标系与世界坐标系的夹角计算，角度大小与目标连线的角度互为相反数
		Angle_Calculate(&(Slope_angle), Status->Start_positon, Status->Target_position);
		//将目标点的坐标从世界坐标系下转换到目标线坐标系，便于之后计算
 		coordinitioate_transformation(Status->Target_position,
									  &Status->Target, 
									  Status->Start_positon, 
									  &SlopeAngle_LINE);
		//计算减速区大小
		Status->Slow_length=SlowDown_PositionCalculate(Status->Max_Speed,
															 Status->Target_Speed,
															 Status->Slow_accelerated_speed);
		//计算加速区大小
		Status->Speedup_Length=SlowDown_PositionCalculate(Status->Start_Speed,
																Status->Max_Speed,
																Status->Speedup_accelerated_speed);
		Angle_Z_0=Status->Angle_Target;		
		
		//以下是自转部分计算
		Angle_Aim=Angle_Z_0+Status->angle_rotate_Sum;
		//计算减速区大小
			
		Status->Slow_angle=SlowDown_PositionCalculate(Status->Max_w_Speed,
															 Status->Target_w_Speed,
															 Status->Slow_accelerated_w_speed);
		//计算加速区大小
		Status->Speedup_angle=SlowDown_PositionCalculate(Status->Start_w_Speed,
										  					Status->Max_w_Speed,
																Status->Speedup_accelerated_w_speed);
		Status->Angle_Target=Angle_Aim;
		
	}
	World_Coordinate_system_Position.x=Action_Data.x;
	World_Coordinate_system_Position.y=Action_Data.y;
	/* 0 计算在目标线坐标系下机器人的当前坐标 */
	coordinitioate_transformation(World_Coordinate_system_Position,
								  &TargetLine_Coordinate_system,
								  Status->Start_positon,
								  &SlopeAngle_LINE);
	
	//计算角速度在每个区间的值
	//以下角速度计算方法改自直线行走，注释暂时未改，看程序时以英文名为主
	//计算与目标角度差值
	Angle_offset=(Angle_Aim - Action_Data.angle_Z);
	if(Status->angle_rotate_Sum<0)	Angle_offset*=-1;//说明是顺时针，保持角度差值是恒正的

	/* 1.1计算速度坐标系下的切向速度 */
	if(ABS(Action_Data.angle_Z-Angle_Z_0 )< Status->Speedup_angle)
	{
		Robot_Coordinate_system.Angular_velocity=
		-SlowDown_VelocityCalculate(Status->Start_w_Speed,//初速度
									 Action_Data.angle_Z-Angle_Z_0 ,//距离
									Status->Speedup_accelerated_w_speed);//加速度
	}
	// 1.1.1 如果没在减速区外，切向速度为正常速度
	else if(Angle_offset >= Status->Slow_angle)
	{
		Robot_Coordinate_system.Angular_velocity=-Status->Max_w_Speed;
	}
	//1.1.2如果在减速区内停止区外
	else if(Angle_offset >= Status->Stop_angle)
	{
		Robot_Coordinate_system.Angular_velocity=
		-SlowDown_VelocityCalculate(Status->Target_w_Speed,//初速度
									 Angle_offset,				  //距离
									 Status->Slow_accelerated_w_speed);//加速度
	}	
	//1.1.3末尾速度为零，则需要在直线结尾精确定位
	else if(fabs(Status->Target_w_Speed)<0.001f && Status->Stop_angle!=0)
	{
		PID_Calculate(&Line_Angle_PID,Angle_offset,0);
		Robot_Coordinate_system.Angular_velocity=-Line_Angle_PID.Output;
	}
	else if(fabs(Status->Stop_angle)<0.001F)
	{
		Robot_Coordinate_system.Angular_velocity=-Robot_Coordinate_system.Angular_velocity;
	}
	

	//i逆时针旋转则速度取反
	if(Status->angle_rotate_Sum>0)
	{
		Robot_Coordinate_system.Angular_velocity*=-1;
	}
	//判断路径是否走完
	if(Angle_offset<0.5f)	
	{
		Status->angle_rotate_Sum=0;//清零，以后不再运行自转部分了
	}		

	/* 1 计算速度坐标系下速度大小 */
	/* 1.1计算速度坐标系下的切向速度 */
	// 1.1.1如果在加速区
	if((TargetLine_Coordinate_system.x) < Status->Speedup_Length)
	{
		Velocity_Coordinate_system.x=//期望速度
		SlowDown_VelocityCalculate(Status->Start_Speed,//初速度
								   TargetLine_Coordinate_system.x ,//距离
								   Status->Speedup_accelerated_speed);//加速度
	}
	// 1.1.2 如果没在减速区外，切向速度为正常速度
	else if((TargetLine_Coordinate_system.x) <= (Status->Target.x - Status->Slow_length))
	{
 		Velocity_Coordinate_system.x=Status->Max_Speed;
	}
	//1.1.3如果在减速区内
	else if((Status->Target.x - TargetLine_Coordinate_system.x) > Status->Stop_length &&(TargetLine_Coordinate_system.x+Status->Slow_length) >= Status->Target.x)
	{
		Velocity_Coordinate_system.x=
		SlowDown_VelocityCalculate(Status->Target_Speed,//初速度
								   fabs(-TargetLine_Coordinate_system.x + Status->Target.x),//距离
								   Status->Slow_accelerated_speed);//加速度
	}
	//1.1.4末尾速度为零，则需要在直线结尾精确定位
	else if(fabs(Status->Target_Speed)<0.001f&&Status->Stop_length!=0)
	{
		PID_Calculate(&Line_X_PID,TargetLine_Coordinate_system.x,Status->Target.x);
	  Velocity_Coordinate_system.x=Line_X_PID.Output;
	}
	//1.1.5末尾速度不为零，速度等于末端速度
	else if(fabs(Status->Stop_length)<0.001F)
	{
	 Velocity_Coordinate_system.x=Velocity_Coordinate_system.x;
	}
	/*1.1结束*/
	

	

	/* 1.2计算速度坐标系下的法向向速度 */
	//法向速度为PID调节
	PID_Calculate(&Line_Y_PID,TargetLine_Coordinate_system.y,Status->Target.y);	//目标线坐标系，所以不用调x
	Velocity_Coordinate_system.y=Line_Y_PID.Output;
	/* 2 将速度坐标系下的速度转化到车身坐标系*/
	//2.1首先转化到世界坐标系
	coordinitioate_transformation(Velocity_Coordinate_system,
								  &World_Coordinate_system_Velocity,
								  ZERO_Position,
								  &(Slope_angle));
	//陀螺仪左转为正，车身坐标系与世界坐标系的夹角与陀螺仪角度相反
	gyroIntegral.cos=(float)cos(-Action_Data.angle_Z*PI/180);
	gyroIntegral.sin=(float)sin(-Action_Data.angle_Z*PI/180);
	
	//2.2 从世界坐标系转化到车身坐标系
	coordinitioate_transformation(World_Coordinate_system_Velocity,
								  &Robot_Coordinate_system,
								  ZERO_Position,
								  &gyroIntegral);
	if(Angle_offset<0.5f||Status->angle_rotate_Sum==0)	//自转结束后再开启保持车身角度，且角度target已被赋值为Angle_Aim
	{													
	/* 3 保持车身角度 */
	PID_Calculate(&Line_Angle_PID,Action_Data.angle_Z,Status->Angle_Target);
	PID_Calculate(&Line_w_PID,Action_Data.w,Line_Angle_PID.Output);
	Robot_Coordinate_system.Angular_velocity=Line_w_PID.Output;
	}//角度和角速度双环PID


	//判断路径是否走完
	if(Status->Target.x - TargetLine_Coordinate_system.x<1)	
	{
		Status->Angle_Target=Angle_Aim;
		Work_Over=0;
	}
	
	return (Status->Target.x-TargetLine_Coordinate_system.x);
}

float Angle_sum=0;
extern float Angle_Test;
float arc_remain;//与目标点的距离差值
float Robot_Control_Circle(ROBOT_Status_Struct *Status)
{
	static Angle_Struvt Slope_angle;//速度坐标系与世界坐标系的夹角
	static float Angle_last;//机器人在目标线坐标系下走过的角度、上一时刻的角度
	static float angle_temp_1,angle_temp_2,angle_temp;//一些计算角度的临时变量
	
	/*如果是第一次进入走该路径的程序,需要计算一些基本参数，并将一些参数初始化*/
	if(Status->NewState==ENABLE)
	{
		Status->NewState=DISABLE;
		//将目标点的直角坐标，并将其映射到目标线坐标系上
		//此时目标线坐标线原点为圆心，X,Y轴分别于世界坐标系X,Y平行
		
//		Angle_Z_0=Action_Data.angle_Z;
		Angle_Z_0=Status->Angle_Target;
		
		Status->Target.x -=Status->Heart.x;
		Status->Target.y -=Status->Heart.y;//新老轴平行，画图，这是将世界系的目标点转为目标线系的目标点，return有用
		
		if(Status->Robot_type==Robot_LineType_Cycle_Clockwise)	Status->angle_reg_Sum*=-1;
		//计算减速区大小
		Status->Slow_length=SlowDown_PositionCalculate(Status->Max_Speed,
															 Status->Target_Speed,
															 Status->Slow_accelerated_speed);
    //计算加速区大小
		Status->Speedup_Length=SlowDown_PositionCalculate(Status->Start_Speed,
																Status->Max_Speed,
															    Status->Speedup_accelerated_speed);
		//PId误差归零
		Cir_Y_PID.Err=0;
		Cir_Y_PID.Last_Err=0;
		
		Cir_X_PID.Err=0;
		Cir_X_PID.Last_Err=0;
		//起始点计算
		TargetLine_Coordinate_system.x=Status->Start_positon.x-Status->Heart.x;
		TargetLine_Coordinate_system.y=Status->Start_positon.y-Status->Heart.y;

		//极坐标
		Polar_coordinates.Rho=sqrt(TargetLine_Coordinate_system.x*TargetLine_Coordinate_system.x
								  +TargetLine_Coordinate_system.y*TargetLine_Coordinate_system.y);
		Polar_coordinates.Theta.cos=TargetLine_Coordinate_system.x/Polar_coordinates.Rho;
		Polar_coordinates.Theta.sin=TargetLine_Coordinate_system.y/Polar_coordinates.Rho;
		Polar_coordinates.Theta.angle_reg=get_angle(Polar_coordinates.Theta.sin,Polar_coordinates.Theta.cos);
//		Polar_coordinates.Theta.angle_reg=get_polar_angle(Polar_coordinates.Theta.sin,Polar_coordinates.Theta.cos);
		
		Angle_last=Polar_coordinates.Theta.angle_reg;
		Angle_sum=0;
	}
	World_Coordinate_system_Position.x=Action_Data.x;
	World_Coordinate_system_Position.y=Action_Data.y;
	
	/* 0 计算在目标线坐标系下机器人的当前位置 */
	//0.1 坐标
	//0.1.1直角坐标
	TargetLine_Coordinate_system.x=World_Coordinate_system_Position.x-Status->Heart.x;
	TargetLine_Coordinate_system.y=World_Coordinate_system_Position.y-Status->Heart.y;
	//0.1.2极坐标
	Polar_coordinates.Rho=sqrt(TargetLine_Coordinate_system.x*TargetLine_Coordinate_system.x
							  +TargetLine_Coordinate_system.y*TargetLine_Coordinate_system.y);
	Polar_coordinates.Theta.cos=TargetLine_Coordinate_system.x/Polar_coordinates.Rho;
	Polar_coordinates.Theta.sin=TargetLine_Coordinate_system.y/Polar_coordinates.Rho;
	Polar_coordinates.Theta.angle_reg=get_angle(Polar_coordinates.Theta.sin,Polar_coordinates.Theta.cos);

	//0.1.3速度坐标系与世界坐标系的夹角计算
	Slope_angle.cos=-Polar_coordinates.Theta.sin;
	Slope_angle.sin=Polar_coordinates.Theta.cos;
	
	/* 1 计算速度坐标系下速度大小 */
	/* 1.1计算速度坐标系下的切向速度 */
	//1.1.0计算机器人与目标点的距离
	//首先计算与目标点的角度差值，计算方法借鉴了M3508无刷电机计算角度的方法
	if(Angle_last<Polar_coordinates.Theta.angle_reg)//一种情况
	{
		angle_temp_1=Polar_coordinates.Theta.angle_reg-Angle_last;//逆时针
		angle_temp_2=Polar_coordinates.Theta.angle_reg-Angle_last-2*PI;
	}
	else
	{
		angle_temp_1=Polar_coordinates.Theta.angle_reg-Angle_last;//顺时针
		angle_temp_2=2*PI+Polar_coordinates.Theta.angle_reg-Angle_last;
	}
	//无论顺时针转还是逆时针转，都是取小的那个角度
	angle_temp=(ABS(angle_temp_1))>(ABS(angle_temp_2)) ? angle_temp_2:angle_temp_1;
	Angle_sum+=angle_temp;
	Angle_last=Polar_coordinates.Theta.angle_reg;//当前所在位置极角变为last
	//再计算与目标点的距离差值?
	arc_remain=(Status->angle_reg_Sum - Angle_sum)*Status->R;
	if(Status->Robot_type==Robot_LineType_Cycle_Clockwise)	arc_remain*=-1;

	
	/* 1.1计算速度坐标系下的切向速度 */
	if(ABS(Angle_sum)*Status->R < Status->Speedup_Length)
	{
		Velocity_Coordinate_system.x=
		SlowDown_VelocityCalculate(Status->Start_Speed,//初速度
								   (Angle_sum)*Status->R ,//距离
									Status->Speedup_accelerated_speed);//加速度
	}
	// 1.1.1 如果没在减速区外，切向速度为正常速度
	else if(arc_remain >= Status->Slow_length)
	{
 		Velocity_Coordinate_system.x=Status->Max_Speed;
	}
	//1.1.2如果在减速区内
	else if(arc_remain >= Status->Stop_length)
	{
		Velocity_Coordinate_system.x=
		SlowDown_VelocityCalculate(Status->Target_Speed,//初速度
							       arc_remain,				  //距离
								   Status->Slow_accelerated_speed);//加速度
	}	
	//1.1.3末尾速度为零，则需要在直线结尾精确定位
	else if(fabs(Status->Target_Speed)<0.001f && Status->Stop_length!=0)
	{
		PID_Calculate(&Cir_X_PID,arc_remain,0);
	  Velocity_Coordinate_system.x=-Cir_X_PID.Output;
	}
	else if(fabs(Status->Stop_length)<0.001F)
	{
		Velocity_Coordinate_system.x=Velocity_Coordinate_system.x;
	}
	


	//i顺时针旋转则切向速度取反
	if(Status->Robot_type==Robot_LineType_Cycle_Clockwise)
	{
		Velocity_Coordinate_system.x*=-1;
	}
	
	if(yanqiexian)
	{	//他这的Angle——sum是有正负的
		if(	Status->Robot_type == Robot_LineType_Cycle_Clockwise)
		Status->Angle_Target=Angle_Z_0+Angle_sum*180/3.1415926f;//////////////////////////////////////////
		else Status->Angle_Target=Angle_Z_0+Angle_sum*180/3.1415926f;
	}
	/*1.2计算速度坐标系下的法向向速度 */
	//法向速度为PID调节
	PID_Calculate(&Cir_Y_PID,Polar_coordinates.Rho,Status->R);
	Velocity_Coordinate_system.y=-Cir_Y_PID.Output;
	
		/* 2 将速度坐标系下的速度转化到车身坐标系*/
	//2.1首先转化到世界坐标系
	coordinitioate_transformation(Velocity_Coordinate_system,
								  &World_Coordinate_system_Velocity,
								  ZERO_Position,
							 	  &(Slope_angle));
	//陀螺仪左转为正，车身坐标系与世界坐标系的夹角与陀螺仪角度相反
	gyroIntegral.cos=(float)cos(-Action_Data.angle_Z*PI/180);
	gyroIntegral.sin=(float)sin(-Action_Data.angle_Z*PI/180);
	//2.2 从世界坐标系转化到车身坐标系
	coordinitioate_transformation(World_Coordinate_system_Velocity,
								  &Robot_Coordinate_system,
								  ZERO_Position,
								  &gyroIntegral);														
																
		/* 3 保持车身角度 */
	PID_Calculate(&Cir_Angle_PID,Action_Data.angle_Z,Status->Angle_Target);	
	PID_Calculate(&Cir_W_PID,Action_Data.w,Cir_Angle_PID.Output);	
	Robot_Coordinate_system.Angular_velocity=Cir_W_PID.Output;
	
	//判断路径是否走完
	if(arc_remain<5)		
	{
		if(yanqiexian)
				Status->Angle_Target=Angle_Z_0+Status->angle_reg_Sum*180/3.1415926f;
		else
			Status->Angle_Target=Angle_Z_0;
		Work_Over=0;
	}
	
	return (Status->Target.x-TargetLine_Coordinate_system.x);//看不懂看第一句注释
}




//先Angle再X再Y
#define DEAD_LINE_XY    5
#define DEAD_LINE_ANGLE 0.5
#define DEAD_LINE_XY_TRIGGER 10
#define DEAD_LINE_ANGLE_TRIGGER 1
#define MAX_ADJUST_TIMES 2000


#if 0
void Keep_Robot_Position(float Angle,float X,float Y)
{	
	char Is_x_need_to_adjust = 1, Is_y_need_to_adjust = 1,Is_angle_need_to_adjust = 1;
	char j;
	long int i;
	//x和y上速度位置环，可以控制最大调整速度，还能减少静差
	j = 0;
	while(((Is_x_need_to_adjust == 1) || (Is_y_need_to_adjust == 1) || (Is_angle_need_to_adjust == 1)) && (j <= 4))
	{
		//判断是否需要调整
		if(ABS(Action_Data.x - X) <= DEAD_LINE_XY_TRIGGER)
			Is_x_need_to_adjust = 0;
		if(ABS(Action_Data.y - Y) <= DEAD_LINE_XY_TRIGGER)
			Is_y_need_to_adjust = 0;
		if(ABS(Action_Data.angle_Z - Angle) <= DEAD_LINE_ANGLE_TRIGGER)
			Is_angle_need_to_adjust = 0;
		//调整angle
		if(Is_angle_need_to_adjust == 1)
		{
			i = 0;
			Keep_PID_Clear(&Keep_W_PID);
			Keep_PID_Clear(&Keep_Angle_PID);
			while(((ABS(Action_Data.angle_Z - Angle) >= DEAD_LINE_ANGLE)|| (ABS(Keep_W_PID.Output) >= 40)) && (i <= MAX_ADJUST_TIMES))
			{
				PID_Calculate(&Keep_Angle_PID,Action_Data.angle_Z,Angle);	
				PID_Calculate(&Keep_W_PID,Action_Data.w,Keep_Angle_PID.Output);
				Robot_Coordinate_system.x=0;
				Robot_Coordinate_system.y=0;
				Robot_Coordinate_system.Angular_velocity=Keep_W_PID.Output;
				Robot_Wheel_Control();
				delay_ms(2);
				i ++;						
			}
		}
#if 1
		//调整x
		if(Is_x_need_to_adjust == 1)
		{
			i = 0;
			Keep_PID_Clear(&Keep_X_PID);
			while(((ABS(Action_Data.x - X) >= DEAD_LINE_XY)|| (ABS(Keep_X_PID.Output) >= 40)) && (i <= MAX_ADJUST_TIMES) )
			{
				PID_Calculate(&Keep_X_PID,Action_Data.x,X);
				Robot_Coordinate_system.x=Keep_X_PID.Output;
				Robot_Coordinate_system.y=0;
				Robot_Coordinate_system.Angular_velocity=0;
				Robot_Wheel_Control();
				delay_ms(2);
				i ++;			
			}
		}
		//调整y
		if(Is_y_need_to_adjust == 1)
		{
			i = 0;
			Keep_PID_Clear(&Keep_Y_PID);
			while(((ABS(Action_Data.y - Y) >= DEAD_LINE_XY)|| (ABS(Keep_Y_PID.Output) >= 40)) && (i <= MAX_ADJUST_TIMES) )
			{
				PID_Calculate(&Keep_Y_PID,Action_Data.y,Y);
				Robot_Coordinate_system.x=0;
				Robot_Coordinate_system.y=Keep_Y_PID.Output;
				Robot_Coordinate_system.Angular_velocity=0;
				Robot_Wheel_Control();
				delay_ms(2);
				i ++;			
			}
		}
#else //XY一起调，再调Angle
		//调整XY
		if((Is_x_need_to_adjust == 1) || (Is_y_need_to_adjust == 1))
		{
			if((Is_x_need_to_adjust == 1) && (Is_y_need_to_adjust == 1))
			{
				float adjust_angle = 0, adjust_line = 0;
				adjust_angle = atan((Action_Data.y - Y)/(Action_Data.x - X));
				adjust_line = sqrt(pow(Action_Data.y - Y,2)+pow(Action_Data.x - X,2));
			}
			else if((Is_x_need_to_adjust == 1) && (Is_y_need_to_adjust != 1))
			{
			
			}
			else if((Is_x_need_to_adjust != 1) && (Is_y_need_to_adjust == 1))
			{
			
			}
		}
#endif
		j ++;
	}
}
#else
void Keep_Robot_Position(float Angle,float X,float Y)
{
	Keep_PID_Clear(&Keep_X_PID);
	Keep_PID_Clear(&Keep_Y_PID);
	Keep_PID_Clear(&Keep_W_PID);
	Keep_PID_Clear(&Keep_Angle_PID);
	while((ABS(Action_Data.x-X)>=DEAD_LINE_XY_TRIGGER)||(ABS(Action_Data.y-Y)>=DEAD_LINE_XY_TRIGGER)||(ABS(Action_Data.angle_Z-Angle)>=DEAD_LINE_ANGLE_TRIGGER))
	{
		if(ABS(Action_Data.x-X)>=DEAD_LINE_XY)
		{
			PID_Calculate(&Keep_X_PID,Action_Data.x,X);
			Robot_Coordinate_system.x=Keep_X_PID.Output;			
		}
		else
			Robot_Coordinate_system.x=0;
		if(ABS(Action_Data.y-Y)>=DEAD_LINE_XY)
		{
			PID_Calculate(&Keep_Y_PID,Action_Data.y,Y);
			Robot_Coordinate_system.y=Keep_Y_PID.Output;	
		}
		else
			Robot_Coordinate_system.y=0;
		if(ABS(Action_Data.angle_Z-Angle)>=DEAD_LINE_ANGLE)
		{
			PID_Calculate(&Keep_Angle_PID,Action_Data.angle_Z,Angle);	
			PID_Calculate(&Keep_W_PID,Action_Data.w,Keep_Angle_PID.Output);
			Robot_Coordinate_system.Angular_velocity=Keep_W_PID.Output;			
		}
		else
			Robot_Coordinate_system.Angular_velocity=0;
		
		Robot_Wheel_Control();
		delay_ms(2);
	}
}

#endif

void Keep_PID_Clear(PID_TypeDef *pid)
{
	pid->Err = 0;
	pid->Last_Err = 0;
	pid->Last_Last_Err = 0;
	pid->Output = 0;
}


extern wheel_Struct  	wheel_rotation_v[5], wheel_final_v[5];
void Robot_Wait_for_Command(void)
{
	wheel_final_v[1].resultant_v= 0;
	wheel_final_v[2].resultant_v= 0;
	wheel_final_v[3].resultant_v= 0;
	wheel_final_v[4].resultant_v= 0;

	wheel_final_v[1].v_angle = -45;
	wheel_final_v[2].v_angle = 45;
//	wheel_final_v[1].v_angle = 0;
//	wheel_final_v[2].v_angle = 0;
	wheel_final_v[3].v_angle = 0;
	wheel_final_v[4].v_angle = 0;

	//获得轮子当前在【0,360)内的绝对角度
	gain_absolute_angle(&wheel_final_v[1],1);
	gain_absolute_angle(&wheel_final_v[2],2);
	gain_absolute_angle(&wheel_final_v[3],3);
	gain_absolute_angle(&wheel_final_v[4],4); 
	//将要转的角度wheel_final_v->angle_gap，化为 90 度内的，按不同情况合速度有的要取反方向
	gain_gap_angle(&wheel_final_v[1]);
	gain_gap_angle(&wheel_final_v[2]);   
	gain_gap_angle(&wheel_final_v[3]);
	gain_gap_angle(&wheel_final_v[4]);
	//累加积分得 轮子增量式的角度和	
	wheel_final_v[1].angle_sum += wheel_final_v[1].angle_gap; 
	wheel_final_v[2].angle_sum += wheel_final_v[2].angle_gap;
	wheel_final_v[3].angle_sum += wheel_final_v[3].angle_gap;
	wheel_final_v[4].angle_sum += wheel_final_v[4].angle_gap;	
}





