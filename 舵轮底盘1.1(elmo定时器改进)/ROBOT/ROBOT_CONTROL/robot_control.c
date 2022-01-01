#include "robot_control.h"
#include "wheel_control.h"
#include "math.h"
#include "usart.h"
#include "m3508.h"
#include "pid.h"
#include "mapan.h"
#include "delay.h"
#include "timer.h"

//��λͳһ    (δ���)
//�ٶ�v   	��λ��mm/s
//·��s   	��λ��mm
//���ٶ�a 	��λ��mm/s2
//�ǶȦ�     ��λ����   (��) 
//���ٶ�w 	��λ��rad/s (����ÿ��)
//�Ǽ��ٶȦ� ��λ��rad/s2(����ÿ���η���)

ROBOT_Status_Struct ROBOT_Status;
ROBOT_Status_Struct ROBOT_Wheel;
u8 Work_Over;
int yanqiexian;//������

Polar_coordinates_Struct Polar_coordinates;		//��������
coordinitioate_Struct  World_Coordinate_system_Position	/*��������ϵ--λ��*/
					  ,World_Coordinate_system_Velocity	/*��������ϵ--�ٶ�*/
					  ,Velocity_Coordinate_system		/*�ٶ�����ϵ*/
					  ,Robot_Coordinate_system			/*�����˳�������ϵ*/
					  ,TargetLine_Coordinate_system		/*Ŀ��������ϵ���൱�����ٶ�����ϵ�µ�λ��*/
					  ,ZERO_Position={.x=0,.y=0};		/*ԭ��*/

Angle_Struvt 		   gyroIntegral						/*��������ϵ*/
					  ,SlopeAngle_LINE;					/*Ŀ��������ϵ*/

extern Action_data Action_Data;

static float ABS(float a)
{
	return a>0?a:-a;
}

//����ת������ 
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

//�����������Ƕȴ�С
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
//sin��cos��Ƕ�
float get_angle(float sin_num,float cos_num)
{//������
	if(sin_num>=0&&cos_num>=0)//��һ���޵�
		return asin(sin_num);
	else if(sin_num>=0&&cos_num<=0)//�ڶ�����
		return PI-asin(sin_num);
	else if(sin_num<=0&&cos_num<=0)//��������
		return PI-asin(sin_num);
	else if(sin_num<=0&&cos_num>=0)//��������
		return 2*PI+asin(sin_num);
	return 0;
}


/*����ټ��ټ���--����ĩ�ٶ�*/
static float SlowDown_VelocityCalculate(float V_0 ,float X,float a)
{
	float temp;
	temp=2*a*X+V_0*V_0;
	return sqrt(temp>0?temp:-temp);
}
/*����ټ��ټ���--�������*/
static float SlowDown_PositionCalculate(float V_0 ,float V,float a)
{
	float temp;
	temp=(V*V-V_0*V_0)/2/a;
	return temp>0?temp:-temp;
}

//�Ƕ�ȡ��
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



float Angle_offset;//ʣ��Ҫת�ĽǶ�
float Angle_Z_0;//��ʼ�����
float Angle_Aim;//Ŀ��Ƕ�ֵ
float X0,Y0;//��ʼλ��

/////////////////////////////////////////////////////////////////////////////////////////////////////////////ֱ��
float Robot_Control_Line(ROBOT_Status_Struct *Status)
{
	static Angle_Struvt Slope_angle;	//�ٶ�����ϵ����������ϵ�ļн�
	/*����ǵ�һ�ν����߸�·���ĳ���,��Ҫ����һЩ��������������һЩ������ʼ��*/
	if(Status->NewState==ENABLE)
	{
		Status->NewState=DISABLE;
		//Ŀ�����ߵĽǶ�
		Angle_Calculate(&(SlopeAngle_LINE),	Status->Start_positon, Status->Target_position);
		Angle_invert(&(SlopeAngle_LINE));
		//�ٶ�����ϵ����������ϵ�ļнǼ��㣬�Ƕȴ�С��Ŀ�����ߵĽǶȻ�Ϊ�෴��
		Angle_Calculate(&(Slope_angle), Status->Start_positon, Status->Target_position);
		//��Ŀ�����������������ϵ��ת����Ŀ��������ϵ������֮�����
 		coordinitioate_transformation(Status->Target_position,
									  &Status->Target, 
									  Status->Start_positon, 
									  &SlopeAngle_LINE);
		//�����������С
		Status->Slow_length=SlowDown_PositionCalculate(Status->Max_Speed,
															 Status->Target_Speed,
															 Status->Slow_accelerated_speed);
		//�����������С
		Status->Speedup_Length=SlowDown_PositionCalculate(Status->Start_Speed,
																Status->Max_Speed,
																Status->Speedup_accelerated_speed);
		Angle_Z_0=Status->Angle_Target;		
		
		//��������ת���ּ���
		Angle_Aim=Angle_Z_0+Status->angle_rotate_Sum;
		//�����������С
			
		Status->Slow_angle=SlowDown_PositionCalculate(Status->Max_w_Speed,
															 Status->Target_w_Speed,
															 Status->Slow_accelerated_w_speed);
		//�����������С
		Status->Speedup_angle=SlowDown_PositionCalculate(Status->Start_w_Speed,
										  					Status->Max_w_Speed,
																Status->Speedup_accelerated_w_speed);
		Status->Angle_Target=Angle_Aim;
		
	}
	World_Coordinate_system_Position.x=Action_Data.x;
	World_Coordinate_system_Position.y=Action_Data.y;
	/* 0 ������Ŀ��������ϵ�»����˵ĵ�ǰ���� */
	coordinitioate_transformation(World_Coordinate_system_Position,
								  &TargetLine_Coordinate_system,
								  Status->Start_positon,
								  &SlopeAngle_LINE);
	
	//������ٶ���ÿ�������ֵ
	//���½��ٶȼ��㷽������ֱ�����ߣ�ע����ʱδ�ģ�������ʱ��Ӣ����Ϊ��
	//������Ŀ��ǶȲ�ֵ
	Angle_offset=(Angle_Aim - Action_Data.angle_Z);
	if(Status->angle_rotate_Sum<0)	Angle_offset*=-1;//˵����˳ʱ�룬���ֽǶȲ�ֵ�Ǻ�����

	/* 1.1�����ٶ�����ϵ�µ������ٶ� */
	if(ABS(Action_Data.angle_Z-Angle_Z_0 )< Status->Speedup_angle)
	{
		Robot_Coordinate_system.Angular_velocity=
		-SlowDown_VelocityCalculate(Status->Start_w_Speed,//���ٶ�
									 Action_Data.angle_Z-Angle_Z_0 ,//����
									Status->Speedup_accelerated_w_speed);//���ٶ�
	}
	// 1.1.1 ���û�ڼ������⣬�����ٶ�Ϊ�����ٶ�
	else if(Angle_offset >= Status->Slow_angle)
	{
		Robot_Coordinate_system.Angular_velocity=-Status->Max_w_Speed;
	}
	//1.1.2����ڼ�������ֹͣ����
	else if(Angle_offset >= Status->Stop_angle)
	{
		Robot_Coordinate_system.Angular_velocity=
		-SlowDown_VelocityCalculate(Status->Target_w_Speed,//���ٶ�
									 Angle_offset,				  //����
									 Status->Slow_accelerated_w_speed);//���ٶ�
	}	
	//1.1.3ĩβ�ٶ�Ϊ�㣬����Ҫ��ֱ�߽�β��ȷ��λ
	else if(fabs(Status->Target_w_Speed)<0.001f && Status->Stop_angle!=0)
	{
		PID_Calculate(&Line_Angle_PID,Angle_offset,0);
		Robot_Coordinate_system.Angular_velocity=-Line_Angle_PID.Output;
	}
	else if(fabs(Status->Stop_angle)<0.001F)
	{
		Robot_Coordinate_system.Angular_velocity=-Robot_Coordinate_system.Angular_velocity;
	}
	

	//i��ʱ����ת���ٶ�ȡ��
	if(Status->angle_rotate_Sum>0)
	{
		Robot_Coordinate_system.Angular_velocity*=-1;
	}
	//�ж�·���Ƿ�����
	if(Angle_offset<0.5f)	
	{
		Status->angle_rotate_Sum=0;//���㣬�Ժ���������ת������
	}		

	/* 1 �����ٶ�����ϵ���ٶȴ�С */
	/* 1.1�����ٶ�����ϵ�µ������ٶ� */
	// 1.1.1����ڼ�����
	if((TargetLine_Coordinate_system.x) < Status->Speedup_Length)
	{
		Velocity_Coordinate_system.x=//�����ٶ�
		SlowDown_VelocityCalculate(Status->Start_Speed,//���ٶ�
								   TargetLine_Coordinate_system.x ,//����
								   Status->Speedup_accelerated_speed);//���ٶ�
	}
	// 1.1.2 ���û�ڼ������⣬�����ٶ�Ϊ�����ٶ�
	else if((TargetLine_Coordinate_system.x) <= (Status->Target.x - Status->Slow_length))
	{
 		Velocity_Coordinate_system.x=Status->Max_Speed;
	}
	//1.1.3����ڼ�������
	else if((Status->Target.x - TargetLine_Coordinate_system.x) > Status->Stop_length &&(TargetLine_Coordinate_system.x+Status->Slow_length) >= Status->Target.x)
	{
		Velocity_Coordinate_system.x=
		SlowDown_VelocityCalculate(Status->Target_Speed,//���ٶ�
								   fabs(-TargetLine_Coordinate_system.x + Status->Target.x),//����
								   Status->Slow_accelerated_speed);//���ٶ�
	}
	//1.1.4ĩβ�ٶ�Ϊ�㣬����Ҫ��ֱ�߽�β��ȷ��λ
	else if(fabs(Status->Target_Speed)<0.001f&&Status->Stop_length!=0)
	{
		PID_Calculate(&Line_X_PID,TargetLine_Coordinate_system.x,Status->Target.x);
	  Velocity_Coordinate_system.x=Line_X_PID.Output;
	}
	//1.1.5ĩβ�ٶȲ�Ϊ�㣬�ٶȵ���ĩ���ٶ�
	else if(fabs(Status->Stop_length)<0.001F)
	{
	 Velocity_Coordinate_system.x=Velocity_Coordinate_system.x;
	}
	/*1.1����*/
	

	

	/* 1.2�����ٶ�����ϵ�µķ������ٶ� */
	//�����ٶ�ΪPID����
	PID_Calculate(&Line_Y_PID,TargetLine_Coordinate_system.y,Status->Target.y);	//Ŀ��������ϵ�����Բ��õ�x
	Velocity_Coordinate_system.y=Line_Y_PID.Output;
	/* 2 ���ٶ�����ϵ�µ��ٶ�ת������������ϵ*/
	//2.1����ת������������ϵ
	coordinitioate_transformation(Velocity_Coordinate_system,
								  &World_Coordinate_system_Velocity,
								  ZERO_Position,
								  &(Slope_angle));
	//��������תΪ������������ϵ����������ϵ�ļн��������ǽǶ��෴
	gyroIntegral.cos=(float)cos(-Action_Data.angle_Z*PI/180);
	gyroIntegral.sin=(float)sin(-Action_Data.angle_Z*PI/180);
	
	//2.2 ����������ϵת������������ϵ
	coordinitioate_transformation(World_Coordinate_system_Velocity,
								  &Robot_Coordinate_system,
								  ZERO_Position,
								  &gyroIntegral);
	if(Angle_offset<0.5f||Status->angle_rotate_Sum==0)	//��ת�������ٿ������ֳ���Ƕȣ��ҽǶ�target�ѱ���ֵΪAngle_Aim
	{													
	/* 3 ���ֳ���Ƕ� */
	PID_Calculate(&Line_Angle_PID,Action_Data.angle_Z,Status->Angle_Target);
	PID_Calculate(&Line_w_PID,Action_Data.w,Line_Angle_PID.Output);
	Robot_Coordinate_system.Angular_velocity=Line_w_PID.Output;
	}//�ǶȺͽ��ٶ�˫��PID


	//�ж�·���Ƿ�����
	if(Status->Target.x - TargetLine_Coordinate_system.x<1)	
	{
		Status->Angle_Target=Angle_Aim;
		Work_Over=0;
	}
	
	return (Status->Target.x-TargetLine_Coordinate_system.x);
}

float Angle_sum=0;
extern float Angle_Test;
float arc_remain;//��Ŀ���ľ����ֵ
float Robot_Control_Circle(ROBOT_Status_Struct *Status)
{
	static Angle_Struvt Slope_angle;//�ٶ�����ϵ����������ϵ�ļн�
	static float Angle_last;//��������Ŀ��������ϵ���߹��ĽǶȡ���һʱ�̵ĽǶ�
	static float angle_temp_1,angle_temp_2,angle_temp;//һЩ����Ƕȵ���ʱ����
	
	/*����ǵ�һ�ν����߸�·���ĳ���,��Ҫ����һЩ��������������һЩ������ʼ��*/
	if(Status->NewState==ENABLE)
	{
		Status->NewState=DISABLE;
		//��Ŀ����ֱ�����꣬������ӳ�䵽Ŀ��������ϵ��
		//��ʱĿ����������ԭ��ΪԲ�ģ�X,Y��ֱ�����������ϵX,Yƽ��
		
//		Angle_Z_0=Action_Data.angle_Z;
		Angle_Z_0=Status->Angle_Target;
		
		Status->Target.x -=Status->Heart.x;
		Status->Target.y -=Status->Heart.y;//������ƽ�У���ͼ�����ǽ�����ϵ��Ŀ���תΪĿ����ϵ��Ŀ��㣬return����
		
		if(Status->Robot_type==Robot_LineType_Cycle_Clockwise)	Status->angle_reg_Sum*=-1;
		//�����������С
		Status->Slow_length=SlowDown_PositionCalculate(Status->Max_Speed,
															 Status->Target_Speed,
															 Status->Slow_accelerated_speed);
    //�����������С
		Status->Speedup_Length=SlowDown_PositionCalculate(Status->Start_Speed,
																Status->Max_Speed,
															    Status->Speedup_accelerated_speed);
		//PId������
		Cir_Y_PID.Err=0;
		Cir_Y_PID.Last_Err=0;
		
		Cir_X_PID.Err=0;
		Cir_X_PID.Last_Err=0;
		//��ʼ�����
		TargetLine_Coordinate_system.x=Status->Start_positon.x-Status->Heart.x;
		TargetLine_Coordinate_system.y=Status->Start_positon.y-Status->Heart.y;

		//������
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
	
	/* 0 ������Ŀ��������ϵ�»����˵ĵ�ǰλ�� */
	//0.1 ����
	//0.1.1ֱ������
	TargetLine_Coordinate_system.x=World_Coordinate_system_Position.x-Status->Heart.x;
	TargetLine_Coordinate_system.y=World_Coordinate_system_Position.y-Status->Heart.y;
	//0.1.2������
	Polar_coordinates.Rho=sqrt(TargetLine_Coordinate_system.x*TargetLine_Coordinate_system.x
							  +TargetLine_Coordinate_system.y*TargetLine_Coordinate_system.y);
	Polar_coordinates.Theta.cos=TargetLine_Coordinate_system.x/Polar_coordinates.Rho;
	Polar_coordinates.Theta.sin=TargetLine_Coordinate_system.y/Polar_coordinates.Rho;
	Polar_coordinates.Theta.angle_reg=get_angle(Polar_coordinates.Theta.sin,Polar_coordinates.Theta.cos);

	//0.1.3�ٶ�����ϵ����������ϵ�ļнǼ���
	Slope_angle.cos=-Polar_coordinates.Theta.sin;
	Slope_angle.sin=Polar_coordinates.Theta.cos;
	
	/* 1 �����ٶ�����ϵ���ٶȴ�С */
	/* 1.1�����ٶ�����ϵ�µ������ٶ� */
	//1.1.0�����������Ŀ���ľ���
	//���ȼ�����Ŀ���ĽǶȲ�ֵ�����㷽�������M3508��ˢ�������Ƕȵķ���
	if(Angle_last<Polar_coordinates.Theta.angle_reg)//һ�����
	{
		angle_temp_1=Polar_coordinates.Theta.angle_reg-Angle_last;//��ʱ��
		angle_temp_2=Polar_coordinates.Theta.angle_reg-Angle_last-2*PI;
	}
	else
	{
		angle_temp_1=Polar_coordinates.Theta.angle_reg-Angle_last;//˳ʱ��
		angle_temp_2=2*PI+Polar_coordinates.Theta.angle_reg-Angle_last;
	}
	//����˳ʱ��ת������ʱ��ת������ȡС���Ǹ��Ƕ�
	angle_temp=(ABS(angle_temp_1))>(ABS(angle_temp_2)) ? angle_temp_2:angle_temp_1;
	Angle_sum+=angle_temp;
	Angle_last=Polar_coordinates.Theta.angle_reg;//��ǰ����λ�ü��Ǳ�Ϊlast
	//�ټ�����Ŀ���ľ����ֵ?
	arc_remain=(Status->angle_reg_Sum - Angle_sum)*Status->R;
	if(Status->Robot_type==Robot_LineType_Cycle_Clockwise)	arc_remain*=-1;

	
	/* 1.1�����ٶ�����ϵ�µ������ٶ� */
	if(ABS(Angle_sum)*Status->R < Status->Speedup_Length)
	{
		Velocity_Coordinate_system.x=
		SlowDown_VelocityCalculate(Status->Start_Speed,//���ٶ�
								   (Angle_sum)*Status->R ,//����
									Status->Speedup_accelerated_speed);//���ٶ�
	}
	// 1.1.1 ���û�ڼ������⣬�����ٶ�Ϊ�����ٶ�
	else if(arc_remain >= Status->Slow_length)
	{
 		Velocity_Coordinate_system.x=Status->Max_Speed;
	}
	//1.1.2����ڼ�������
	else if(arc_remain >= Status->Stop_length)
	{
		Velocity_Coordinate_system.x=
		SlowDown_VelocityCalculate(Status->Target_Speed,//���ٶ�
							       arc_remain,				  //����
								   Status->Slow_accelerated_speed);//���ٶ�
	}	
	//1.1.3ĩβ�ٶ�Ϊ�㣬����Ҫ��ֱ�߽�β��ȷ��λ
	else if(fabs(Status->Target_Speed)<0.001f && Status->Stop_length!=0)
	{
		PID_Calculate(&Cir_X_PID,arc_remain,0);
	  Velocity_Coordinate_system.x=-Cir_X_PID.Output;
	}
	else if(fabs(Status->Stop_length)<0.001F)
	{
		Velocity_Coordinate_system.x=Velocity_Coordinate_system.x;
	}
	


	//i˳ʱ����ת�������ٶ�ȡ��
	if(Status->Robot_type==Robot_LineType_Cycle_Clockwise)
	{
		Velocity_Coordinate_system.x*=-1;
	}
	
	if(yanqiexian)
	{	//�����Angle����sum����������
		if(	Status->Robot_type == Robot_LineType_Cycle_Clockwise)
		Status->Angle_Target=Angle_Z_0+Angle_sum*180/3.1415926f;//////////////////////////////////////////
		else Status->Angle_Target=Angle_Z_0+Angle_sum*180/3.1415926f;
	}
	/*1.2�����ٶ�����ϵ�µķ������ٶ� */
	//�����ٶ�ΪPID����
	PID_Calculate(&Cir_Y_PID,Polar_coordinates.Rho,Status->R);
	Velocity_Coordinate_system.y=-Cir_Y_PID.Output;
	
		/* 2 ���ٶ�����ϵ�µ��ٶ�ת������������ϵ*/
	//2.1����ת������������ϵ
	coordinitioate_transformation(Velocity_Coordinate_system,
								  &World_Coordinate_system_Velocity,
								  ZERO_Position,
							 	  &(Slope_angle));
	//��������תΪ������������ϵ����������ϵ�ļн��������ǽǶ��෴
	gyroIntegral.cos=(float)cos(-Action_Data.angle_Z*PI/180);
	gyroIntegral.sin=(float)sin(-Action_Data.angle_Z*PI/180);
	//2.2 ����������ϵת������������ϵ
	coordinitioate_transformation(World_Coordinate_system_Velocity,
								  &Robot_Coordinate_system,
								  ZERO_Position,
								  &gyroIntegral);														
																
		/* 3 ���ֳ���Ƕ� */
	PID_Calculate(&Cir_Angle_PID,Action_Data.angle_Z,Status->Angle_Target);	
	PID_Calculate(&Cir_W_PID,Action_Data.w,Cir_Angle_PID.Output);	
	Robot_Coordinate_system.Angular_velocity=Cir_W_PID.Output;
	
	//�ж�·���Ƿ�����
	if(arc_remain<5)		
	{
		if(yanqiexian)
				Status->Angle_Target=Angle_Z_0+Status->angle_reg_Sum*180/3.1415926f;
		else
			Status->Angle_Target=Angle_Z_0;
		Work_Over=0;
	}
	
	return (Status->Target.x-TargetLine_Coordinate_system.x);//����������һ��ע��
}




//��Angle��X��Y
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
	//x��y���ٶ�λ�û������Կ����������ٶȣ����ܼ��پ���
	j = 0;
	while(((Is_x_need_to_adjust == 1) || (Is_y_need_to_adjust == 1) || (Is_angle_need_to_adjust == 1)) && (j <= 4))
	{
		//�ж��Ƿ���Ҫ����
		if(ABS(Action_Data.x - X) <= DEAD_LINE_XY_TRIGGER)
			Is_x_need_to_adjust = 0;
		if(ABS(Action_Data.y - Y) <= DEAD_LINE_XY_TRIGGER)
			Is_y_need_to_adjust = 0;
		if(ABS(Action_Data.angle_Z - Angle) <= DEAD_LINE_ANGLE_TRIGGER)
			Is_angle_need_to_adjust = 0;
		//����angle
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
		//����x
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
		//����y
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
#else //XYһ������ٵ�Angle
		//����XY
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

	//������ӵ�ǰ�ڡ�0,360)�ڵľ��ԽǶ�
	gain_absolute_angle(&wheel_final_v[1],1);
	gain_absolute_angle(&wheel_final_v[2],2);
	gain_absolute_angle(&wheel_final_v[3],3);
	gain_absolute_angle(&wheel_final_v[4],4); 
	//��Ҫת�ĽǶ�wheel_final_v->angle_gap����Ϊ 90 ���ڵģ�����ͬ������ٶ��е�Ҫȡ������
	gain_gap_angle(&wheel_final_v[1]);
	gain_gap_angle(&wheel_final_v[2]);   
	gain_gap_angle(&wheel_final_v[3]);
	gain_gap_angle(&wheel_final_v[4]);
	//�ۼӻ��ֵ� ��������ʽ�ĽǶȺ�	
	wheel_final_v[1].angle_sum += wheel_final_v[1].angle_gap; 
	wheel_final_v[2].angle_sum += wheel_final_v[2].angle_gap;
	wheel_final_v[3].angle_sum += wheel_final_v[3].angle_gap;
	wheel_final_v[4].angle_sum += wheel_final_v[4].angle_gap;	
}





