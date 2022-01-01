#include "wheel_control.h"
#include "robot_control.h"
#include "tdfhandkey.h"
#include "M3508.h"
#include "math.h"
#include "elmo.h"
#include "delay.h"


u8 Wheel_mode = 1;
float multiple_angle = 1802.24f;    //�Ƕ���X���
wheel_Struct  	wheel_rotation_v[5],        //������ת�ٶ�
								wheel_final_v[5];           //���������ٶ�
extern float get_angle(float sin_num,float cos_num);    //  [0,2PI]
extern TDF_struct tdf_handkey;
extern coordinitioate_Struct Robot_Coordinate_system;
extern M350x_STA M2006[8];

float ABS(float a)
{
	return a>0?a:-a;
}

void gain_absolute_angle(wheel_Struct *wheel_v,int id)
{
	switch(id)
	{
		case 1:    wheel_v->absolute_angle = wheel_v->angle_sum; //�ǶȲ�Ļ���  +    ����ֵĳ�ʼ�Ƕȣ������0����0������45����45
					break;
		case 2:    wheel_v->absolute_angle = wheel_v->angle_sum; //�ǶȲ�Ļ���
					break;
		case 3:    wheel_v->absolute_angle = wheel_v->angle_sum; //�ǶȲ�Ļ���
					break;
		case 4:    wheel_v->absolute_angle = wheel_v->angle_sum; //�ǶȲ�Ļ���
					break;
		default:	break;
	}

	while(wheel_v->absolute_angle>=360)
	wheel_v->absolute_angle -= 360;
	while(wheel_v->absolute_angle<0)
	wheel_v->absolute_angle += 360;          //��absolute_angle��Ϊ��0,360),
}

void gain_gap_angle(wheel_Struct *wheel_v)
{
	float speed;
	speed = wheel_v->resultant_v;
	wheel_v->angle_gap= wheel_v->v_angle - wheel_v->absolute_angle; //�ֽ�����Ҫת�ĽǶȣ�Ŀ���ٶȽǶȣ���ǰ���ֽǶȣ�������������[0,360),���Ϊ�������˳ʱ��ת		   
	if(wheel_v->angle_gap>270)
	{
		wheel_v->angle_gap = -(360-wheel_v->angle_gap);
		speed = speed;
	}
	else if(wheel_v->angle_gap>180)
	{
		wheel_v->angle_gap = -180+wheel_v->angle_gap;
		speed = -speed;
		//wheel_v->resultant_v = -wheel_v->resultant_v;
	}
	else if(wheel_v->angle_gap>90)
	{
		wheel_v->angle_gap = -(180-wheel_v->angle_gap);
		speed = -speed; 
		//wheel_v->resultant_v = -wheel_v->resultant_v;
	} 
	

	if(wheel_v->angle_gap<-270)
	{
		wheel_v->angle_gap = (360+wheel_v->angle_gap);
		speed = speed;
	}
	else if(wheel_v->angle_gap<-180)
	{
		wheel_v->angle_gap = (180+wheel_v->angle_gap);
		speed = -speed; 
		//wheel_v->resultant_v = -wheel_v->resultant_v;
	}

	else if(wheel_v->angle_gap<-90)
	{
		wheel_v->angle_gap = (180+wheel_v->angle_gap);
		speed = -speed;  
		//wheel_v->resultant_v = -wheel_v->resultant_v;
	}
	wheel_v->resultant_v = speed;
}
		 
void Robot_Wheel_Control(void)
{

	if(Wheel_mode  ==  vector_mode)
	{    
		//3.2��ת�ٶȷ��䵽ÿ������
		 wheel_rotation_v[1].x = -Robot_Coordinate_system.Angular_velocity * HASSIS_Struct_o1_length * cos(HASSIS_Struct_o1_angle);
		 wheel_rotation_v[1].y =  Robot_Coordinate_system.Angular_velocity * HASSIS_Struct_o1_length * sin(HASSIS_Struct_o1_angle);
		 wheel_rotation_v[2].x =  Robot_Coordinate_system.Angular_velocity * HASSIS_Struct_o2_length * cos(HASSIS_Struct_o2_angle);
		 wheel_rotation_v[2].y =  Robot_Coordinate_system.Angular_velocity * HASSIS_Struct_o2_length * sin(HASSIS_Struct_o2_angle);
		 wheel_rotation_v[3].x =  Robot_Coordinate_system.Angular_velocity * HASSIS_Struct_o3_length * cos(HASSIS_Struct_o3_angle);
		 wheel_rotation_v[3].y = -Robot_Coordinate_system.Angular_velocity * HASSIS_Struct_o3_length * sin(HASSIS_Struct_o3_angle);
		 wheel_rotation_v[4].x = -Robot_Coordinate_system.Angular_velocity * HASSIS_Struct_o4_length * cos(HASSIS_Struct_o4_angle);
		 wheel_rotation_v[4].y = -Robot_Coordinate_system.Angular_velocity * HASSIS_Struct_o4_length * sin(HASSIS_Struct_o4_angle);
		
		/*4  ���պ��ٶȷ���*/
		wheel_final_v[1].x  =  wheel_rotation_v[1].x+Robot_Coordinate_system.x;
		wheel_final_v[1].y  =  wheel_rotation_v[1].y+Robot_Coordinate_system.y;
		wheel_final_v[1].resultant_v  = sqrt(wheel_final_v[1].x*wheel_final_v[1].x+wheel_final_v[1].y*wheel_final_v[1].y);
		wheel_final_v[1].v_angle  =  get_angle(wheel_final_v[1].y/wheel_final_v[1].resultant_v,
										   wheel_final_v[1].x/wheel_final_v[1].resultant_v)   *180/PI;	//Ŀ���ٶȵĽǶ�
		
		wheel_final_v[2].x = wheel_rotation_v[2].x+Robot_Coordinate_system.x;
		wheel_final_v[2].y = wheel_rotation_v[2].y+Robot_Coordinate_system.y;  
		wheel_final_v[2].resultant_v  = sqrt(wheel_final_v[2].x*wheel_final_v[2].x+wheel_final_v[2].y*wheel_final_v[2].y);
		wheel_final_v[2].v_angle = get_angle(wheel_final_v[2].y/wheel_final_v[2].resultant_v,
										   wheel_final_v[2].x/wheel_final_v[2].resultant_v)   *180/PI;	//Ŀ���ٶȵĽǶ�
		
		wheel_final_v[3].x = wheel_rotation_v[3].x+Robot_Coordinate_system.x;
		wheel_final_v[3].y = wheel_rotation_v[3].y+Robot_Coordinate_system.y;  
		wheel_final_v[3].resultant_v  = sqrt(wheel_final_v[3].x*wheel_final_v[3].x+wheel_final_v[3].y*wheel_final_v[3].y) ;
		wheel_final_v[3].v_angle = get_angle(wheel_final_v[3].y/wheel_final_v[3].resultant_v,
										   wheel_final_v[3].x/wheel_final_v[3].resultant_v)   *180/PI;	//Ŀ���ٶȵĽǶ�
		
		wheel_final_v[4].x = wheel_rotation_v[4].x+Robot_Coordinate_system.x;
		wheel_final_v[4].y = wheel_rotation_v[4].y+Robot_Coordinate_system.y;  
		wheel_final_v[4].resultant_v  = sqrt(wheel_final_v[4].x*wheel_final_v[4].x+wheel_final_v[4].y*wheel_final_v[4].y) ;
		wheel_final_v[4].v_angle = get_angle(wheel_final_v[4].y /wheel_final_v[4].resultant_v,
											 wheel_final_v[4].x/wheel_final_v[4].resultant_v)  *180/PI;	//Ŀ���ٶȵĽǶ�
		//�����B���м䣬����ң�����ǶȺ�С���ó�����45����ת��̬��������
//		if( (tdf_handkey.Buttom_B == buttom1)&&
//			(wheel_final_v[1].resultant_v < 2)&&(wheel_final_v[2].resultant_v < 2)
//			&&(wheel_final_v[3].resultant_v < 2)&&(wheel_final_v[4].resultant_v < 2)  )   
//		{ 
//			wheel_final_v[1].v_angle = 135;
//			wheel_final_v[2].v_angle = 45;
//			wheel_final_v[3].v_angle = 315;
//			wheel_final_v[4].v_angle = 225;
//		}									 
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
		if( ! ((ABS(wheel_final_v[1].resultant_v) < 10)&&(ABS(wheel_final_v[2].resultant_v) < 10)   //��ֹkeep_positionʱƵ������
		     &&(ABS(wheel_final_v[3].resultant_v < 10))&&(ABS(wheel_final_v[4].resultant_v) < 10))) //�����ֵ��ٶȶ���С����angle_sum������һ�ε�λ�ò��� 
		{	
			wheel_final_v[1].angle_sum += wheel_final_v[1].angle_gap; 
			wheel_final_v[2].angle_sum += wheel_final_v[2].angle_gap;
			wheel_final_v[3].angle_sum += wheel_final_v[3].angle_gap;
			wheel_final_v[4].angle_sum += wheel_final_v[4].angle_gap;
		}
	   
		TIM_Cmd(TIM2,ENABLE);
		//���ٶȷ����4������
//		Elmo_Set_Speed((s32)wheel_final_v[1].resultant_v,0X301);
//		delay_ms(1);
//		Elmo_Set_Speed((s32)wheel_final_v[2].resultant_v,0X302);
//		delay_ms(1);
//		Elmo_Set_Speed((s32)wheel_final_v[3].resultant_v,0X303);
//		delay_ms(1);
//		Elmo_Set_Speed((s32)wheel_final_v[4].resultant_v,0X304);
//		delay_ms(1);
//	  
//		Elmo_Set_Begin(0X301);
//		delay_ms(1);
//		Elmo_Set_Begin(0X302);
//		delay_ms(1);
//		Elmo_Set_Begin(0X303);
//		delay_ms(1);
//		Elmo_Set_Begin(0X304); 
//		delay_ms(1);
	}
	if(Wheel_mode  ==  3)  //
	{
		
	}
	if(Wheel_mode  ==  4)  //���ֽǶ� ,û��x��y�����ϵģ�
	{ 

		wheel_final_v[1].resultant_v= 0;
		wheel_final_v[2].resultant_v= 0;
		wheel_final_v[3].resultant_v= 0;
		wheel_final_v[4].resultant_v= 0;

			wheel_final_v[1].v_angle = -45;
			wheel_final_v[2].v_angle = 45;
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
		
		//���ٶȷ����4������
//		Elmo_Set_Speed((s32)wheel_final_v[1].resultant_v,0X301);
//		delay_ms(1);
//		Elmo_Set_Speed((s32)wheel_final_v[2].resultant_v,0X302);
//		delay_ms(1);
//		Elmo_Set_Speed((s32)wheel_final_v[3].resultant_v,0X303);
//		delay_ms(1);
//		Elmo_Set_Speed((s32)wheel_final_v[4].resultant_v,0X304);
//		delay_ms(1);
//	  
//		Elmo_Set_Begin(0X301);
//		delay_ms(1);
//		Elmo_Set_Begin(0X302);
//		delay_ms(1);
//		Elmo_Set_Begin(0X303);
//		delay_ms(1);
//		Elmo_Set_Begin(0X304); 
//		delay_ms(1);
		
	}
	if(Wheel_mode  ==  2)
	{   //�ػ���λʱʹ�ã�ǿ�����㣬���ֻص���ʼλ��
	  wheel_final_v[1].angle_sum=0;
		wheel_final_v[2].angle_sum=0;
		wheel_final_v[3].angle_sum=0;
	  wheel_final_v[4].angle_sum=0;
		
//		Elmo_Set_Speed(0,0X301);
//		delay_ms(1);
//		Elmo_Set_Speed(0,0X302);
//		delay_ms(1);
//		Elmo_Set_Speed(0,0X303);
//		delay_ms(1);
//		Elmo_Set_Speed(0,0X304);
//		delay_ms(1);
//		
//		Elmo_Set_Begin(0X301);
//		delay_ms(1);
//		Elmo_Set_Begin(0X302);
//		delay_ms(1);
//		Elmo_Set_Begin(0X303);
//		delay_ms(1);
//		Elmo_Set_Begin(0X304);
		
	}
}

void Robot_Wheel_Control_3508(void)
{
	
		M3510_Position_S(M2006  ,(s32)(multiple_angle*wheel_final_v[1].angle_sum));
		M3510_Position_S(M2006+1,(s32)(multiple_angle*wheel_final_v[2].angle_sum));
		M3510_Position_S(M2006+2,(s32)(multiple_angle*wheel_final_v[3].angle_sum));
		M3510_Position_S(M2006+3,(s32)(multiple_angle*wheel_final_v[4].angle_sum));     
		CAN2_SetMotor_0_3(M2006);
}

