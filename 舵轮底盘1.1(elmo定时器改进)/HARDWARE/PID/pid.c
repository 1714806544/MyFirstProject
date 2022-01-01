#include "pid.h"
#include "math.h"
#include "m3508.h"
extern M350x_STA M2006[8];
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


#if IS_PID_MORE_FUNCTION == 1

void PID_Calculate (PID_TypeDef *pid,float measure,float target)
{	
	//误差更新
	pid->Measure = measure;
  pid->Target = target;
  pid->Err = pid->Target - pid->Measure;
	
	pid->Output += pid->Kp * (pid->Err - pid->Last_Err)
							 + pid->Ki * 	pid->Err
							 + pid->Kd * (pid->Err - 2 * pid->Last_Err + pid->Last_Last_Err);
	pid->Last_Last_Err = pid->Last_Err;
	pid->Last_Err = pid->Err;
}
							
void PID_Init(PID_TypeDef *pid , float kp, float Ki, float Kd)
{
	pid->Kp = kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
}

void All_PID_Init(void)
{
	//M2006 速度位置环PID
	PID_Init(&M2006[0].Position_S_PID_STruct, 10, 0.5, 2);
	PID_Init(&M2006[1].Position_S_PID_STruct, 10, 0.5, 2);
	PID_Init(&M2006[2].Position_S_PID_STruct, 10, 0.5, 2);
	PID_Init(&M2006[3].Position_S_PID_STruct, 10, 0.5, 2);
	PID_Init(&M2006[0].Position_P_PID_STruct, 1, 0.01, 0);
	PID_Init(&M2006[1].Position_P_PID_STruct, 1, 0.01, 0);
	PID_Init(&M2006[2].Position_P_PID_STruct, 1, 0.01, 0);
	PID_Init(&M2006[3].Position_P_PID_STruct, 1, 0.01, 0);
	
	//路径PID
	PID_Init(&Keep_X_PID, 1.5, 0, 0);// 15, 0.02, 10
	PID_Init(&Keep_Y_PID, 1.5, 0, 0);// 15, 0.02, 10
	PID_Init(&Keep_W_PID, 1.5, 0, 0);// 3.5, 0.1, 0
	PID_Init(&Keep_Angle_PID, 2, 0, 5);// 2, 0, 5
	
	PID_Init(&Line_X_PID, 1, 0, 0);
	PID_Init(&Line_Y_PID, 1, 0, 0);
	PID_Init(&Line_w_PID, 1, 0, 0);
	PID_Init(&Line_Angle_PID, 1, 0, 0);
	
	PID_Init(&Cir_X_PID, 1, 0, 0);
	PID_Init(&Cir_Y_PID, 1, 0, 0);
	PID_Init(&Cir_W_PID, 1, 0, 0);
	PID_Init(&Cir_Angle_PID, 1, 0, 0);
}

#endif


#if IS_PID_MORE_FUNCTION == 2

float Abs(float a)
{
	if(a>0) return a;
	else return -a;
}

void PID_Calculate (PID_TypeDef *pid,float measure,float target)
{
	//误差更新
  pid->Measure = measure;
  pid->Target = target;
  pid->Err = pid->Target - pid->Measure;

	if(Abs(pid->Err)>pid->DeadBand)
	{
		pid->Pout=pid->Kp*pid->Err;
		
		PID_Function_Selection(pid);
		PID_Integral_Limit(pid);
		
		pid->Iout += pid->ITerm;
		pid->Output = pid->Pout+pid->Iout+pid->Dout;

	if(pid->Output>=pid->MaxOut)  pid->Output=pid->MaxOut;
	if(pid->Output<=-pid->MaxOut) pid->Output=-pid->MaxOut;
	}
	else
	{
		pid->Output=0;
	}
	
	pid->Last_Measure=pid->Measure;
	pid->Last_Err=pid->Err;
	pid->Last_Output=pid->Output;
}

void PID_Integral_Limit(PID_TypeDef *pid)
{
  float temp_Output, temp_Iout;
  temp_Iout = pid->Iout + pid->ITerm;
  temp_Output = pid->Pout + pid->Iout + pid->Dout;
	
	if(Abs(temp_Output) > pid->MaxOut)
	{
		if (pid->Err * pid->Iout > 0)
			pid->ITerm = 0;
	}
	
  if (temp_Iout > pid->IntegralLimit)
	{
			pid->ITerm = 0;
			pid->Iout = pid->IntegralLimit;
	}
	if (temp_Iout < -pid->IntegralLimit)
	{
			pid->ITerm = 0;
			pid->Iout = -pid->IntegralLimit;
	}
}

void PID_Function_Selection(PID_TypeDef *pid)
{
//梯形积分
	if(pid->Function_Mode&0X01)
		pid->ITerm=pid->Ki*((pid->Err+pid->Last_Err)/2);
	else
		pid->ITerm=pid->Ki*pid->Err;
	
//微分先行
	if(pid->Function_Mode&0X02)
		pid->Dout=-pid->Kd*(pid->Measure-pid->Last_Measure);
	else
		pid->Dout=pid->Kd*(pid->Err-pid->Last_Err);
	
//积分分离
	if(pid->Function_Mode&0X04)
		if(pid->Err*pid->Iout>0)
		{
			if(Abs(pid->Err)<=pid->Max_Err)
				pid->ITerm=pid->ITerm;
			else
				pid->ITerm=0;
		}
		
//变速积分
	if(pid->Function_Mode&0X08)
		if(pid->Err*pid->Iout>0)
		{
			if(Abs(pid->Err)<=pid->ScalarB)
				pid->ITerm=pid->ITerm;
			else if(Abs(pid->Err)>pid->ScalarA+pid->ScalarB)
				pid->ITerm=0;
			else
				pid->ITerm*=(pid->ScalarA-Abs(pid->Err)+pid->ScalarB)/pid->ScalarA;
		}	
}


extern M350x_STA M2006[8];
void PID_Init(PID_TypeDef *pid,		uint16_t max_out,		uint16_t intergral_limit,		float deadband,
							float kp,    float Ki,    float Kd,
							float Changing_Integral_A,    float Changing_Integral_B, uint16_t mode)
{
    pid->DeadBand = deadband;
    pid->IntegralLimit = intergral_limit;
    pid->MaxOut = max_out;
    pid->Target = 0;

    pid->Kp = kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->ITerm = 0;

    pid->ScalarA = Changing_Integral_A;
    pid->ScalarB = Changing_Integral_B;

    pid->Output = 0;	
		pid->Function_Mode=0;
		pid->Function_Mode=mode;
}

//void PID_Init(PID_TypeDef *pid,		uint16_t max_out,		uint16_t intergral_limit,		float deadband,
//							float kp,    float Ki,    float Kd,
//							float Changing_Integral_A,    float Changing_Integral_B, uint16_t mode);
void All_PID_Init(void)//积分限幅随便给的(最大车速的0.7)，死去随便给的，ab随便给的
												 //这里不用积分分离，使用梯形积分，微分先行，变速积分
{
	PID_Init(&M2006[0].Speed_PID_STruct,15000,1500,-1,        10,0,0,30,90,
					All_Function_But_Integral_Separation);
	PID_Init(&M2006[1].Speed_PID_STruct,15000,1500,-1,        10,0,0,30,90,
					All_Function_But_Integral_Separation);
	PID_Init(&M2006[2].Speed_PID_STruct,15000,1500,-1,        10,0,0,30,90,
					All_Function_But_Integral_Separation);
	PID_Init(&M2006[3].Speed_PID_STruct,15000,1500,-1,        10,0,0,30,90,
					All_Function_But_Integral_Separation);
	
	PID_Init(&M2006[0].Position_PID_STruct,1500,1500,-1,     0.1,0,0,40,120,
					0);
	PID_Init(&M2006[1].Position_PID_STruct,1500,1500,-1,     0.1,0,0,40,120,
					0);
	PID_Init(&M2006[2].Position_PID_STruct,1500,1500,-1,     0.1,0,0,40,120,
					0);
	PID_Init(&M2006[3].Position_PID_STruct,1500,1500,-1,     0.1,0,0,40,120,
					0);

	PID_Init(&M2006[0].Position_P_PID_STruct,15000,1500,-1,     0.1,0,0,40,120,
					0);
	PID_Init(&M2006[1].Position_P_PID_STruct,15000,1500,-1,     0.1,0,0,40,120,
					0);
	PID_Init(&M2006[2].Position_P_PID_STruct,15000,1500,-1,     0.1,0,0,40,120,
					0);
	PID_Init(&M2006[3].Position_P_PID_STruct,15000,1500,-1,     0.1,0,0,40,120,
					0);
					
	PID_Init(&M2006[0].Position_S_PID_STruct,15000,1500,-1,     10,0.5,2,30,90,
					0);
	PID_Init(&M2006[1].Position_S_PID_STruct,15000,1500,-1,     10,0.5,2,30,90,
					0);
	PID_Init(&M2006[2].Position_S_PID_STruct,15000,1500,-1,     10,0.5,2,30,90,
					0);
	PID_Init(&M2006[3].Position_S_PID_STruct,15000,1500,-1,     10,0.5,2,50,90,
					0);
					
					
//路径PID
//保持位置
	PID_Init(&Keep_X_PID,5000,1500,4,                    0,0,0,5000,10000,//5,0.02,50,5000,10000,
					0);	
	PID_Init(&Keep_Y_PID,5000,1500,4,                    0,0,0,5000,10000,//5,0.02,50,5000,10000,
					0);	

	PID_Init(&Keep_W_PID,2000,1500,-1,                   1,0,0,8,50,          //积分限幅随便给的
					0);	
	PID_Init(&Keep_Angle_PID,800,1500,0.2,               0.1,0,0,20,90,
					0);	
//直线
	PID_Init(&Line_X_PID,5000,1500,0,                    13,0.03,5,50,100,
					All_Function_But_Integral_Separation);	
	PID_Init(&Line_Y_PID,5000,1500,0,                    13,0.03,5,50,100,
					All_Function_But_Integral_Separation);	
	PID_Init(&Line_w_PID,2000,1500,-1,                   5,0,15,8,50,            //5,0.45,15,8,50,
					All_Function_But_Integral_Separation);	
	PID_Init(&Line_Angle_PID,2000,1500,0,                1,0,0,20,90,              //15,0,0,20,90,
					All_Function_But_Integral_Separation);	
//圆弧
	PID_Init(&Cir_X_PID,5000,1500,0,                     6,0.01,10,50,100,
					All_Function_But_Integral_Separation);	
	PID_Init(&Cir_Y_PID,5000,1500,0,                     25,0.15,40,100,200,
					All_Function_But_Integral_Separation);	
	PID_Init(&Cir_W_PID,5000,1500,-1,                     5,0.45,15,8,50,
					All_Function_But_Integral_Separation);	
	PID_Init(&Cir_Angle_PID,5000,1500,0,                 15,0,0,20,90,
					All_Function_But_Integral_Separation);	
}
#endif




