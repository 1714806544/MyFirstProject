#ifndef _PID_H_
#define _PID_H_
#include "sys.h"

#define IS_PID_MORE_FUNCTION 1   //选择PID模式
																 //1.传统PID
																 //2.魔改PID 梯形积分 微分先行 积分分离 变速积分 
																 //3...可拓展功能



#if IS_PID_MORE_FUNCTION == 1

typedef struct PID_TypeDef
{
    float Target;
    float Kp;
    float Ki;
    float Kd;

    float Measure;
    float Err;
    float Last_Err;
		float Last_Last_Err;

    float Output;
} PID_TypeDef;
							
							
void PID_Calculate (PID_TypeDef *pid,float measure,float target);
							
void PID_Init(PID_TypeDef *pid , float kp,    float Ki,    float Kd);
void All_PID_Init(void);
#endif

							
							
#if IS_PID_MORE_FUNCTION == 2
							
#define Trapezoid_Intergral       0X01       //梯形积分
#define Derivative_On_Measurement 0X02       //微分先行
#define Integral_Separation       0X04       //积分分离
#define ChangingIntegralRate      0X08       //变速积分
#define All_Function_But_Integral_Separation 0XFB
							
							
//PID结构体
typedef struct PID_TypeDef
{
    float Target;
    float Kp;
    float Ki;
    float Kd;

    float Measure;
    float Last_Measure;
    float Err;
    float Last_Err;
		float Max_Err;

    float Pout;
    float Iout; //Iout = ITerm_0 + ITerm_1 +....+ ITerm_n
    float Dout;
    float ITerm; //ITerm = Err * Ki

    float Output;
    float Last_Output;

    float MaxOut;
    float IntegralLimit;
    float DeadBand;
    float ScalarA; //变积分公式参数
    float ScalarB; //ITerm = Err*((A-abs(err)+B)/A)  when B<|err|<A+B
		
		uint16_t Function_Mode;//PID使能功能
} PID_TypeDef;

float Abs(float a);

void PID_Function_Selection(PID_TypeDef *pid);
void PID_Integral_Limit(PID_TypeDef *pid);
void PID_Calculate (PID_TypeDef *pid,float measure,float target);
void PID_Init(PID_TypeDef *pid,		uint16_t max_out,		uint16_t intergral_limit,		float deadband,
							float kp,    float Ki,    float Kd,
							float Changing_Integral_A,    float Changing_Integral_B, uint16_t mode);
void All_PID_Init(void);
#endif



#endif
