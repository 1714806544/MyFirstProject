#ifndef _M3508_H_
#define _M3508_H_ 
#include "sys.h"
#include "pid.h"

#define M2006_0 1
#define M2006_1 1
#define M2006_2 1
#define M2006_3 1
#define M2006_4 0
#define M2006_5 0
#define M2006_6 0
#define M2006_7 0

typedef struct 
{
	s16 Angle;
  s16 Speed;

	PID_TypeDef Speed_PID_STruct;
	PID_TypeDef Position_PID_STruct;
	PID_TypeDef Position_S_PID_STruct;
	PID_TypeDef Position_P_PID_STruct;
	s32 dAngle_Sum;
	s16 Angle_Old;
	s16 Out;
}M350x_STA;

void CAN2_Configuration(u8 mode);

void CAN2_SetMotor_0_3(M350x_STA *M350x);
void CAN2_SetMotor_4_7(M350x_STA *M350x);


void M2006_Can_Receive(CanRxMsg rx_message);
void M3510_Position_S(M350x_STA *M350x,s32 Angle_Sum);

void M3510_SpeedMode(s32 Speed,M350x_STA *M350x);
void M3510_Angle_Calculate(M350x_STA *M350x);
void M3510_KeepPosition(M350x_STA *M350x,s32 Angle_Sum);
#endif
