#ifndef __ELMO_H
#define __ELMO_H
#include "sys.h"

typedef struct 
{
  s16 Speed;
}LUN_GU_STA;

void Elmo_Can_Init(void);
void Elmo_Motor_Init(void);
void Elmo_Set_Speed(s32 Speed,s32 ID);
void Elmo_Set_Begin(s32 ID);
#endif
