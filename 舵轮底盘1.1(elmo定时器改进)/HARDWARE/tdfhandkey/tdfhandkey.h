#ifndef __TDFHANDKEY_H
#define __TDFHANDKEY_H
#include "sys.h"

#define buttom0 1648
#define buttom1 1024
#define buttom2 364

#define JockerMidd 1024

typedef struct
{
	int Buttom_Left;
	int Buttom_Right;
	
	int Joystick_Leftx;
	int Joystick_Lefty;
	int Joystick_Rightx;
	int Joystick_Righty;
	
	int Knob;

} TDF_struct;

typedef __packed struct
{
__packed struct
	 {
	 uint16_t ch0;
	 uint16_t ch1;
	 uint16_t ch2;
	 uint16_t ch3;
	 uint8_t s1;
	 uint8_t s2;
	 }rc;
	 
 __packed struct
	 {
	 int16_t x;
	 int16_t y;
	 int16_t z;
	 uint8_t press_l;
	 uint8_t press_r;
	 }mouse;
	 
__packed struct
	 {
	 uint16_t v;
	 }key;
	 
}RC_Ctl_t;

void Data_Init(void);
void uart2_init(u32 bound);

#endif
