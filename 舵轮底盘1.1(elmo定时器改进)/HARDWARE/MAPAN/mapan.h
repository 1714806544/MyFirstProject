#ifndef _MAPAN_H_
#define _MAPAN_H_
#include "sys.h"

#define MAPAN_USART 1 //码盘使用的串口


typedef struct 
{
	float angle_Z;
	float angle_y;
	float angle_x;
	float x;
	float y;
	float w;
} Action_data;

void mapan_init(void);

#endif
