#ifndef __LJ_H
#define __LJ_H
#include "sys.h"


//ֱ�ߣ�Ŀ��x,y,Ҫת�ĽǶȣ�V����Vĩ��a���٣�a����,w���  15000 5000 1000
void line(float Target_x,float Target_y,float rotate_Sum,int Start_Speed,int Target_Speed,int speedup_a,int slow_a,int Max_w,int speed_max);

//˳ʱ��Բ����Ŀ��x,y,Բ�Ľǣ�Բ��x��y���뾶����������V����Vĩ��
void circle(float Target_x,float Target_y,float angle,float c_x,float c_y,float c_r,int yanqie,int start_v,int target_v,int speed_max);

//��ʱ��Բ����Ŀ��x,y,Բ�Ľǣ�Բ��x��y���뾶����������V����Vĩ��
void anticircle(float Target_x,float Target_y,float angle,float c_x,float c_y,float c_r,int yanqie,int start_v,int target_v,int speed_max);


void lj1_0(void);
void lj1_1(float acc_jia,float acc_jian,float v_max);
void lj2_0(void);

void lj2_3(float v__max);

#endif
