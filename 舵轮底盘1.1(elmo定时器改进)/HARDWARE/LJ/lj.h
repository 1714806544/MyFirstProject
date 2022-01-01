#ifndef __LJ_H
#define __LJ_H
#include "sys.h"


//直线（目标x,y,要转的角度，V初，V末，a加速，a减速,w最大）  15000 5000 1000
void line(float Target_x,float Target_y,float rotate_Sum,int Start_Speed,int Target_Speed,int speedup_a,int slow_a,int Max_w,int speed_max);

//顺时针圆弧（目标x,y,圆心角，圆心x，y，半径，沿切线吗，V初，V末）
void circle(float Target_x,float Target_y,float angle,float c_x,float c_y,float c_r,int yanqie,int start_v,int target_v,int speed_max);

//逆时针圆弧（目标x,y,圆心角，圆心x，y，半径，沿切线吗，V初，V末）
void anticircle(float Target_x,float Target_y,float angle,float c_x,float c_y,float c_r,int yanqie,int start_v,int target_v,int speed_max);


void lj1_0(void);
void lj1_1(float acc_jia,float acc_jian,float v_max);
void lj2_0(void);

void lj2_3(float v__max);

#endif
