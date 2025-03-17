#ifndef __ACCELERATION_H
#define __ACCELERATION_H

#include "SVPWM.h"
#include "LowPass_Filter.h"
#include "PID.h"

void Acceleration_PIDInit_M0(float P, float I, float D);//初始化加速度PID
void Acceleration_PIDInit_M1(float P, float I, float D);

void Acl_LowPass_FilterInit_M0(void);//初始化低通滤波器
void Acl_LowPass_FilterInit_M1(void);

PIDparameters Get_Acceleration_PID_M0(void);//获取pid参数接口
PIDparameters Get_Acceleration_PID_M1(void);

float s0_GetAcceleration(void);//获取编码器加速度
float s1_GetAcceleration(void);

float s0_GetAcceleration_Filtered(void);
float s1_GetAcceleration_Filtered(void);

void M0_SetAcceleration(float Target);//直接控制加速度
void M1_SetAcceleration(float Target);

void M0_Set_Acceleration_Velocity(float Target);//加速度-速度串级PID控制速度
void M1_Set_Acceleration_Velocity(float Target);

void M0_Set_Acceleration_Velocity_Angle(float Target);//加速度-速度-角度串级PID控制角度
void M1_Set_Acceleration_Velocity_Angle(float Target);

void set_MAX_AcAcceleration_M0(float MAX_Acceleration_M0);//设定加速度最大值（加速度-速度-角度串级pid）
void set_MAX_AcAcceleration_M1(float MAX_Acceleration_M1);

#endif
