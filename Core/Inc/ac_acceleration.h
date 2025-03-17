#ifndef __AC_ACCELERATION_H
#define __AC_ACCELERATION_H

#include "SVPWM.h"
#include "PID.h"
#include "acceleration.h"

void Acceleration_PIDInit_M0(float P, float I, float D);//初始化加速度PID
void Acceleration_PIDInit_M1(float P, float I, float D);

float s0_GetAcAcceleration(void);//获取编码器加加速度
float s1_GetAcAcceleration(void);

void M0_SetAcAcceleration(float Target);//直接控制加加速度
void M1_SetAcAcceleration(float Target);

void M0_Set_AcAcceleration_Acceleration(float Target);//加加速度-加速度串级PID控制速度
void M1_Set_AcAcceleration_Acceleration(float Target);

void M0_Set_AcAcceleration_Acceleration_Velocity(float Target);//加加速度-加速度-速度串级PID控制角度
void M1_Set_AcAcceleration_Acceleration_Velocity(float Target);

void M0_Set_AcAcceleration_Acceleration_Velocity_Angle(float Target);//加加速度-加速度-速度-角度串级PID控制角度
void M1_Set_AcAcceleration_Acceleration_Velocity_Angle(float Target);

#endif
