#ifndef __SVPWM_H
#define __SVPWM_H

#include "PID.h"
#include "tim.h"
#include "micro_5ms.h"
#include "Current_Sensor.h"
#include "as5600.h"
#include "LowPass_Filter.h"
#define ARM_MATH_CM4
#include "arm_math.h"

float S0_electricalAngle(void);//获取电角度
float S1_electricalAngle(void);

void M0_SetVelocity(float Target);//设定角速度
void M1_SetVelocity(float Target);

void M0_Set_Velocity_Angle(float Target);//设定角度（速度-角度串级pid）
void M1_Set_Velocity_Angle(float Target);

void M0_set_Force_Angle(float Target);//设定角度
void M1_set_Force_Angle(float Target);

float s0_GetVelocity(void);//获取编码器速度值
float s1_GetVelocity(void);

void Vel_LowPass_FilterInit_M0(void);//初始化速度低通滤波器
void Vel_LowPass_FilterInit_M1(void);

float s0_GetVelocity_Filtered(void);//获取速度低通滤波值
float s1_GetVelocity_Filtered(void);

void Velocity_PIDInit_M0(float P, float I, float D);//初始化速度PID
void Velocity_PIDInit_M1(float P, float I, float D);

void Angle_PIDInit_M0(float P, float I, float D);//初始化角度PID
void Angle_PIDInit_M1(float P, float I, float D);

PIDparameters Get_Angle_PID_M0(void);//获取pid参数接口
PIDparameters Get_Angle_PID_M1(void);
PIDparameters Get_Velocity_PID_M0(void);
PIDparameters Get_Velocity_PID_M1(void);

void M0_setTorque(float Uq, float angle_el);//设定Uq
void M1_setTorque(float Uq, float angle_el);

void set_MAX_Velocity_M0(float MAX_Velocity_M0);//设定速度最大值（速度-角度串级pid）
void set_MAX_Velocity_M1(float MAX_Velocity_M1);

void set_MAX_Acceleration_M0(float MAX_Acceleration_M0);//设定加速度最大值（加速度-速度-角度串级pid）
void set_MAX_Acceleration_M1(float MAX_Acceleration_M1);

#endif
