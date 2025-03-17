#ifndef __ROUTINE_H
#define __ROUTINE_H

#include "SVPWM.h"
#include "as5600.h"
#include "acceleration.h"

#define Car_D 0.08f
#define Car_R 0.04f
#define a 12.5f
#define b 8
#define sqrt_a_b 14.840822f


//小车持续运行设定结构体，改变结构体参数以改变运行方式（角度制）

void Calculate_CarVelocity(float Speed, float Omega);
void Calculate_CarDisPlacement(float Distance, float Omega);

#endif
