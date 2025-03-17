#include "Routine.h"


//设定小车速度

/*
此处未来写串口通信中断回调函数
*/

void Calculate_CarVelocity(float Speed, float Omega)
{
	float Wheel0_Velocity = Speed / Car_R + Omega * sqrt_a_b / (Car_R * b);
	float Wheel1_Velocity = -Speed / Car_R + Omega * sqrt_a_b / (Car_R * b);
	M0_Set_Acceleration_Velocity(Wheel0_Velocity);
	M1_Set_Acceleration_Velocity(Wheel1_Velocity);
}

void Calculate_CarDisPlacement(float Distance, float Omega )
{
	if (Distance != 0)
	{
		Omega = Omega / Distance;
	}
	float Wheel0_Angle = Distance / Car_R + Omega * sqrt_a_b / (Car_R * b);
	float Wheel1_Angle = -Distance / Car_R + Omega * sqrt_a_b / (Car_R * b);
	
	M0_Set_Acceleration_Velocity_Angle(Wheel0_Angle);
	M1_Set_Acceleration_Velocity_Angle(Wheel1_Angle);
		
}

