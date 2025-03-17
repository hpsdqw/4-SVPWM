#include "ac_acceleration.h"

static float A_Acl0_pre = 0.0f;       // 保存上一次计算的加加速度
static float M0_Acl_acc = 0.0f;  // 累计的加速度变化
static uint64_t T_start0 = 0;       // 累计时间的起点
static float M0_Acl_pre = 0.0f;  // 保存上一次的加速度
static float A_Acl1_pre = 0.0f;       // 保存上一次计算的加加速度
static float M1_Acl_acc = 0.0f;  // 累计的加速度变化
static uint64_t T_start1 = 0;       // 累计时间的起点
static float M1_Acl_pre = 0.0f;  // 保存上一次的加速度

static PIDparameters Ac_Acceleration_PID_M0 = {0};
static PIDparameters Ac_Acceleration_PID_M1 = {0};

void Ac_Acceleration_PIDInit_M0(float P, float I, float D)
{
	Ac_Acceleration_PID_M0.P = P;
	Ac_Acceleration_PID_M0.I = I;
	Ac_Acceleration_PID_M0.D = D;
	Ac_Acceleration_PID_M0.output_ramp = 1000000;
	Ac_Acceleration_PID_M0.limit = 100;
	Ac_Acceleration_PID_M0.error_prev = 0;
	Ac_Acceleration_PID_M0.output_prev = 0;
	Ac_Acceleration_PID_M0.integral_prev = 0;
	Ac_Acceleration_PID_M0.timestamp_prev = 0;
}

void Ac_Acceleration_PIDInit_M1(float P, float I, float D)
{
	Ac_Acceleration_PID_M1.P = P;
	Ac_Acceleration_PID_M1.I = I;
	Ac_Acceleration_PID_M1.D = D;
	Ac_Acceleration_PID_M1.output_ramp = 1000000;
	Ac_Acceleration_PID_M1.limit = 100;
	Ac_Acceleration_PID_M1.error_prev = 0;
	Ac_Acceleration_PID_M1.output_prev = 0;
	Ac_Acceleration_PID_M1.integral_prev = 0;
	Ac_Acceleration_PID_M1.timestamp_prev = 0;
}

float s0_GetAcAcceleration(void)
{
    float M0_Acl = s0_GetAcceleration(); // 当前加速度
    float d_Acl_0 = M0_Acl - M0_Acl_pre;

    // 累计加速度变化
    M0_Acl_acc += d_Acl_0;

    // 获取当前时间
    uint64_t Time_Now_0 = micro_5us();

    // 初始化时间起点（仅在第一次进入时执行）
    if (T_start0 == 0) {
        T_start0 = Time_Now_0;
        M0_Acl_pre = M0_Acl; // 更新历史加速度
        return A_Acl0_pre;         // 返回初始加加速度
    }

    // 计算累计时间间隔
    float Ts = (Time_Now_0 - T_start0) * 5e-6f; // 转换为秒

    // 如果时间间隔足够长，计算加加速度
    float A_Acl;
    if (Ts >= 0.00025f) { // 时间窗口 0.25 ms，采样频率4kHZ
        A_Acl = M0_Acl_acc / Ts; // 累计角度变化计算加加速度
        A_Acl0_pre = A_Acl;          // 更新加加速度
        M0_Acl_acc = 0.0f;     // 清空累计加速度
        T_start0 = Time_Now_0;      // 重置时间起点
    } 
	else
	{
        A_Acl = A_Acl0_pre; // 时间不足时，返回之前的加加速度
    }

    // 更新历史速度
    M0_Acl_pre = M0_Acl;

    return A_Acl; // 返回加加速度
}

float s1_GetAcAcceleration(void)
{
    float M1_Acl = s1_GetAcceleration(); // 当前加速度
    float d_Acl_1 = M1_Acl - M1_Acl_pre;

    // 累计加速度变化
    M1_Acl_acc += d_Acl_1;

    // 获取当前时间
    uint64_t Time_Now_1 = micro_5us();

    // 初始化时间起点（仅在第一次进入时执行）
    if (T_start1 == 0) {
        T_start1 = Time_Now_1;
        M1_Acl_pre = M1_Acl; // 更新历史加速度
        return A_Acl1_pre;         // 返回初始加加速度
    }

    // 计算累计时间间隔
    float Ts = (Time_Now_1 - T_start1) * 5e-6f; // 转换为秒

    // 如果时间间隔足够长，计算加加速度
    float A_Acl;
    if (Ts >= 0.00025f) { // 时间窗口 0.25 ms，采样频率4kHZ
        A_Acl = M1_Acl_acc / Ts; // 累计角度变化计算加加速度
        A_Acl1_pre = A_Acl;          // 更新加加速度
        M1_Acl_acc = 0.0f;     // 清空累计加速度
        T_start1 = Time_Now_1;      // 重置时间起点
    } 
	else
	{
        A_Acl = A_Acl1_pre; // 时间不足时，返回之前的加加速度
    }

    // 更新历史速度
    M1_Acl_pre = M1_Acl;

    return A_Acl; // 返回加加速度
}

void M0_SetAcAcceleration(float Target)
{
	float error_M0 = Target - s0_GetAcAcceleration();
	M0_setTorque(PIDController(&Ac_Acceleration_PID_M0 , error_M0) , S0_electricalAngle());   //加加速度闭环
}

void M1_SetAcAcceleration(float Target)
{
	float error_M1 = Target - s1_GetAcAcceleration();
	M1_setTorque(PIDController(&Ac_Acceleration_PID_M1 , error_M1) , S1_electricalAngle());   //加加速度闭环
}

void M0_Set_AcAcceleration_Acceleration(float Target)
{
	PIDparameters Acceleration_PID_M0 = Get_Acceleration_PID_M0();
	float error_M0_First = Target - s0_GetAcceleration();
	float error_M0_Second = PIDController(&Acceleration_PID_M0 , error_M0_First) - s0_GetAcceleration();
	M0_setTorque(PIDController(&Ac_Acceleration_PID_M0 , error_M0_Second) , S0_electricalAngle());
}

void M1_Set_AcAcceleration_Acceleration(float Target)
{
	PIDparameters Acceleration_PID_M1 = Get_Acceleration_PID_M1();
	float error_M1_First = Target - s1_GetAcceleration();
	float error_M1_Second = PIDController(&Acceleration_PID_M1 , error_M1_First) - s1_GetAcceleration();
	M1_setTorque(PIDController(&Ac_Acceleration_PID_M1 , error_M1_Second) , S1_electricalAngle());
}

void M0_Set_AcAcceleration_Acceleration_Velocity(float Target)
{
	PIDparameters Velocity_PID_M0 = Get_Velocity_PID_M0();
	PIDparameters Acceleration_PID_M0 = Get_Acceleration_PID_M0();
	float error_M0_First = Target - s0_GetVelocity_Filtered();
	float error_M0_Second = PIDController(&Velocity_PID_M0 , error_M0_First) - s0_GetAcceleration();
	float error_M0_Third = PIDController(&Acceleration_PID_M0 , error_M0_Second) - s0_GetAcAcceleration();
	M0_setTorque(PIDController(&Ac_Acceleration_PID_M0 , error_M0_Third) , S0_electricalAngle());
}

void M1_Set_AcAcceleration_Acceleration_Velocity(float Target)
{
	PIDparameters Velocity_PID_M1 = Get_Velocity_PID_M1();
	PIDparameters Acceleration_PID_M1 = Get_Acceleration_PID_M1();
	float error_M1_First = Target - s1_GetVelocity_Filtered();
	float error_M1_Second = PIDController(&Velocity_PID_M1 , error_M1_First) - s1_GetAcceleration();
	float error_M1_Third = PIDController(&Acceleration_PID_M1 , error_M1_Second) - s1_GetAcAcceleration();
	M1_setTorque(PIDController(&Ac_Acceleration_PID_M1 , error_M1_Third) , S1_electricalAngle());
}

void M0_Set_AcAcceleration_Acceleration_Velocity_Angle(float Target)
{
	PIDparameters Angle_PID_M0 = Get_Angle_PID_M0();
	PIDparameters Velocity_PID_M0 = Get_Velocity_PID_M0();
	PIDparameters Acceleration_PID_M0 = Get_Acceleration_PID_M0();
	float error_M0_First = Target - bsp_as5600_1_GetAngle();
	float error_M0_Second = PIDController(&Angle_PID_M0 , error_M0_First) - s0_GetVelocity_Filtered();
	float error_M0_Third = PIDController(&Velocity_PID_M0 , error_M0_Second) - s0_GetAcceleration();
	float error_M0_Forth = PIDController(&Acceleration_PID_M0 , error_M0_Second) - s0_GetAcAcceleration();
	M0_setTorque(PIDController(&Ac_Acceleration_PID_M0 , error_M0_Forth) , S0_electricalAngle());
}

void M1_Set_AcAcceleration_Acceleration_Velocity_Angle(float Target)
{
	PIDparameters Angle_PID_M1 = Get_Angle_PID_M1();
	PIDparameters Velocity_PID_M1 = Get_Velocity_PID_M1();
	PIDparameters Acceleration_PID_M1 = Get_Acceleration_PID_M1();
	float error_M1_First = Target - bsp_as5600_2_GetAngle();
	float error_M1_Second = PIDController(&Angle_PID_M1 , error_M1_First) - s1_GetVelocity_Filtered();
	float error_M1_Third = PIDController(&Velocity_PID_M1 , error_M1_Second) - s1_GetAcceleration();
	float error_M1_Forth = PIDController(&Acceleration_PID_M1 , error_M1_Second) - s1_GetAcAcceleration();
	M1_setTorque(PIDController(&Ac_Acceleration_PID_M1 , error_M1_Forth) , S1_electricalAngle());
}
