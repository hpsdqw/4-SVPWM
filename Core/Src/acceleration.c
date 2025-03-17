#include "acceleration.h"

static float Acl0_pre = 0.0f;       // 保存上一次计算的加速度
static float M0_Vel_acc = 0.0f;  // 累计的速度变化
static uint64_t T_start0 = 0;       // 累计时间的起点
static float M0_Vel_pre = 0.0f;  // 保存上一次的速度
static float Acl1_pre = 0.0f;       // 保存上一次计算的加速度
static float M1_Vel_acc = 0.0f;  // 累计的速度变化
static uint64_t T_start1 = 0;       // 累计时间的起点
static float M1_Vel_pre = 0.0f;  // 保存上一次的速度

static PIDparameters Acceleration_PID_M0 = {0};
static PIDparameters Acceleration_PID_M1 = {0};

static LowPass_Filter_Parameters Acl_LowPass_Filter_M0 = {0};
static LowPass_Filter_Parameters Acl_LowPass_Filter_M1 = {0};

void Acceleration_PIDInit_M0(float P, float I, float D)
{
	Acceleration_PID_M0.P = P;
	Acceleration_PID_M0.I = I;
	Acceleration_PID_M0.D = D;
	Acceleration_PID_M0.output_ramp = 1000000;
	Acceleration_PID_M0.limit = 6;
	Acceleration_PID_M0.error_prev = 0;
	Acceleration_PID_M0.output_prev = 0;
	Acceleration_PID_M0.integral_prev = 0;
	Acceleration_PID_M0.timestamp_prev = 0;
	Acceleration_PID_M1.N = 20;          
    Acceleration_PID_M1.D_filter_prev = 0; 
}

void Acceleration_PIDInit_M1(float P, float I, float D)
{
	Acceleration_PID_M1.P = P;
	Acceleration_PID_M1.I = I;
	Acceleration_PID_M1.D = D;
	Acceleration_PID_M1.output_ramp = 1000000;
	Acceleration_PID_M1.limit = 6;
	Acceleration_PID_M1.error_prev = 0;
	Acceleration_PID_M1.output_prev = 0;
	Acceleration_PID_M1.integral_prev = 0;
	Acceleration_PID_M1.timestamp_prev = 0;
	Acceleration_PID_M1.N = 20;          
    Acceleration_PID_M1.D_filter_prev = 0; 
}

//速度低通滤波器初始化
void Acl_LowPass_FilterInit_M0(void)
{
	Acl_LowPass_Filter_M0.timestamp_prev = 0;
	Acl_LowPass_Filter_M0.y_prev = 0;
	Acl_LowPass_Filter_M0.Tf = 0.005;
}

void Acl_LowPass_FilterInit_M1(void)
{
	Acl_LowPass_Filter_M1.timestamp_prev = 0;
	Acl_LowPass_Filter_M1.y_prev = 0;
	Acl_LowPass_Filter_M1.Tf = 0.005;
}

PIDparameters Get_Acceleration_PID_M0(void)
{
	return Acceleration_PID_M0;
}
PIDparameters Get_Acceleration_PID_M1(void)
{
	return Acceleration_PID_M1;
}

float s0_GetAcceleration(void)
{

    float M0_Vel = s0_GetVelocity_Filtered(); // 当前速度
    float d_Velocity_0 = M0_Vel - M0_Vel_pre;

    // 累计速度变化
    M0_Vel_acc += d_Velocity_0;

    // 获取当前时间
    uint64_t Time_Now_0 = micro_5us();

    // 初始化时间起点（仅在第一次进入时执行）
    if (T_start0 == 0) {
        T_start0 = Time_Now_0;
        M0_Vel_pre = M0_Vel; // 更新历史速度
        return Acl0_pre;         // 返回初始加速度
    }

    // 计算累计时间间隔
    float Ts = (Time_Now_0 - T_start0) * 5e-6f; // 转换为秒

    // 如果时间间隔足够长，计算速度
    float Acl;
    if (Ts >= 0.000025f) { // 时间窗口 0.025 ms，采样频率40kHZ
        Acl = M0_Vel_acc / Ts; // 累计角度变化计算加速度
        Acl0_pre = Acl;          // 更新加速度
        M0_Vel_acc = 0.0f;     // 清空累计角度
        T_start0 = Time_Now_0;      // 重置时间起点
    } 
	else
	{
        Acl = Acl0_pre; // 时间不足时，返回之前的加速度
    }

    // 更新历史速度
    M0_Vel_pre = M0_Vel;

    return Acl; // 返回加速度
}

float s1_GetAcceleration(void)
{

    float M1_Vel = s1_GetVelocity_Filtered(); // 当前速度
    float d_Velocity_1 = M1_Vel - M1_Vel_pre;

    // 累计速度变化
    M1_Vel_acc += d_Velocity_1;

    // 获取当前时间
    uint64_t Time_Now_1 = micro_5us();

    // 初始化时间起点（仅在第一次进入时执行）
    if (T_start1 == 0) {
        T_start1 = Time_Now_1;
        M1_Vel_pre = M1_Vel; // 更新历史速度
        return Acl1_pre;         // 返回初始加速度
    }

    // 计算累计时间间隔
    float Ts = (Time_Now_1 - T_start1) * 5e-6f; // 转换为秒

    // 如果时间间隔足够长，计算速度
    float Acl;
    if (Ts >= 0.000025f) { // 时间窗口 0.05 ms，采样频率40kHZ
        Acl = M1_Vel_acc / Ts; // 累计角度变化计算加速度
        Acl1_pre = Acl;          // 更新加速度
        M1_Vel_acc = 0.0f;     // 清空累计角度
        T_start1 = Time_Now_1;      // 重置时间起点
    } 
	else
	{
        Acl = Acl1_pre; // 时间不足时，返回之前的加速度
    }

    // 更新历史速度
    M1_Vel_pre = M1_Vel;

    return Acl; // 返回加速度
}

//滤波后的速度
float s0_GetAcceleration_Filtered(void)
{
	return LowPass_Filter(s0_GetAcceleration() , &Acl_LowPass_Filter_M0);
}

float s1_GetAcceleration_Filtered(void)
{
	return LowPass_Filter(s1_GetAcceleration() , &Acl_LowPass_Filter_M1);
}

void M0_SetAcceleration(float Target)
{
	float error_M0 = Target - s0_GetAcceleration_Filtered();
	M0_setTorque(PIDController(&Acceleration_PID_M0 , error_M0) , S0_electricalAngle());   //加速度闭环
}

void M1_SetAcceleration(float Target)
{
	float error_M1 = Target - s1_GetAcceleration_Filtered();
	M1_setTorque(PIDController(&Acceleration_PID_M1 , error_M1) , S1_electricalAngle());   //加速度闭环
}

void M0_Set_Acceleration_Velocity(float Target)
{
	PIDparameters Velocity_PID_M0 = Get_Velocity_PID_M0();
	float error_M0_First = Target - s0_GetVelocity_Filtered();
	float error_M0_Second = PIDController(&Velocity_PID_M0 , error_M0_First) - s0_GetAcceleration_Filtered();
	M0_setTorque(PIDController(&Acceleration_PID_M0 , error_M0_Second) , S0_electricalAngle());
}

void M1_Set_Acceleration_Velocity(float Target)
{
	PIDparameters Velocity_PID_M1 = Get_Velocity_PID_M1();
	float error_M1_First = Target - s1_GetVelocity_Filtered();
	float error_M1_Second = PIDController(&Velocity_PID_M1 , error_M1_First) - s1_GetAcceleration_Filtered();
	M1_setTorque(PIDController(&Acceleration_PID_M1 , error_M1_Second) , S1_electricalAngle());
}

void M0_Set_Acceleration_Velocity_Angle(float Target)
{
	PIDparameters Angle_PID_M0 = Get_Angle_PID_M0();
	PIDparameters Velocity_PID_M0 = Get_Velocity_PID_M0();
	float error_M0_First = Target - bsp_as5600_1_GetAngle();
	float error_M0_Second = PIDController(&Angle_PID_M0 , error_M0_First) - s0_GetVelocity_Filtered();
	float error_M0_Third = PIDController(&Velocity_PID_M0 , error_M0_Second) - s0_GetAcceleration_Filtered();
	M0_setTorque(PIDController(&Acceleration_PID_M0 , error_M0_Second) , S0_electricalAngle());
}

void M1_Set_Acceleration_Velocity_Angle(float Target)
{
	PIDparameters Angle_PID_M1 = Get_Angle_PID_M1();
	PIDparameters Velocity_PID_M1 = Get_Velocity_PID_M1();
	float error_M1_First = Target - bsp_as5600_2_GetAngle();
	float error_M1_Second = PIDController(&Angle_PID_M1 , error_M1_First) - s1_GetVelocity_Filtered();
	float error_M1_Third = PIDController(&Velocity_PID_M1 , error_M1_Second) - s1_GetAcceleration_Filtered();
	M1_setTorque(PIDController(&Acceleration_PID_M1 , error_M1_Second) , S1_electricalAngle());
}

void set_MAX_AcAcceleration_M0(float MAX_Acceleration_M0)
{
	Acceleration_PID_M0.limit = MAX_Acceleration_M0;
}

void set_MAX_AcAcceleration_M1(float MAX_Acceleration_M1)
{
	Acceleration_PID_M1.limit = MAX_Acceleration_M1;
}

