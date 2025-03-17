#include "SVPWM.h"

#define _PI 3.14159265359f
#define _PI_2 1.57079632679f
#define _PI_3 1.0471975512f
#define _3PI_2 4.71238898038f

#define _SQRT3 1.73205080757f
#define _SQRT3_2 0.86602540378f
#define _1_SQRT3 0.57735026919f
#define _2_SQRT3 1.15470053838f

static float S0_zero_electric_angle = 0;
static float S1_zero_electric_angle = 0;
static float voltage_power_supply = 12;

static float Vel0_pre = 0.0f;       // 保存上一次计算的速度
static float M0_Angle_acc = 0.0f;  // 累计的角度变化
static uint64_t T_start0 = 0;       // 累计时间的起点
static float M0_Angle_pre = 0.0f;  // 保存上一次的角度
static float Vel1_pre = 0.0f;       // 保存上一次计算的速度
static float M1_Angle_acc = 0.0f;  // 累计的角度变化
static uint64_t T_start1 = 0;       // 累计时间的起点
static float M1_Angle_pre = 0.0f;  // 保存上一次的角度

static int M0_PP = 7, M0_DIR = 1;
static int M1_PP = 7, M1_DIR = 1;

static PIDparameters Velocity_PID_M0 = {0};
static PIDparameters Velocity_PID_M1 = {0};
static PIDparameters Angle_PID_M0 = {0};
static PIDparameters Angle_PID_M1 = {0};

static LowPass_Filter_Parameters Vel_LowPass_Filter_M0 = {0};
static LowPass_Filter_Parameters Vel_LowPass_Filter_M1 = {0};
//初始变量及函数定义
#define _constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
//宏定义实现的一个约束函数,用于限制一个值的范围。
//具体来说，该宏定义的名称为 _constrain，接受三个参数 amt、low 和 high，分别表示要限制的值、最小值和最大值。该宏定义的实现使用了三元运算符，根据 amt 是否小于 low 或大于 high，返回其中的最大或最小值，或者返回原值。
//换句话说，如果 amt 小于 low，则返回 low；如果 amt 大于 high，则返回 high；否则返回 amt。这样，_constrain(amt, low, high) 就会将 amt 约束在 [low, high] 的范围内。1

//速度PID初始化
void Velocity_PIDInit_M0(float P, float I, float D)
{
	Velocity_PID_M0.P = P;
	Velocity_PID_M0.I = I;
	Velocity_PID_M0.D = D;
	Velocity_PID_M0.output_ramp = 1000000;
	Velocity_PID_M0.limit = 6;
	Velocity_PID_M0.error_prev = 0;
	Velocity_PID_M0.output_prev = 0;
	Velocity_PID_M0.integral_prev = 0;
	Velocity_PID_M0.timestamp_prev = 0;
	Velocity_PID_M0.N = 20;          
    Velocity_PID_M0.D_filter_prev = 0; 
}

void Velocity_PIDInit_M1(float P, float I, float D)
{
	Velocity_PID_M1.P = P;
	Velocity_PID_M1.I = I;
	Velocity_PID_M1.D = D;
	Velocity_PID_M1.output_ramp = 1000000;
	Velocity_PID_M1.limit = 6;
	Velocity_PID_M1.error_prev = 0;
	Velocity_PID_M1.output_prev = 0;
	Velocity_PID_M1.integral_prev = 0;
	Velocity_PID_M1.timestamp_prev = 0;
	Velocity_PID_M1.N = 20;          
    Velocity_PID_M1.D_filter_prev = 0; 	
}

//角度PID初始化
void Angle_PIDInit_M0(float P, float I, float D)
{
	Angle_PID_M0.P = P;
	Angle_PID_M0.I = I;
	Angle_PID_M0.D = D;
	Angle_PID_M0.output_ramp = 1000000;
	Angle_PID_M0.limit = 5;
	Angle_PID_M0.error_prev = 0;
	Angle_PID_M0.output_prev = 0;
	Angle_PID_M0.integral_prev = 0;
	Angle_PID_M0.timestamp_prev = 0;
	Angle_PID_M0.N = 20;          
    Angle_PID_M0.D_filter_prev = 0; 
}

void Angle_PIDInit_M1(float P, float I, float D)
{
	Angle_PID_M1.P = P;
	Angle_PID_M1.I = I;
	Angle_PID_M1.D = D;
	Angle_PID_M1.output_ramp = 1000000;
	Angle_PID_M1.limit = 5;
	Angle_PID_M1.error_prev = 0;
	Angle_PID_M1.output_prev = 0;
	Angle_PID_M1.integral_prev = 0;
	Angle_PID_M1.timestamp_prev = 0;
	Angle_PID_M1.N = 20;          
    Angle_PID_M1.D_filter_prev = 0; 	
}

//获取pid参数api
PIDparameters Get_Angle_PID_M0(void)
{
	return Angle_PID_M0;
}
PIDparameters Get_Angle_PID_M1(void)
{
	return Angle_PID_M1;
}
PIDparameters Get_Velocity_PID_M0(void)
{
	return Velocity_PID_M0;
}
PIDparameters Get_Velocity_PID_M1(void)
{
	return Velocity_PID_M1;
}

//速度低通滤波器初始化
void Vel_LowPass_FilterInit_M0(void)
{
	Vel_LowPass_Filter_M0.timestamp_prev = 0;
	Vel_LowPass_Filter_M0.y_prev = 0;
	Vel_LowPass_Filter_M0.Tf = 0.005;
}

void Vel_LowPass_FilterInit_M1(void)
{
	Vel_LowPass_Filter_M1.timestamp_prev = 0;
	Vel_LowPass_Filter_M1.y_prev = 0;
	Vel_LowPass_Filter_M1.Tf = 0.005;
}

// 归一化角度到 [0,2PI]
float _normalizeAngle(float angle) {
  float a = fmod(angle, 2 * PI);  //取余运算可以用于归一化，列出特殊值例子算便知
  return a >= 0 ? a : (a + 2 * PI);
  //三目运算符。格式：condition ? expr1 : expr2
  //其中，condition 是要求值的条件表达式，如果条件成立，则返回 expr1 的值，否则返回 expr2 的值。可以将三目运算符视为 if-else 语句的简化形式。
  //fmod 函数的余数的符号与除数相同。因此，当 angle 的值为负数时，余数的符号将与 _2PI 的符号相反。也就是说，如果 angle 的值小于 0 且 _2PI 的值为正数，则 fmod(angle, _2PI) 的余数将为负数。
  //例如，当 angle 的值为 -PI/2，_2PI 的值为 2PI 时，fmod(angle, _2PI) 将返回一个负数。在这种情况下，可以通过将负数的余数加上 _2PI 来将角度归一化到 [0, 2PI] 的范围内，以确保角度的值始终为正数。
}


// 设置PWM到控制器输出
void M0_setPwm(float Ua, float Ub, float Uc) {
  // 限制上限
  Ua = _constrain(Ua, 0.0f, voltage_power_supply);
  Ub = _constrain(Ub, 0.0f, voltage_power_supply);
  Uc = _constrain(Uc, 0.0f, voltage_power_supply);
  // 计算占空比
  // 限制占空比从0到1
  float dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
  float dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
  float dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);

  //写入PWM到PWM 0 1 2 通道
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, dc_a * 210);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, dc_b * 210);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, dc_c * 210);
}

void M1_setPwm(float Ua, float Ub, float Uc) {
  // 限制上限
  Ua = _constrain(Ua, 0.0f, voltage_power_supply);
  Ub = _constrain(Ub, 0.0f, voltage_power_supply);
  Uc = _constrain(Uc, 0.0f, voltage_power_supply);
  // 计算占空比
  // 限制占空比从0到1
  float dc_a = _constrain(Ua / voltage_power_supply, 0.0f, 1.0f);
  float dc_b = _constrain(Ub / voltage_power_supply, 0.0f, 1.0f);
  float dc_c = _constrain(Uc / voltage_power_supply, 0.0f, 1.0f);

  //写入PWM到PWM 0 1 2 通道
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, dc_a * 210);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, dc_b * 210);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, dc_c * 210);
}

void M0_setTorque(float Uq, float angle_el)
{
  if (Uq < 0)
    angle_el += _PI;
  Uq = fabs(Uq);

  angle_el = _normalizeAngle(angle_el + _PI_2);
  int sector = floor(angle_el / _PI_3) + 1;
  // calculate the duty cycles
  float T1 = _SQRT3 * sin(sector * _PI_3 - angle_el) * Uq / voltage_power_supply;
  float T2 = _SQRT3 * sin(angle_el - (sector - 1.0) * _PI_3) * Uq / voltage_power_supply;
  float T0 = 1 - T1 - T2;

  float Ta, Tb, Tc;
  switch (sector)
  {
  case 1:
    Ta = T1 + T2 + T0 / 2;
    Tb = T2 + T0 / 2;
    Tc = T0 / 2;
    break;
  case 2:
    Ta = T1 + T0 / 2;
    Tb = T1 + T2 + T0 / 2;
    Tc = T0 / 2;
    break;
  case 3:
    Ta = T0 / 2;
    Tb = T1 + T2 + T0 / 2;
    Tc = T2 + T0 / 2;
    break;
  case 4:
    Ta = T0 / 2;
    Tb = T1 + T0 / 2;
    Tc = T1 + T2 + T0 / 2;
    break;
  case 5:
    Ta = T2 + T0 / 2;
    Tb = T0 / 2;
    Tc = T1 + T2 + T0 / 2;
    break;
  case 6:
    Ta = T1 + T2 + T0 / 2;
    Tb = T0 / 2;
    Tc = T1 + T0 / 2;
    break;
  default:
    Ta = 0;
    Tb = 0;
    Tc = 0;
  }

  float Ua = Ta * voltage_power_supply;
  float Ub = Tb * voltage_power_supply;
  float Uc = Tc * voltage_power_supply;

  M0_setPwm(Ua, Ub, Uc);
}

void M1_setTorque(float Uq, float angle_el)
{

  if (Uq < 0)
    angle_el += _PI;
  Uq = fabs(Uq);

  angle_el = _normalizeAngle(angle_el + _PI_2);
  int sector = floor(angle_el / _PI_3) + 1;
  // calculate the duty cycles
  float T1 = _SQRT3 * sin(sector * _PI_3 - angle_el) * Uq / voltage_power_supply;
  float T2 = _SQRT3 * sin(angle_el - (sector - 1.0) * _PI_3) * Uq / voltage_power_supply;
  float T0 = 1 - T1 - T2;

  float Ta, Tb, Tc;
  switch (sector)
  {
  case 1:
    Ta = T1 + T2 + T0 / 2;
    Tb = T2 + T0 / 2;
    Tc = T0 / 2;
    break;
  case 2:
    Ta = T1 + T0 / 2;
    Tb = T1 + T2 + T0 / 2;
    Tc = T0 / 2;
    break;
  case 3:
    Ta = T0 / 2;
    Tb = T1 + T2 + T0 / 2;
    Tc = T2 + T0 / 2;
    break;
  case 4:
    Ta = T0 / 2;
    Tb = T1 + T0 / 2;
    Tc = T1 + T2 + T0 / 2;
    break;
  case 5:
    Ta = T2 + T0 / 2;
    Tb = T0 / 2;
    Tc = T1 + T2 + T0 / 2;
    break;
  case 6:
    Ta = T1 + T2 + T0 / 2;
    Tb = T0 / 2;
    Tc = T1 + T0 / 2;
    break;
  default:
    Ta = 0;
    Tb = 0;
    Tc = 0;
  }

  float Ua = Ta * voltage_power_supply;
  float Ub = Tb * voltage_power_supply;
  float Uc = Tc * voltage_power_supply;

  M1_setPwm(Ua, Ub, Uc);
}

//通过编码器值获得电角度
float S0_electricalAngle(void) 
{
	return _normalizeAngle((float)(M0_DIR * M0_PP) * bsp_as5600_1_GetAngle() - S0_zero_electric_angle);
}

float S1_electricalAngle(void) 
{
	return _normalizeAngle((float)(M1_DIR * M1_PP) * bsp_as5600_2_GetAngle() - S1_zero_electric_angle);
}

//通过编码器值获得转速
float s0_GetVelocity(void)
{

    float M0_Angle = bsp_as5600_1_GetAngle(); // 当前角度
    float d_angle_0 = M0_Angle - M0_Angle_pre;

    // 累计角度变化
    M0_Angle_acc += d_angle_0;

    // 获取当前时间
    uint64_t Time_Now_0 = micro_5us();

    // 初始化时间起点（仅在第一次进入时执行）
    if (T_start0 == 0) {
        T_start0 = Time_Now_0;
        M0_Angle_pre = M0_Angle; // 更新历史角度
        return Vel0_pre;         // 返回初始速度
    }

    // 计算累计时间间隔
    float Ts = (Time_Now_0 - T_start0) * 5e-6f; // 转换为秒

    // 如果时间间隔足够长，计算速度
    float vel;
    if (Ts >= 0.000005f) { // 时间窗口 0.005 ms，采样频率200kHZ
        vel = M0_Angle_acc / Ts; // 累计角度变化计算速度
        Vel0_pre = vel;          // 更新速度
        M0_Angle_acc = 0.0f;     // 清空累计角度
        T_start0 = Time_Now_0;      // 重置时间起点
    } 
	else
	{
        vel = Vel0_pre; // 时间不足时，返回之前的速度
    }

    // 更新历史角度
    M0_Angle_pre = M0_Angle;

    return vel; // 返回速度
}


float s1_GetVelocity(void)
{

    float M1_Angle = bsp_as5600_2_GetAngle(); // 当前角度
    float d_angle_1 = M1_Angle - M1_Angle_pre;

    // 累计角度变化
    M1_Angle_acc += d_angle_1;

    // 获取当前时间
    uint64_t Time_Now_1 = micro_5us();

    // 初始化时间起点（仅在第一次进入时执行）
    if (T_start1 == 0) {
        T_start1 = Time_Now_1;
        M1_Angle_pre = M1_Angle; // 更新历史角度
        return Vel1_pre;         // 返回初始速度
    }

    // 计算累计时间间隔
    float Ts = (Time_Now_1 - T_start1) * 5e-6f; // 转换为秒

    // 如果时间间隔足够长，计算速度
    float vel;
    if (Ts >= 0.000005f) { // 时间窗口 0.005 ms
        vel = M1_Angle_acc / Ts; // 累计角度变化计算速度
        Vel1_pre = vel;          // 更新速度
        M1_Angle_acc = 0.0f;     // 清空累计角度
        T_start1 = Time_Now_1;      // 重置时间起点
    } else {
        vel = Vel1_pre; // 时间不足时，返回之前的速度
    }

    // 更新历史角度
    M1_Angle_pre = M1_Angle;

    return vel; // 返回速度
}

//滤波后的速度
float s0_GetVelocity_Filtered(void)
{
	return LowPass_Filter(s0_GetVelocity() , &Vel_LowPass_Filter_M0);
}

float s1_GetVelocity_Filtered(void)
{
	return LowPass_Filter(s1_GetVelocity() , &Vel_LowPass_Filter_M1);
}

void M0_SetVelocity(float Target)
{
	float error_M0 = Target - s0_GetVelocity_Filtered();
	M0_setTorque(PIDController(&Velocity_PID_M0 , error_M0) , S0_electricalAngle());   //速度闭环
}

void M1_SetVelocity(float Target)
{
	float error_M1 = Target - s1_GetVelocity_Filtered();
	M1_setTorque(PIDController(&Velocity_PID_M1 , error_M1) , S1_electricalAngle());   //速度闭环
}

void M0_Set_Velocity_Angle(float Target)
{
	float error_M0_First = Target - bsp_as5600_1_GetAngle();
	float error_M0_Second = PIDController(&Angle_PID_M0 , error_M0_First) - s0_GetVelocity_Filtered();
	M0_setTorque(PIDController(&Velocity_PID_M0 , error_M0_Second),S0_electricalAngle());   //角度-速度闭环
}

void M1_Set_Velocity_Angle(float Target)
{
	float error_M1_First = Target - bsp_as5600_2_GetAngle();
	float error_M1_Second = PIDController(&Angle_PID_M1 , error_M1_First) - s1_GetVelocity_Filtered();
	M1_setTorque(PIDController(&Velocity_PID_M1 , error_M1_Second),S1_electricalAngle());   //角度-速度闭环
}

void M0_set_Force_Angle(float Target)   //力位
{
	float error_M0 = Target - bsp_as5600_1_GetAngle();
    M0_setTorque(PIDController(&Angle_PID_M0 , error_M0),S0_electricalAngle());
}

void M1_set_Force_Angle(float Target)   //力位
{
	float error_M1 = Target - bsp_as5600_2_GetAngle();
    M1_setTorque(PIDController(&Angle_PID_M1 , error_M1),S1_electricalAngle());
}

void set_MAX_Velocity_M0(float MAX_Velocity_M0)
{
	Angle_PID_M0.limit = MAX_Velocity_M0;
}

void set_MAX_Velocity_M1(float MAX_Velocity_M1)
{
	Angle_PID_M1.limit = MAX_Velocity_M1;
}

void set_MAX_Acceleration_M0(float MAX_Acceleration_M0)
{
	Velocity_PID_M0.limit = MAX_Acceleration_M0;
}

void set_MAX_Acceleration_M1(float MAX_Acceleration_M1)
{
	Velocity_PID_M1.limit = MAX_Acceleration_M1;
}
