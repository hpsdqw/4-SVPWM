#include "PID.h"
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

//输入pid参数和差值，输出本次应输出的值
float PIDController(PIDparameters * PIDSetting , float error){
    // 计算两次循环中间的间隔时间
   uint64_t timestamp_now = micro_5us();
    float Ts = (timestamp_now - PIDSetting->timestamp_prev) * 5e-6f;
        
    // P环
    float proportional = PIDSetting->P * error;
    // 散点积分（I环）
    float integral = PIDSetting->integral_prev + PIDSetting->I*Ts*0.5f*(error + PIDSetting->error_prev);
    integral = _constrain(integral, -PIDSetting->limit, PIDSetting->limit);
    // D环（带滤波的微分）
    float delta_error = error - PIDSetting->error_prev;
	float numerator = PIDSetting->D * PIDSetting->N * delta_error;
	float denominator = PIDSetting->N + Ts;

	// 计算当前D环输出（含低通滤波）
	float D_term = (numerator + PIDSetting->D_filter_prev * Ts) / denominator;

	// 更新滤波状态
	PIDSetting->D_filter_prev = D_term;

	// 最终微分项
	float derivative = D_term;

    // 将P,I,D三环的计算值加起来
    float output = proportional + integral + derivative;
    output = _constrain(output, -PIDSetting->limit, PIDSetting->limit);

    if(PIDSetting->output_ramp > 0){
        // 对PID的变化速率进行限制
        float output_rate = (output - PIDSetting->output_prev)/Ts;
        if (output_rate > PIDSetting->output_ramp)
            output = PIDSetting->output_prev + PIDSetting->output_ramp*Ts;
        else if (output_rate < -PIDSetting->output_ramp)
            output = PIDSetting->output_prev - PIDSetting->output_ramp*Ts;
    }
    // 保存值（为了下一次循环）
    PIDSetting->integral_prev = integral;
    PIDSetting->output_prev = output;
    PIDSetting->error_prev = error;
    PIDSetting->timestamp_prev = timestamp_now;
    return output;
}
