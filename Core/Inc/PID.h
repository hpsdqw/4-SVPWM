#ifndef __PID_H
#define __PID_H

#include <stdint.h>
#include "micro_5ms.h"

typedef struct
{
	float P;
	float I;
	float D;
	float output_ramp;    // PID控制器加速度限幅
    float limit;         // PID控制器输出限幅
	float error_prev;
    float output_prev;
    float integral_prev;
	float N;          // 滤波时间常数N
    float D_filter_prev; // 保存上一次D环滤波输出
	uint64_t timestamp_prev;
} PIDparameters;


float PIDController(PIDparameters * PIDSetting , float error);

#endif
