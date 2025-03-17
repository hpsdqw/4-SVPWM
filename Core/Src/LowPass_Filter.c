#include "LowPass_Filter.h"

//低通滤波器，x为传入数据，Tf为滤波器时间常数
float LowPass_Filter(float x , LowPass_Filter_Parameters * LowPass_Filter_Structure)
{
    uint64_t timestamp = micro_5us();
    float dt = (timestamp - LowPass_Filter_Structure->timestamp_prev)*5e-6f;
    if (dt < 0.0f ) dt = 1e-3f;
    else if(dt > 0.3f) {
        LowPass_Filter_Structure->y_prev = x;
        LowPass_Filter_Structure->timestamp_prev = timestamp;
        return x;
    }

    float alpha = LowPass_Filter_Structure->Tf/(LowPass_Filter_Structure->Tf + dt);
    float y = alpha*LowPass_Filter_Structure->y_prev + (1.0f - alpha)*x;
    LowPass_Filter_Structure->y_prev = y;
    LowPass_Filter_Structure->timestamp_prev = timestamp;
    return y;
}
