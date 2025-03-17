#include "Current_Sensor.h"

float R = 0.01;
float amp_gain = 50.0;

void Current_CalibrateOffsets(void)
{
    const int calibration_rounds = 1000;

    // 查找0电流时候的电压
    // 读数1000次
    for (int i = 0; i < calibration_rounds; i++) 
	{
        offset_ia_1 += HAL_ADC_GetValue(&hadc1);
        offset_ib_1 += HAL_ADC_GetValue(&hadc2);
        HAL_Delay(1);
    }
    // 求平均，得到误差
    offset_ia_1 = offset_ia_1 / calibration_rounds;
    offset_ib_1 = offset_ib_1 / calibration_rounds;
}

void getPhaseCurrents(void)
{
    Current_a_1 = (HAL_ADC_GetValue(&hadc1) - offset_ia_1)*amp_gain/R;
    Current_b_1 = (HAL_ADC_GetValue(&hadc2) - offset_ib_1)*amp_gain/R;
    Current_c_1 = -Current_a_1-Current_b_1; // amps
}

