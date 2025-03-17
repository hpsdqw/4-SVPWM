#ifndef __LOWPASS_FILTER_H
#define __LOWPASS_FILTER_H

#include "micro_5ms.h"

typedef struct
{
	uint64_t timestamp_prev;
	float y_prev;
	float Tf;
} LowPass_Filter_Parameters;

float LowPass_Filter(float x , LowPass_Filter_Parameters * LowPass_Filter_Structure);

#endif
