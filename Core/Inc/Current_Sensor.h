#ifndef __CURRENT_SENSOR_H
#define __CURRENT_SENSOR_H

#include "adc.h"

void Current_CalibrateOffsets(void);
void getPhaseCurrents(void);



extern float offset_ia_1;
extern float offset_ib_1;

extern float Current_a_1;
extern float Current_b_1;
extern float Current_c_1;

#endif
