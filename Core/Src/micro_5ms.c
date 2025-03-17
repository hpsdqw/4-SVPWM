#include "micro_5ms.h"

extern uint64_t REAL_TIME;
uint64_t micro_5us(void)
{
	return REAL_TIME ;
}


