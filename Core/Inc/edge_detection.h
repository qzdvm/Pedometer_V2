#ifndef EDGE_DETECTION_H
#define EDGE_DETECTION_H

#include "stdbool.h"
#include "stdint.h"

enum 
{
	ED_STEP = 0,
	ED_CL_DAT,
	ED_END,
};


#define ED_OBJ_NUM	ED_END

bool edge_detection(uint8_t b_id, bool o_catch);

#endif
