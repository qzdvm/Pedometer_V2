#ifndef UTILS_H
#define UTILS_H

#include "stdbool.h"
#include "stdint.h"

typedef struct
{
	bool o_aux;
}rising_edge_detection_t;

bool o_rising_edge_detection(rising_edge_detection_t *s, bool o_catch);

#endif
