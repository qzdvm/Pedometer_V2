#ifndef SOFT_TIMER_H
#define SOFT_TIMER_H

#include "stdint.h"
#include "stdbool.h"

/**
 * \def TIME_OVER
 * \brief check if time over in max 24.8 days acc.to ms
 *
 */
#define TIME_OVER(target,time) ((uint32_t)((time) - (target)) < 0x80000000U)
#define TIME_OVER_U16(target,time) ((uint16_t)((time) - (target)) < 0x8000U) // max target is 32768

typedef struct
{
	bool oIn;
	bool oAux;
	void(*fp)(void);
	uint32_t dwInterval;
	uint32_t dwSince;
} soft_timer_t;

typedef struct
{
	bool oAux;
	uint32_t dwSince;
} ton_t;

void timer_check(soft_timer_t *s, uint32_t pdwNow);
void timer_start(soft_timer_t *s);
void timer_stop(soft_timer_t *s);
void timer_set(soft_timer_t *s, uint32_t dwinterval, void(*cb)(void));
bool TON(ton_t *s, bool oIn, uint32_t pdwNow, uint32_t dwPresetTime);
bool TON_16U(ton_t *s, bool oIn, uint32_t pdwNow, uint32_t dwPresetTime);


#endif /* SOFT_TIMER_H */
