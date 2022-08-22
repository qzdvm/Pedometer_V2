#ifndef SOFT_TIMER_H
#define SOFT_TIMER_H

#include "stdint.h"
#include "stdbool.h"

enum 
{
	TON_STEP = 0,
	TON_WATCH_RTC_WAKEUP,
	TON_AS3933_TIMEOUT,
	TON_END,
};

#define SOFT_TIMER_OBJ_SIZE	  1
#define TON_OBJ_SIZE					TON_END

void timer_check(uint8_t b_id, uint32_t dw_now);
void timer_start(uint8_t b_id);
void timer_stop(uint8_t b_id);
void timer_set(uint8_t b_id, uint32_t dw_interval, void(*cb)(void));
bool TON(uint8_t b_id, bool o_in, uint32_t dw_now, uint32_t dw_preset_time);
bool TON_16U(uint8_t b_id, bool o_in, uint32_t dw_now, uint32_t dw_preset_time);


#endif /* SOFT_TIMER_H */
