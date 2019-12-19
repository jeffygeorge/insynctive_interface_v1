#ifndef TIMER_H_
#define TIMER_H_

#include <time.h>
#include "EventQ.h"

void TimerStart(timer_t timerId, int sec, int msec, int it_sec, int it_msec);
void TimerStop(timer_t timerId);
void TimerDisable(timer_t timerId);
void TimerInit(timer_t *timerId, enum EventType evt /* = TIMER_EVENT */);
bool TimerIsActive(timer_t timerId);

#endif
