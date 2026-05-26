#ifndef TIMER_H
#define TIMER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ch32v00x.h>
#include <stdio.h>
#include <stdbool.h>

void Timer_init(void);
bool Timer_tick();

#ifdef __cplusplus
}
#endif

#endif /* TIMER_H */
