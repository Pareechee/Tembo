#ifndef FLASH_TIMER_H
#define FLASH_TIMER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ch32v00x.h>
#include <stdio.h>
#include <stdbool.h>

#define INTERVAL 50;

void FlashTimer_init(void);

#ifdef __cplusplus
}
#endif

#endif /* FLASH_TIMER_H */
