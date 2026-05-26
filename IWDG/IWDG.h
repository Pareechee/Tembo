#ifndef IWDG_H_
#define IWDG_H_

#include "ch32v00x_iwdg.h"

void IWDG_Feed_Init(u16 prer, u16 rlr);
void IWDG_Feed(void);

#endif /* IWDG_H_ */
