/*
 * HV_FSM.h
 *
 *  Created on: Feb 15, 2024
 *      Author: r19
 */

#ifndef HV_FSM_H
#define HV_FSM_H

#include "FSM_typedef.h"
#include "BQ.h"
#include "GPIO.h"
#include "stdbool.h"
#include "config.h"


void HV_FSM_tick(void);
STATE_t HV_FLASH_state(void);


#endif /* HV_FSM_H */
