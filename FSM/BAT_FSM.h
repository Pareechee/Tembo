#ifndef BAT_FSM_H
#define BAT_FSM_H


#include "FSM_typedef.h"
#include "BQ.h"
#include "GPIO.h"
#include "stdbool.h"
#include "config.h"


EVENT_t get_BAT_event(void);
void BAT_FSM_tick(void);
STATE_t BAT_FLASH_state(void);


#endif /* BAT_FSM_H */
