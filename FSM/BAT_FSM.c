#include "BAT_FSM.h"
#include "HV_FSM.h"
#include "BQ.h"

volatile STATE_t BAT_state = STATE_ERR;
volatile EVENT_t BAT_event = EVENT_ERR;


EVENT_t get_BAT_event(void)
{
    BQ_ReceiveData(ADCVSYSVBAT);
    // §¢§Ö§â§Ö§Þ §´§°§­§¾§¬§° §Þ§Ý§Ñ§Õ§ê§Ú§Û §Ò§Ñ§Û§ä (§Ú§Þ§Ö§ß§ß§à §Ó §ß§Ö§Þ §Ý§Ö§Ø§Ñ§ä §Õ§Ñ§ß§ß§í§Ö §¡§¸§± VBAT)
    uint8_t vbat_reg = (uint8_t)(REG_getVal(ADCVSYSVBAT) & 0xFF); 

    // §°§ä§Ý§Ñ§Õ§à§é§ß§í§Û §Ó§í§Ó§à§Õ, §é§ä§à§Ò§í §å§Ó§Ú§Õ§Ö§ä§î §²§¦§¡§­§¾§¯§°§¦ §é§Ú§ã§Ý§à §Õ§Ý§ñ §ã§â§Ñ§Ó§ß§Ö§ß§Ú§ñ
    //printf("Comparing: %02x with threshold: %02x\r\n", vbat_reg, VBAT_SHUTDOWN_THRESHOLD);

    if (vbat_reg < VBAT_SHUTDOWN_THRESHOLD) {
        return EVENT_ERR; 
    }
    return EVENT_OK;
}




bool check_BAT_charge(void)
{
    if (BQ_CHARGE_monitor() != BQ_HV_OK)
    {
        return false;
    }
    return true;
}


void BAT_OK(void) {}


void BAT_OK_to_LOW(void)
{
    ROUTER_OFF;
}


void BAT_OK_to_ERR(void)
{
    ROUTER_OFF;
    BAT_OFF;
}


void BAT_LOW_to_OK(void)
{
    if (check_BAT_charge())
    {
        ROUTER_ON;
    }

}


void BAT_LOW(void)
{
    ROUTER_OFF;
}


void BAT_LOW_to_ERR(void)
{
    BAT_OFF;
}


void BAT_ERR_to_OK(void)
{
    BAT_ON;
    if (check_BAT_charge())
    {
        ROUTER_ON;
    }
}


void BAT_ERR_to_LOW(void)
{
    BAT_ON;
}


void BAT_ERR(void)
{

//printf("!!! TRIGGER SHUTDOWN !!!\r\n");
    // §°§ä§Ü§Ý§ð§é§Ñ§Ö§Þ §ß§Ñ§Ô§â§å§Ù§Ü§Ú
    ROUTER_OFF;
    ESM_OFF;
    
    // §©§Ñ§Õ§Ö§â§Ø§Ü§Ñ, §é§ä§à§Ò§í §ß§Ñ§Ô§â§å§Ù§Ü§Ú §å§ã§á§Ö§Ý§Ú §à§Ò§Ö§ã§ä§à§é§Ú§ä§î§ã§ñ
    Delay_Ms(500); 
    
    // §©§¡§¬§²§½§£§¡§¦§® VT16 (§á§Ú§ß PC0 §Ó §ß§à§Ý§î)
    // §³§Ú§ã§ä§Ö§Þ§Ñ §á§à§Ý§ß§à§ã§ä§î§ð §à§Ò§Ö§ã§ä§à§é§Ú§ä §ã§Ñ§Þ§Ñ §ã§Ö§Ò§ñ
    GPIO_WriteBit(GPIOC, GPIO_Pin_0, Bit_RESET); 
}



void BAT_debug_print(void) {
   // printf("VBAT: %04x\r\n", REG_getVal(ADCVSYSVBAT));
   // printf("BAT: ");
    switch (BAT_state) {
    case STATE_OK:
       // printf("OK\r\n");
        break;
    case STATE_LOW:
       // printf("LOW\r\n");
        break;
    case STATE_ERR:
       // printf("ERR\r\n");
        break;
    default:
        break;
    }
}


void (*BAT_transitin_table[STATE_MAX][EVENT_MAX]) (void) =
{
        [STATE_OK][EVENT_OK]   = BAT_OK,
        [STATE_OK][EVENT_LOW]  = BAT_OK_to_LOW,
        [STATE_OK][EVENT_ERR]  = BAT_OK_to_ERR,

        [STATE_LOW][EVENT_OK]  = BAT_LOW_to_OK,
        [STATE_LOW][EVENT_LOW] = BAT_LOW,
        [STATE_LOW][EVENT_ERR] = BAT_LOW_to_ERR,

        [STATE_ERR][EVENT_OK]  = BAT_ERR_to_OK,
        [STATE_ERR][EVENT_LOW] = BAT_ERR_to_LOW,
        [STATE_ERR][EVENT_ERR] = BAT_ERR
};


void BAT_FSM_tick(void)
{
    BAT_event = get_BAT_event();
    BAT_transitin_table[BAT_state][BAT_event]();
    
    // §ª§ã§á§â§Ñ§Ó§Ý§Ö§ß§ß§Ñ§ñ §ã§ä§â§à§Ü§Ñ (§Õ§à§Ò§Ñ§Ó§Ý§Ö§ß§à §ñ§Ó§ß§à§Ö §á§â§Ú§Ó§Ö§Õ§Ö§ß§Ú§Ö §ä§Ú§á§à§Ó):
    BAT_state = (STATE_t)BAT_event; 

#ifdef DEBUG_ON
    BAT_debug_print();
#endif
}


STATE_t BAT_FLASH_state(void){
    return BAT_state;
}
