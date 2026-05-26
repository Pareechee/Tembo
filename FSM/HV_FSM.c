#include "HV_FSM.h"


volatile STATE_t HV_state = STATE_ERR;
volatile EVENT_t HV_event = EVENT_ERR;

uint32_t HV_error_counter = 0;
FlagStatus HV_error_flag = SET;

uint8_t HV_config_counter = 0;
FlagStatus HV_config_flag = SET;


EVENT_t get_HV_event(void)
{
    if (BQ_STAT_monitor() == BQ_HV_OK)
    {
        return EVENT_OK;
    }
    return EVENT_ERR;
}


void check_BQ_CURR(void)
{
    if (BQ_CURR_monitor() != BQ_HV_OK)
    {
        HV_config_flag = SET;
    }
}


void HV_OK(void)
{
    check_BQ_CURR();
    if (HV_config_flag)
    {
        if (HV_config_counter >= HV_CONFIG_TIMEOUT)
        {
            BQ_config();
            ESM_ON;
            ROUTER_ON;
            HV_config_counter = 0;
            HV_config_flag = RESET;
        }
        else
        {
            HV_config_counter++;
        }
    }
}


void HV_OK_to_ERR(void)
{
#ifdef DEBUG_ON
    printf("HV  IS TURNED OFF, SWITCHING TO THE BATTERY SOURCE, SENDING SMS\r\n");
    //������ѧӧݧ�֧� ������ݧܧ�, �ܧ����ѧ� �ӧ��٧�ӧ֧� �����ѧӧܧ� ��ާ�, �٧է֧�� ��ܧѧ٧ѧ� ��֧ܧ���ӧ��� ���ڧާ֧�
#endif
    HV_error_counter = 0;
    HV_error_flag = SET;

}


void HV_ERR_to_OK(void)
{
    HV_config_flag = SET;
    HV_error_counter = 0;
    HV_config_counter = 0;
}

void HV_ERR(void) {
    if (HV_error_flag) {
        if (HV_error_counter >= HV_ERROR_TIMEOUT) {
            ESM_OFF;
            ROUTER_OFF;
            HV_error_flag = RESET;
            HV_error_counter = 0;
        } else {
            HV_error_counter++;
        }
        /*if(HV_error_counter >= HV_SMS_TIMEOUT){
            �� ���ڧާ֧�� ���ӧ���ߧ�� �����ѧӧܧ� ��ާ� �֧�ݧ� hv �ߧ� �٧ѧ�ѧҧ��ѧ�,
            ������� �ܧѧܧ��-��� �ӧ�֧ާ�.
        }*/
    }
}


void (*HV_transitin_table[STATE_MAX][EVENT_MAX])(void) =
{
    [STATE_OK][EVENT_OK] = HV_OK,
    [STATE_OK][EVENT_ERR] = HV_OK_to_ERR,

    [STATE_ERR][EVENT_OK] = HV_ERR_to_OK,
    [STATE_ERR][EVENT_ERR] = HV_ERR
};


void HV_debug_print(void) {
   // printf("HV: ");
    switch (HV_state) {
    case STATE_OK:
       // printf("OK\r\n");
        break;
    case STATE_ERR:
       // printf("ERR\r\n");
       // printf("HV err count: %d\r\n", HV_error_counter);
        break;
    default:
        break;
    }
}


void HV_FSM_tick(void)
{
    HV_event = get_HV_event();

    HV_transitin_table[HV_state][HV_event]();
    HV_state = (STATE_t)HV_event;

#ifdef DEBUG_ON
    HV_debug_print();
#endif
}

STATE_t HV_FLASH_state(void){
    return HV_state;
}
