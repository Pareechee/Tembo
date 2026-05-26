#ifndef FSM_TYPEDEF_H
#define FSM_TYPEDEF_H


typedef enum
{
    EVENT_OK = 0,
    EVENT_LOW,
    EVENT_ERR,
    EVENT_MAX
} EVENT_t;


typedef enum
{
    STATE_OK = 0,
    STATE_LOW,
    STATE_ERR,
    STATE_TIMEOUT,
    STATE_MAX
} STATE_t;


#endif /* FSM_TYPEDEF_H */
