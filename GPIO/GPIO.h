#ifndef __GPIO_H
#define __GPIO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <ch32v00x.h>
#include "stdbool.h"


#define BAT_PORT GPIOC
#define BAT_Pin  GPIO_Pin_0

//�����٧ާ�اߧ� ����ڧ� �ڧ���ݧ�٧�ӧѧ��
//#define BAT_Pin  GPIO_Pin_3

#define ESM_PORT GPIOD
#define ESM_Pin  GPIO_Pin_2

#define ROUTER_PORT GPIOC
#define ROUTER_Pin  GPIO_Pin_7

#define LED_PORT GPIOD
#define LED_Pin  GPIO_Pin_3

#define BAT_ON  GPIO_WriteBit(BAT_PORT, BAT_Pin, Bit_SET)
#define BAT_OFF GPIO_WriteBit(BAT_PORT, BAT_Pin, Bit_RESET)

#define ESM_ON  GPIO_WriteBit(ESM_PORT, ESM_Pin, Bit_SET)
#define ESM_OFF GPIO_WriteBit(ESM_PORT, ESM_Pin, Bit_RESET)

#define ROUTER_ON  GPIO_WriteBit(ROUTER_PORT, ROUTER_Pin, Bit_SET)
#define ROUTER_OFF GPIO_WriteBit(ROUTER_PORT, ROUTER_Pin, Bit_RESET)

#define LED_TOGGLE(i) GPIO_WriteBit(LED_PORT, LED_Pin, ((i) == 0) ? ((i) = Bit_SET) : ((i) = Bit_RESET))

#define LED_ON GPIO_WriteBit(LED_PORT, LED_Pin, Bit_SET)
#define LED_OFF GPIO_WriteBit(LED_PORT, LED_Pin, Bit_RESET)


void GPIO_periph_init(void);
void GPIO_LED_toggle(void);
void GPIO_LED_ON(void);
void GPIO_LED_OFF(void);


#ifdef __cplusplus
}
#endif

#endif /* __GPIO_H */
