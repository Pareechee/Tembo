#include "GPIO.h"



void GPIO_periph_init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_GPIOD | RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = BAT_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(BAT_PORT, &GPIO_InitStructure);
    GPIO_WriteBit(BAT_PORT, BAT_Pin, Bit_SET);

    GPIO_InitStructure.GPIO_Pin = ESM_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(ESM_PORT, &GPIO_InitStructure);
    GPIO_WriteBit(ESM_PORT, ESM_Pin, Bit_RESET);

    GPIO_InitStructure.GPIO_Pin = ROUTER_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(ROUTER_PORT, &GPIO_InitStructure);
    GPIO_WriteBit(ROUTER_PORT, ROUTER_Pin, Bit_RESET);

    GPIO_InitStructure.GPIO_Pin = LED_Pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(LED_PORT, &GPIO_InitStructure);
    GPIO_WriteBit(LED_PORT, LED_Pin, Bit_RESET);
    
    // 3. §¯§Ñ§ã§ä§â§Ñ§Ú§Ó§Ñ§Ö§Þ PD4 (RE)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // 4. §£§½§³§´§¡§£§­§Á§¦§® §£§½§³§°§¬§ª§« §µ§²§°§£§¦§¯§¾
    //GPIO_WriteBit(GPIOA, GPIO_Pin_1, Bit_SET); // §¯§Ñ PA1 §Ò§å§Õ§Ö§ä 3.3§£
    GPIO_WriteBit(GPIOD, GPIO_Pin_4, Bit_SET); // §¯§Ñ PD4 §Ò§å§Õ§Ö§ä 3.3§£

}

uint8_t flag = 0;
void GPIO_LED_toggle(void)
{
    LED_TOGGLE(flag);
}

void GPIO_LED_ON(void){
    LED_ON;
}

void GPIO_LED_OFF(void){
    LED_OFF;
}
