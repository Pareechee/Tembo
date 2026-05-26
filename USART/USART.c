#include "USART.h"


void USARTx_CFG(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure = {0};
    USART_InitTypeDef USART_InitStructure = {0};

    // 1. §¥§à§Ò§Ñ§Ó§Ý§ñ§Ö§Þ RCC_APB2Periph_AFIO (§à§Ò§ñ§Ù§Ñ§ä§Ö§Ý§î§ß§à §Õ§Ý§ñ §â§Ö§Þ§Ñ§á§Ñ!)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO, ENABLE);

    // 2. §£§Ü§Ý§ð§é§Ñ§Ö§Þ §Ù§Ö§â§Ü§Ñ§Ý§î§ß§í§Û §â§Ö§Þ§Ñ§á (TX->PD6, RX->PD5)
    GPIO_PinRemapConfig(GPIO_PartialRemap2_USART1, ENABLE);

    /* USART1 TX-->D.6 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; // PD6 §ä§Ö§á§Ö§â§î TX
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /* USART1 RX-->D.5 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; // PD5 §ä§Ö§á§Ö§â§î RX
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate = 9600;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_Parity = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

    USART_Init(USART1, &USART_InitStructure);
    USART_Cmd(USART1, ENABLE);
}

