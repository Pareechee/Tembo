#include "USART.h"


void USARTx_CFG(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure = {0};
    USART_InitTypeDef USART_InitStructure = {0};

    // 1. ����ҧѧӧݧ�֧� RCC_APB2Periph_AFIO (��ҧ�٧ѧ�֧ݧ�ߧ� �էݧ� ��֧ާѧ��!)
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD | RCC_APB2Periph_USART1 | RCC_APB2Periph_AFIO, ENABLE);

    // 2. ���ܧݧ��ѧ֧� �٧֧�ܧѧݧ�ߧ��� ��֧ާѧ� (TX->PD6, RX->PD5)
    GPIO_PinRemapConfig(GPIO_PartialRemap2_USART1, ENABLE);

    /* USART1 TX-->D.6 */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6; // PD6 ��֧�֧�� TX
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    /* USART1 RX-->D.5 */
    // SCM3406ASA: при DE/RE=1 (TX) приёмник в Z. Pull-up удерживает idle high
    // и предотвращает ложные ORE/FE на USART1
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
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

