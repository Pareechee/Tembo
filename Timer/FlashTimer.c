#include "FlashTimer.h"

void FlashTimer_init(void) {
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);// 我扶我扯我忘抖我戒忘扯我攸 找忘抄技快把忘
    TIM_CounterModeConfig(TIM1, TIM_CounterMode_Up);// 把快忪我技 扭把攸技抉忍抉 扼折快找忘
    TIM_SetAutoreload(TIM1, 50);// 扶忘扼找把抉抄抗忘 忘志找抉扭快把快戒忘忍把批戒抗我
    TIM_PrescalerConfig(TIM1, 48000 - 1, TIM_PSCReloadMode_Immediate); //扭把快扼抗忘抖抖快把
    TIM_Cmd(TIM1, ENABLE); //把忘戒把快扮快扶我快 把忘忌抉找抑 找忘抄技快把忘
    }
