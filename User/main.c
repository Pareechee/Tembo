#include "HAL_I2C.h"
#include "BQ.h"
#include "IWDG.h"
#include "GPIO.h"
#include "Timer.h"
#include "FlashTimer.h"
#include "BAT_FSM.h"
#include "HV_FSM.h"
#include "USART.h"
#include "config.h"
#include "TableFlash.h"
#include "debug.h"
uint8_t last_power_status = 0;

// --- §±§²§°§´§°§´§ª§±§½ §¯§¡§º§ª§· §¶§µ§¯§¬§¸§ª§« ---
void Print_Readable_Telemetry(void);
uint16_t Calculate_CRC16(uint8_t *buffer, uint16_t buffer_length);
uint8_t Modbus_Master_Write_Register(uint8_t slave_id, uint16_t reg_addr, uint16_t data_val);



// --- §´§¡§¢§­§ª§¸§½ §¥§­§Á §¢§½§³§´§²§°§¤§° §²§¡§³§¹§¦§´§¡ CRC-16 MODBUS ---
static const uint8_t table_crc_hi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40
};

static const uint8_t table_crc_lo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04,
    0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8,
    0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
    0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10,
    0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4,
    0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38,
    0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C,
    0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0,
    0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
    0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68,
    0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xB7, 0xB6, 0x76,
    0xB4, 0x74, 0x75, 0xB4, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
    0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54,
    0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98,
    0x88, 0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40
};

int main(void)
{
    int FlashStateHV = 2;
    int FlashStateBAT = 2;
    int currentCode = 0;
    int stepCode = 0;
    
    // §±§Ö§â§Ö§Þ§Ö§ß§ß§Ñ§ñ §Õ§Ý§ñ §Ù§Ñ§Õ§Ö§â§Ø§Ü§Ú §Ó§Ü§Ý§ð§é§Ö§ß§Ú§ñ §Ù§Ñ§ë§Ú§ä§í
    uint32_t startup_protection_delay = 0;

    // §·§â§Ñ§ß§Ö§ß§Ú§Ö §á§â§Ö§Õ§í§Õ§å§ë§Ö§Ô§à §ã§à§ã§ä§à§ñ§ß§Ú§ñ §á§Ú§ä§Ñ§ß§Ú§ñ (0 - §³§Ö§ä§î §°§¬, 1 - §¡§Ó§Ñ§â§Ú§ñ §á§Ú§ä§Ñ§ß§Ú§ñ)
    

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SystemCoreClockUpdate();
    Systick_Init();
    Timer_init();
    FlashTimer_init();
    
    // 1. §ª§ß§Ú§è§Ú§Ñ§Ý§Ú§Ù§Ú§â§å§Ö§Þ GPIO (§Ó§ß§å§ä§â§Ú §Õ§à§Ý§Ø§Ö§ß §Ò§í§ä§î BAT_ON §ß§Ñ PC0)
    GPIO_periph_init(); 
    
    // §±§â§Ú§ß§å§Õ§Ú§ä§Ö§Ý§î§ß§à §á§à§Õ§ä§Ó§Ö§â§Ø§Õ§Ñ§Ö§Þ §Ó§Ü§Ý§ð§é§Ö§ß§Ú§Ö §Ò§Ñ§ä§Ñ§â§Ö§Ú §á§Ö§â§Ö§Õ §ã§ä§Ñ§â§ä§à§Þ BQ
    GPIO_WriteBit(GPIOC, GPIO_Pin_0, Bit_SET); 

    USARTx_CFG();

//#ifdef DEBUG_ON
    //printf("System Start...\r\n");
    //printf("SystemClk:%d\r\n", SystemCoreClock);
   // printf("ChipID:%08x\r\n", DBGMCU_GetCHIPID());
//#endif

    // 2. §ª§ß§Ú§è§Ú§Ñ§Ý§Ú§Ù§Ú§â§å§Ö§Þ BQ (§ä§Ö§á§Ö§â§î, §Ü§à§Ô§Õ§Ñ §á§Ú§ä§Ñ§ß§Ú§Ö §ã§ä§Ñ§Ò§Ú§Ý§Ú§Ù§Ú§â§à§Ó§Ñ§Ý§à§ã§î)
    BQ_Init();
    
    IWDG_Feed_Init(IWDG_Prescaler_128, 4000);

while(1)
{
    // §´§Ñ§Û§Þ§Ö§â §ä§Ú§Ü§Ñ§Ö§ä §Ü§Ñ§Ø§Õ§å§ð §ã§Ö§Ü§å§ß§Õ§å §ß§Ö§Ù§Ñ§Ó§Ú§ã§Ú§Þ§à §à§ä DEBUG_ON
    if(Timer_tick())
    {
        // §£§í§Ù§í§Ó§Ñ§Ö§Þ §ã§Ò§à§â §Õ§Ñ§ß§ß§í§ç §ã BQ (§Ò§Ö§Ù §Ó§í§Ó§à§Õ§Ñ printf §ß§Ñ §ï§Ü§â§Ñ§ß)
        BQ_ReceiveData(ADCVSYSVBAT);
        BQ_ReceiveData(ADCVBUS_PSYS);
        BQ_ReceiveData(ADCIBAT);
        BQ_ReceiveData(ADCIINCMPIN);
        BQ_ReceiveData(CHARGE_STATUS_REGISTER);

// --- §¯§¡§¥§¦§¨§¯§¡§Á §°§¢§²§¡§¢§°§´§¬§¡ §³§°§¢§½§´§ª§« §³ §£§½§£§°§¥§°§® §³§´§¡§´§µ§³§¡ ---
// --- §­§°§¤§ª§¬§¡ §°§´§³§­§¦§¨§ª§£§¡§¯§ª§Á §¡§£§¡§²§ª§ª §³ §°§¨§ª§¥§¡§¯§ª§¦§® §°§´§£§¦§´§¡ (9600 §¢§°§¥) ---
uint16_t raw_vbus = REG_getVal(ADCVBUS_PSYS);
uint32_t vbus_mv = (uint32_t)((raw_vbus / 4) * 1.2673);

if (vbus_mv < 4000) 
{
    // §¦§ã§Ý§Ú §Þ§í §Ö§ë§Ö §ß§Ö §Ù§Ñ§æ§Ú§Ü§ã§Ú§â§à§Ó§Ñ§Ý§Ú §å§ã§á§Ö§ê§ß§í§Û §à§ä§Ó§Ö§ä §à§ä §â§à§å§ä§Ö§â§Ñ §à§Ò §Ñ§Ó§Ñ§â§Ú§Ú
    if (last_power_status == 0) 
    {
        // §°§ä§á§â§Ñ§Ó§Ý§ñ§Ö§Þ §Ü§à§Õ §Ñ§Ó§Ñ§â§Ú§Ú (1). §¦§ã§Ý§Ú §â§à§å§ä§Ö§â §à§ä§Ó§Ö§ä§Ú§ä §Ú CRC §ã§à§Ó§á§Ñ§Õ§Ö§ä, §æ§å§ß§Ü§è§Ú§ñ §Ó§Ö§â§ß§Ö§ä 1
        if (Modbus_Master_Write_Register(0xA1, 0, 1) == 1) 
        {
            last_power_status = 1; // §µ§³§±§¦§·: §²§à§å§ä§Ö§â §á§à§Õ§ä§Ó§Ö§â§Õ§Ú§Ý! §°§ä§á§â§Ñ§Ó§Ü§Ñ §á§â§Ö§Ü§â§Ñ§ë§Ñ§Ö§ä§ã§ñ.
        }
    }
} 
else // vbus_mv >= 4000 (§±§Ú§ä§Ñ§ß§Ú§Ö §Ó§à§ã§ã§ä§Ñ§ß§à§Ó§Ú§Ý§à§ã§î)
{
    // §¦§ã§Ý§Ú §Õ§à §ï§ä§à§Ô§à §Ò§í§Ý§Ñ §Ù§Ñ§æ§Ú§Ü§ã§Ú§â§à§Ó§Ñ§ß§Ñ §Ñ§Ó§Ñ§â§Ú§ñ, §Ü§à§ä§à§â§å§ð §â§à§å§ä§Ö§â §á§à§Õ§ä§Ó§Ö§â§Ø§Õ§Ñ§Ý
    if (last_power_status == 1) 
    {
        // §°§ä§á§â§Ñ§Ó§Ý§ñ§Ö§Þ §Ü§à§Õ §à§ä§Þ§Ö§ß§í §Ñ§Ó§Ñ§â§Ú§Ú (0). §¨§Õ§Ö§Þ §á§à§Õ§ä§Ó§Ö§â§Ø§Õ§Ö§ß§Ú§ñ
        if (Modbus_Master_Write_Register(0xA1, 0, 0) == 1) 
        {
            last_power_status = 0; // §µ§³§±§¦§·: §²§à§å§ä§Ö§â §á§à§Õ§ä§Ó§Ö§â§Õ§Ú§Ý §Ó§à§Ù§Ó§â§Ñ§ä §Ó §ã§Ö§ä§î!
        }
    }
}



        // 3. §­§°§¤§ª§¬§¡ §©§¡§»§ª§´§½ §¢§¡§´§¡§²§¦§ª
        if (startup_protection_delay < 5) {
            startup_protection_delay++;
            GPIO_WriteBit(GPIOC, GPIO_Pin_0, Bit_SET); 
        } else {
            BAT_FSM_tick(); 
        }
        HV_FSM_tick();

        IWDG_Feed();
        FlashStateHV = HV_FLASH_state();
        FlashStateBAT = BAT_FLASH_state();
        currentCode = 0;
    }

    // §¢§Ý§à§Ü §à§Ò§â§Ñ§Ò§à§ä§Ü§Ú TIM1 (§Þ§Ú§Ô§Ñ§ß§Ú§Ö §ã§Ó§Ö§ä§à§Õ§Ú§à§Õ§Ñ) §à§ã§ä§Ñ§Ó§Ý§ñ§Ö§Þ §á§à§Ý§ß§à§ã§ä§î§ð §Ò§Ö§Ù §Ú§Ù§Þ§Ö§ß§Ö§ß§Ú§Û
    if(TIM_GetFlagStatus(TIM1, TIM_FLAG_Update) != RESET)
    {
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
        switch(FlashStateBAT){
            case(0):
                if(!FlashStateHV)
                    stepCode = 0;
                else {
                    stepCode = 1;
                }
                break;
            case(1):
                if(!FlashStateHV)
                    stepCode = 2;
                else {
                    stepCode = 3;
                }
                break;
            case(2):
                if(!FlashStateHV)
                    stepCode = 4;
                else {
                    stepCode = 5;
                }
                break;
        }

        if (blinkCodes[stepCode][currentCode])
        {
            GPIO_LED_ON();
        }
        else
        {
            GPIO_LED_OFF();
        }
        currentCode += 1;
    }
}
}

// --- §¶§µ§¯§¬§¸§ª§Á §±§°§¥§³§¹§¦§´§¡ CRC-16 MODBUS ---
// §±§à§Ý§ß§Ñ§ñ §Ù§Ñ§Þ§Ö§ß§Ñ §Ó§Ñ§ê§Ö§Û §ã§ä§Ñ§â§à§Û §æ§å§ß§Ü§è§Ú§Ú §ß§Ñ §Ü§Ñ§ß§à§ß§Ú§é§Ö§ã§Ü§Ú§Û §â§Ñ§ã§é§Ö§ä
uint16_t Calculate_CRC16(uint8_t *buffer, uint16_t buffer_length) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < buffer_length; i++) {
        crc ^= buffer[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}
 

// --- §¶§µ§¯§¬§¸§ª§Á §°§´§±§²§¡§£§¬§ª MODBUS §±§¡§¬§¦§´§¡ §©§¡§±§ª§³§ª (§¶§µ§¯§¬§¸§ª§Á 06) ---
// --- §±§°§­§¯§°§³§´§¾§À §¯§¦§¢§­§°§¬§ª§²§µ§À§»§¡§Á §¶§µ§¯§¬§¸§ª§Á §°§´§±§²§¡§£§¬§ª MODBUS ---
// §¶§å§ß§Ü§è§Ú§ñ §ä§Ö§á§Ö§â§î §Ó§à§Ù§Ó§â§Ñ§ë§Ñ§Ö§ä 1 (§à§ä§Ó§Ö§ä §á§à§Ý§å§é§Ö§ß) §Ú§Ý§Ú 0 (§ä§Ñ§Û§Þ§Ñ§å§ä)
uint8_t Modbus_Master_Write_Register(uint8_t slave_id, uint16_t reg_addr, uint16_t data_val)
{
    uint8_t msg[8];
    uint8_t rx_buf[8];
    uint8_t rx_cnt = 0;
    uint32_t timeout_ms = 500; // [§ª§³§±§²§¡§£§­§¦§¯§ª§¦]: §µ§Ó§Ö§Ý§Ú§é§Ú§Ý§Ú §Õ§à 500 §Þ§ã §Õ§Ý§ñ §Ü§à§Þ§á§Ö§ß§ã§Ñ§è§Ú§Ú §Ù§Ñ§Õ§Ö§â§Ø§Ö§Ü USB §Ó Windows
    
    msg[0] = slave_id;
    msg[1] = 0x06;
    msg[2] = (uint8_t)(reg_addr >> 8);
    msg[3] = (uint8_t)(reg_addr & 0xFF);
    msg[4] = (uint8_t)(data_val >> 8);
    msg[5] = (uint8_t)(data_val & 0xFF);
    
    uint16_t crc = Calculate_CRC16(msg, 6);
    msg[6] = (uint8_t)(crc & 0xFF);
    msg[7] = (uint8_t)(crc >> 8);

    __disable_irq();

// 1. §£§Ü§Ý§ð§é§Ñ§Ö§Þ RS-485 §ß§Ñ §±§¦§²§¦§¥§¡§¹§µ (DE+RE = 1)
GPIO_WriteBit(GPIOD, GPIO_Pin_4, Bit_SET); 
Delay_Us(20); // §±§Ñ§å§Ù§Ñ §ß§Ñ §æ§Ú§Ù§Ú§é§Ö§ã§Ü§à§Ö §á§Ö§â§Ö§Ü§Ý§ð§é§Ö§ß§Ú§Ö §Ü§Ý§ð§é§Ö§Û §Þ§Ú§Ü§â§à§ã§ç§Ö§Þ§í

// 2. §°§ä§á§â§Ñ§Ó§Ý§ñ§Ö§Þ §á§Ñ§Ü§Ö§ä §ß§Ñ§â§å§Ø§å
for (uint8_t i = 0; i < 8; i++) {
    USART_SendData(USART1, msg[i]);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

// §¨§Õ§Ö§Þ, §á§à§Ü§Ñ §±§°§³§­§¦§¥§¯§ª§« §Ò§Ñ§Û§ä §á§à§Ý§ß§à§ã§ä§î§ð §Ó§í§Û§Õ§Ö§ä §Ú§Ù §ã§Õ§Ó§Ú§Ô§à§Ó§à§Ô§à §â§Ö§Ô§Ú§ã§ä§â§Ñ §Ó §Ü§Ñ§Ò§Ö§Ý§î
while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
USART_ClearFlag(USART1, USART_FLAG_TC); // §°§é§Ú§ë§Ñ§Ö§Þ §æ§Ý§Ñ§Ô §à§Ü§à§ß§é§Ñ§ß§Ú§ñ §á§Ö§â§Ö§Õ§Ñ§é§Ú

// §¥§Ñ§Ö§Þ §Ñ§á§á§Ñ§â§Ñ§ä§ß§à§Þ§å §ã§ä§à§á-§Ò§Ú§ä§å §Õ§à§Ý§Ö§ä§Ö§ä§î §Õ§à §±§¬ (~200 §Þ§Ü§ã §ß§Ñ 9600 §Ò§à§Õ)
Delay_Us(200); 

    // 3. §£§à§Ù§Ó§â§Ñ§ë§Ñ§Ö§Þ RS-485 §ß§Ñ §±§²§ª§¦§® (DE+RE = 0)
    GPIO_WriteBit(GPIOD, GPIO_Pin_4, Bit_RESET);
    Delay_Us(500); 

    // --- §¬§²§ª§´§ª§¹§¦§³§¬§ª §£§¡§¨§¯§°: §³§Ò§â§Ñ§ã§í§Ó§Ñ§Ö§Þ §æ§Ý§Ñ§Ô§Ú §Ñ§á§á§Ñ§â§Ñ§ä§ß§í§ç §à§ê§Ú§Ò§à§Ü ---
    // §¹§ä§Ö§ß§Ú§Ö DR §ã§Ò§â§Ñ§ã§í§Ó§Ñ§Ö§ä RXNE §Ú ORE. ClearFlag §ã§Ò§â§Ñ§ã§í§Ó§Ñ§Ö§ä FE, NE, PE.
    if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != RESET) {
        volatile uint16_t dummy = USART_ReceiveData(USART1); 
        (void)dummy;
    }
    USART_ClearFlag(USART1, USART_FLAG_ORE | USART_FLAG_NE | USART_FLAG_FE | USART_FLAG_PE);

    // 4. §±§Ö§â§Ö§ç§à§Õ§Ú§Þ §Ü §Ò§Ö§Ù§à§á§Ñ§ã§ß§à§Þ§å §á§â§Ú§×§Þ§å §à§ä§Ó§Ö§ä§Ñ §à§ä Python-§ã§Ú§Þ§å§Ý§ñ§ä§à§â§Ñ
    uint32_t total_us = 500000;     // §à§Ò§ë§Ú§Û §ä§Ñ§Û§Þ§Ñ§å§ä 500 §Þ§ã
uint32_t inter_us = 0;            // §Þ§Ö§Ø§Ò§Ñ§Û§ä§à§Ó§í§Û §ä§Ñ§Û§Þ§Ñ§å§ä (§Ù§Ñ§á§à§Ý§ß§ñ§Ö§ä§ã§ñ §á§à§ã§Ý§Ö §á§Ö§â§Ó§à§Ô§à §Ò§Ñ§Û§ä§Ñ)
uint8_t got_any = 0;
rx_cnt = 0;

while (rx_cnt < 8 && total_us > 0) {
    if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != RESET) {
        rx_buf[rx_cnt++] = USART_ReceiveData(USART1);
        got_any = 1;
        inter_us = 5000;         // 5 §Þ§ã §ä§Ú§ê§Ú§ß§í = §Ü§à§ß§Ö§è §æ§â§Ö§Û§Þ§Ñ (T3.5 §ß§Ñ 9600 ¡Ö 3.65 §Þ§ã)
    } else {
        // §±§à§Þ§Ö§ç§Ñ/§à§Ò§â§í§Ó ¡ª §ã§Ò§â§Ñ§ã§í§Ó§Ñ§Ö§Þ §Ú §ß§Ñ§é§Ú§ß§Ñ§Ö§Þ §á§â§Ú§×§Þ §Ù§Ñ§ß§à§Ó§à
        if (USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET ||
            USART_GetFlagStatus(USART1, USART_FLAG_FE) != RESET ||
            USART_GetFlagStatus(USART1, USART_FLAG_NE) != RESET)
        {
            volatile uint16_t dummy = USART_ReceiveData(USART1);
            (void)dummy;
            USART_ClearFlag(USART1, USART_FLAG_ORE | USART_FLAG_NE |
                                    USART_FLAG_FE | USART_FLAG_PE);
            rx_cnt = 0;
            got_any = 0;
            inter_us = 0;
        }

        Delay_Us(100);             // 100 §Þ§Ü§ã ¡ª §Ó 10 §â§Ñ§Ù §Ò§í§ã§ä§â§Ö§Ö §Ó§â§Ö§Þ§Ö§ß§Ú §Ò§Ñ§Û§ä§Ñ, ORE §ß§Ö §Ó§à§Ù§ß§Ú§Ü§ß§Ö§ä
        total_us -= 100;

        // §¦§ã§Ý§Ú §å§Ø§Ö §é§ä§à-§ä§à §á§â§Ú§ß§ñ§Ý§Ú §Ú §ß§Ñ§ã§ä§å§á§Ú§Ý§Ñ §ä§Ú§ê§Ú§ß§Ñ ¡ª §æ§â§Ö§Û§Þ §Ù§Ñ§Ü§à§ß§é§Ú§Ý§ã§ñ
        if (got_any) {
            if (inter_us <= 100) break;
            inter_us -= 100;
        }
    }

    // §±§à§Õ§Ü§Ñ§â§Þ§Ý§Ú§Ó§Ñ§Ö§Þ §Ó§Ñ§ä§é§Õ§à§Ô §á§â§Ú§Þ§Ö§â§ß§à §â§Ñ§Ù §Ó 50 §Þ§ã
    if ((total_us % 50000) == 0) IWDG_Feed();
}
    
if (rx_cnt == 5 && rx_buf[0] == slave_id && (rx_buf[1] & 0x80)) {
    printf("[UART-LOG]: Modbus EXCEPTION code=0x%02X\r\n", rx_buf[2]);
    return 0;
}

    // 5. §±§â§à§Ó§Ö§â§ñ§Ö§Þ §Ó§Ñ§Ý§Ú§Õ§ß§à§ã§ä§î §à§ä§Ó§Ö§ä§Ñ §â§à§å§ä§Ö§â§Ñ
    // 5. §¥§¦§´§¡§­§¾§¯§¡§Á §±§²§°§£§¦§²§¬§¡ §ª §±§¡§²§¡§­§­§¦§­§¾§¯§½§« §£§½§£§°§¥ §£ UART
    if (rx_cnt > 0) 
    {
        // §£§í§Ó§à§Õ§Ú§Þ §Ó §Þ§à§ß§Ú§ä§à§â WCH-Link §ã§Ü§à§Ý§î§Ü§à §Ò§Ñ§Û§ä §â§Ö§Ñ§Ý§î§ß§à §Õ§à§Ý§Ö§ä§Ö§Ý§à §Õ§à §®§¬
        printf("\r\n[UART-LOG]: §±§â§Ú§ß§ñ§ä§à §Ò§Ñ§Û§ä: %d §Ú§Ù 8\r\n", rx_cnt);
        
        if (rx_cnt == 8) 
        {
            printf("[UART-LOG]: §¢§å§æ§Ö§â: %02X %02X %02X %02X %02X %02X %02X %02X\r\n", 
                   rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3], 
                   rx_buf[4], rx_buf[5], rx_buf[6], rx_buf[7]);

            uint16_t rx_crc = Calculate_CRC16(rx_buf, 6);
            uint8_t exp_low = (uint8_t)(rx_crc & 0xFF);
            uint8_t exp_high = (uint8_t)(rx_crc >> 8);

            printf("[UART-LOG]: §³§Ó§Ö§â§ñ§Ö§Þ CRC. §£ §á§Ñ§Ü§Ö§ä§Ö: %02X %02X | §²§Ñ§ã§é§Ö§ä: %02X %02X\r\n", 
                   rx_buf[6], rx_buf[7], exp_low, exp_high);

            if (rx_buf[0] == slave_id && rx_buf[1] == 0x06 && 
                rx_buf[6] == exp_high && rx_buf[7] == exp_low) 
            {
                printf("[UART-LOG]: === §¬§£§ª§´§ª§²§°§£§¡§¯§ª§¦ §µ§³§±§¦§º§¯§°! ===\r\n\r\n");
                return 1; 
            }
            else 
            {
                printf("[UART-LOG]: !!! §°§º§ª§¢§¬§¡: §¥§Ñ§ß§ß§í§Ö §Ú§Ý§Ú CRC §ß§Ö §ã§à§Ó§á§Ñ§Ý§Ú !!!\r\n\r\n");
            }
        }
    }
    else 
    {
        // §¦§ã§Ý§Ú §Ó§à§à§Ò§ë§Ö §ß§Ú§é§Ö§Ô§à §ß§Ö §á§â§Ú§Ý§Ö§ä§Ö§Ý§à §Ù§Ñ §Ó§â§Ö§Þ§ñ §ä§Ñ§Û§Þ§Ñ§å§ä§Ñ
        printf("\r\n[UART-LOG]: §°§º§ª§¢§¬§¡: §´§Ñ§Û§Þ§Ñ§å§ä §à§Ø§Ú§Õ§Ñ§ß§Ú§ñ §à§ä§Ó§Ö§ä§Ñ. §¢§å§æ§Ö§â §á§å§ã§ä.\r\n");
    }

    return 0; 
}








// --- §³§¡§®§¡ §¶§µ§¯§¬§¸§ª§Á §£§½§£§°§¥§¡ §´§¦§­§¦§®§¦§´§²§ª§ª ---
void Print_Readable_Telemetry(void)
{
//     BQ_ReceiveData(ADCVSYSVBAT);     
//     BQ_ReceiveData(ADCVBUS_PSYS);    
//     BQ_ReceiveData(ADCIBAT);         
//     BQ_ReceiveData(ADCIINCMPIN);     
//     BQ_ReceiveData(CHARGE_STATUS_REGISTER); 

//     uint16_t raw_vbat = REG_getVal(ADCVSYSVBAT);
//     uint16_t raw_vbus = REG_getVal(ADCVBUS_PSYS);
//     uint16_t raw_ibat = REG_getVal(ADCIBAT);
//     uint16_t raw_iin  = REG_getVal(ADCIINCMPIN);
//     uint16_t raw_stat = REG_getVal(CHARGE_STATUS_REGISTER);

//     // §¢§Ñ§Ù§à§Ó§í§Û §â§Ñ§ã§é§Ö§ä §ß§Ñ§á§â§ñ§Ø§Ö§ß§Ú§Û (1 LSB = 0.25 §Þ§£)
//     uint32_t base_vbus_mv = (uint32_t)(raw_vbus / 4);
//     uint32_t base_vbat_mv = (uint32_t)(raw_vbat / 4);

//     // §¬§Ñ§Ý§Ú§Ò§â§à§Ó§Ü§Ñ §ß§Ñ§á§â§ñ§Ø§Ö§ß§Ú§Û (§ã §å§é§Ö§ä§à§Þ §á§Ñ§Õ§Ö§ß§Ú§ñ §ß§Ñ §á§Ý§Ñ§ä§Ö §á§à§Õ §ß§Ñ§Ô§â§å§Ù§Ü§à§Û)
//     uint32_t vbus_mv = (uint32_t)(base_vbus_mv * 1.2673);
//     uint32_t vbat_mv = (uint32_t)(base_vbat_mv * 1.2653);
    
//     // §£§ç§à§Õ§ß§à§Û §ä§à§Ü §à§ä §¢§± (1 LSB = 0.25 §Þ§¡, §Ü§Ñ§Ý§Ú§Ò§â§à§Ó§Ü§Ñ §ê§å§ß§ä§Ñ)
//     uint32_t base_iin_ma = (uint32_t)(raw_iin / 4);
//     uint32_t iin_ma      = (uint32_t)(base_iin_ma * 1.289);

//     // §±§Ö§â§Ö§Þ§Ö§ß§ß§Ñ§ñ §Õ§Ý§ñ §Ú§ä§à§Ô§à§Ó§à§Ô§à §Ù§ß§Ñ§é§Ö§ß§Ú§ñ §ä§à§Ü§Ñ §¡§¬§¢
//     uint32_t ibat_ma = 0;

//     //printf("====================================\r\n");
//    // printf(" VBUS: %d.%03d V\r\n", vbus_mv / 1000, vbus_mv % 1000);
//    // printf(" VBAT: %d.%03d V\r\n", vbat_mv / 1000, vbat_mv % 1000);
    
//     // §°§±§²§¦§¥§¦§­§¦§¯§ª§¦ §²§¦§¨§ª§®§¡ §ª §±§²§¡§£§ª§­§¾§¯§½§« §²§¡§³§¹§¦§´ §´§°§¬§¡ §¡§¬§¢
//     if (vbus_mv > 4000) {
//         // §²§¦§¨§ª§® §³§¦§´§ª: §£ §â§Ö§Ô§Ú§ã§ä§â§Ö §ä§à§Ü §©§¡§²§Á§¥§¡ (ICHG: 1 LSB §á§à§Ý§ß§à§Ô§à §â§Ö§Ô§Ú§ã§ä§â§Ñ = 0.25 §Þ§¡)
//         ibat_ma = (uint32_t)((raw_ibat / 4) * 1.289); 
        
//         if (ibat_ma > 30) {
//             //printf(" §´§à§Ü §¡§¬§¢ (§ª§Õ§Ö§ä §©§¡§²§Á§¥§¬§¡): +%d §Þ§¡\r\n", ibat_ma);
//         } else {
//             //printf(" §´§à§Ü §¡§¬§¢: §³§ä§Ñ§è§Ú§à§ß§Ñ§â§ß§í§Û §â§Ö§Ø§Ú§Þ (0 §Þ§¡)\r\n");
//         }
//     } else {
//         // §²§¦§¨§ª§® §¢§¡§´§¡§²§¦§ª: §£ §â§Ö§Ô§Ú§ã§ä§â§Ö §ä§à§Ü §²§¡§©§²§Á§¥§¡ (IDCHG: 1 LSB §á§à§Ý§ß§à§Ô§à §â§Ö§Ô§Ú§ã§ä§â§Ñ = 1.0 §Þ§¡)
//         //printf(" DEBUG RAW IDCHG REG: 0x%04X\r\n", raw_ibat);

//         // §²§Ñ§ã§é§Ö§ä: §á§à§Ý§ß§à§Ö 16-§Ò§Ú§ä§ß§à§Ö §é§Ú§ã§Ý§à §ß§Ñ§á§â§ñ§Þ§å§ð §à§ä§â§Ñ§Ø§Ñ§Ö§ä §ä§à§Ü §Ó §Þ§¡, §å§Þ§ß§à§Ø§Ñ§Ö§Þ §ß§Ñ §Ü§à§ï§æ§æ§Ú§è§Ú§Ö§ß§ä §ê§å§ß§ä§Ñ
//         ibat_ma = (uint32_t)(raw_ibat * 1.289);
        
//         //printf(" §´§à§Ü §¡§¬§¢ (§²§Ñ§Ù§â§ñ§Õ §ß§Ñ§Ô§â§å§Ù§Ü§Ú): %d §Þ§¡\r\n", ibat_ma);
//     }

//     //printf(" §°§Ò§ë§Ú§Û §Ó§ç§à§Õ§ñ§ë§Ú§Û §ä§à§Ü §ã§Ú§ã§ä§Ö§Þ§í: %d §Þ§¡\r\n", iin_ma);
//     //printf("====================================\r\n");
}
