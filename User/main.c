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

// --- ������������������ ���������� �������������� ---
void Print_Readable_Telemetry(void);
uint16_t Calculate_CRC16(uint8_t *buffer, uint16_t buffer_length);
uint8_t Modbus_Master_Write_Register(uint8_t slave_id, uint16_t reg_addr, uint16_t data_val);



// --- �������������� ������ ���������������� �������������� CRC-16 MODBUS ---
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
    
    // ���֧�֧ާ֧ߧߧѧ� �էݧ� �٧ѧէ֧�اܧ� �ӧܧݧ��֧ߧڧ� �٧ѧ�ڧ��
    uint32_t startup_protection_delay = 0;

    // ����ѧߧ֧ߧڧ� ���֧է��է��֧ԧ� �������ߧڧ� ��ڧ�ѧߧڧ� (0 - ���֧�� ����, 1 - ���ӧѧ�ڧ� ��ڧ�ѧߧڧ�)
    

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    SystemCoreClockUpdate();
    Systick_Init();
    Timer_init();
    FlashTimer_init();
    
    // 1. ���ߧڧ�ڧѧݧڧ٧ڧ��֧� GPIO (�ӧߧ���� �է�ݧا֧� �ҧ���� BAT_ON �ߧ� PC0)
    GPIO_periph_init(); 
    
    // ����ڧߧ�էڧ�֧ݧ�ߧ� ���է�ӧ֧�اէѧ֧� �ӧܧݧ��֧ߧڧ� �ҧѧ�ѧ�֧� ��֧�֧� ���ѧ���� BQ
    GPIO_WriteBit(GPIOC, GPIO_Pin_0, Bit_SET); 

    USARTx_CFG();

//#ifdef DEBUG_ON
    //printf("System Start...\r\n");
    //printf("SystemClk:%d\r\n", SystemCoreClock);
   // printf("ChipID:%08x\r\n", DBGMCU_GetCHIPID());
//#endif

    // 2. ���ߧڧ�ڧѧݧڧ٧ڧ��֧� BQ (��֧�֧��, �ܧ�ԧէ� ��ڧ�ѧߧڧ� ���ѧҧڧݧڧ٧ڧ��ӧѧݧ���)
    BQ_Init();
    
    IWDG_Feed_Init(IWDG_Prescaler_128, 4000);

while(1)
{
    // ���ѧۧާ֧� ��ڧܧѧ֧� �ܧѧاէ�� ��֧ܧ�ߧէ� �ߧ֧٧ѧӧڧ�ڧާ� ��� DEBUG_ON
    if(Timer_tick())
    {
        // �����٧��ӧѧ֧� ��ҧ�� �էѧߧߧ��� �� BQ (�ҧ֧� �ӧ��ӧ�է� printf �ߧ� ��ܧ�ѧ�)
        BQ_ReceiveData(ADCVSYSVBAT);
        BQ_ReceiveData(ADCVBUS_PSYS);
        BQ_ReceiveData(ADCIBAT);
        BQ_ReceiveData(ADCIINCMPIN);
        BQ_ReceiveData(CHARGE_STATUS_REGISTER);

// --- ���������������� ������������������ �������������� �� �������������� �������������� ---
// --- ������������ ������������������������ ������������ �� ������������������ ������������ (9600 ������) ---
uint16_t raw_vbus = REG_getVal(ADCVBUS_PSYS);
uint32_t vbus_mv = (uint32_t)((raw_vbus / 4) * 1.2673);

if (vbus_mv < 4000) 
{
    // ����ݧ� �ާ� �֧�� �ߧ� �٧ѧ�ڧܧ�ڧ��ӧѧݧ� ����֧�ߧ��� ���ӧ֧� ��� �����֧�� ��� �ѧӧѧ�ڧ�
    if (last_power_status == 0) 
    {
        // ������ѧӧݧ�֧� �ܧ�� �ѧӧѧ�ڧ� (1). ����ݧ� �����֧� ���ӧ֧�ڧ� �� CRC ���ӧ�ѧէ֧�, ���ߧܧ�ڧ� �ӧ֧�ߧ֧� 1
        if (Modbus_Master_Write_Register(0xA1, 0, 1) == 1) 
        {
            last_power_status = 1; // ����������: ������֧� ���է�ӧ֧�էڧ�! ������ѧӧܧ� ���֧ܧ�ѧ�ѧ֧���.
        }
    }
} 
else // vbus_mv >= 4000 (���ڧ�ѧߧڧ� �ӧ����ѧߧ�ӧڧݧ���)
{
    // ����ݧ� �է� ����ԧ� �ҧ��ݧ� �٧ѧ�ڧܧ�ڧ��ӧѧߧ� �ѧӧѧ�ڧ�, �ܧ������ �����֧� ���է�ӧ֧�اէѧ�
    if (last_power_status == 1) 
    {
        // ������ѧӧݧ�֧� �ܧ�� ���ާ֧ߧ� �ѧӧѧ�ڧ� (0). ���է֧� ���է�ӧ֧�اէ֧ߧڧ�
        if (Modbus_Master_Write_Register(0xA1, 0, 0) == 1) 
        {
            last_power_status = 0; // ����������: ������֧� ���է�ӧ֧�էڧ� �ӧ�٧ӧ�ѧ� �� ��֧��!
        }
    }
}



        // 3. ������������ ������������ ��������������
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

    // ���ݧ�� ��ҧ�ѧҧ��ܧ� TIM1 (�ާڧԧѧߧڧ� ��ӧ֧��էڧ�է�) ����ѧӧݧ�֧� ���ݧߧ����� �ҧ֧� �ڧ٧ާ֧ߧ֧ߧڧ�
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

// --- �������������� ���������������� CRC-16 MODBUS ---
// ����ݧߧѧ� �٧ѧާ֧ߧ� �ӧѧ�֧� ���ѧ��� ���ߧܧ�ڧ� �ߧ� �ܧѧߧ�ߧڧ�֧�ܧڧ� ��ѧ��֧�
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
 

// --- �������������� ���������������� MODBUS ������������ ������������ (�������������� 06) ---
// --- ������������������ �������������������������� �������������� ���������������� MODBUS ---
// ����ߧܧ�ڧ� ��֧�֧�� �ӧ�٧ӧ�ѧ�ѧ֧� 1 (���ӧ֧� ���ݧ��֧�) �ڧݧ� 0 (��ѧۧާѧ��)
uint8_t Modbus_Master_Write_Register(uint8_t slave_id, uint16_t reg_addr, uint16_t data_val)
{
    uint8_t msg[8];
    uint8_t rx_buf[8];
    uint8_t rx_cnt = 0;
    uint32_t timeout_ms = 500; // [����������������������]: ���ӧ֧ݧڧ�ڧݧ� �է� 500 �ާ� �էݧ� �ܧ�ާ�֧ߧ�ѧ�ڧ� �٧ѧէ֧�ا֧� USB �� Windows
    
    msg[0] = slave_id;
    msg[1] = 0x06;
    msg[2] = (uint8_t)(reg_addr >> 8);
    msg[3] = (uint8_t)(reg_addr & 0xFF);
    msg[4] = (uint8_t)(data_val >> 8);
    msg[5] = (uint8_t)(data_val & 0xFF);
    
    uint16_t crc = Calculate_CRC16(msg, 6);
    msg[6] = (uint8_t)(crc & 0xFF);
    msg[7] = (uint8_t)(crc >> 8);

    // NB: __disable_irq() убран — он был без парного __enable_irq() и глушил
    // все ISR (SysTick/Timer/IWDG) после первого вызова

// 1. Включаем RS-485 на передачу (DE+RE = 1)
GPIO_WriteBit(GPIOD, GPIO_Pin_4, Bit_SET);
Delay_Us(20); // ���ѧ�٧� �ߧ� ��ڧ٧ڧ�֧�ܧ�� ��֧�֧ܧݧ��֧ߧڧ� �ܧݧ��֧� �ާڧܧ����֧ާ�

// 2. ������ѧӧݧ�֧� ��ѧܧ֧� �ߧѧ��ا�
for (uint8_t i = 0; i < 8; i++) {
    USART_SendData(USART1, msg[i]);
    while (USART_GetFlagStatus(USART1, USART_FLAG_TXE) == RESET);
}

// ���է֧�, ���ܧ� ������������������ �ҧѧۧ� ���ݧߧ����� �ӧ��ۧէ֧� �ڧ� ��էӧڧԧ�ӧ�ԧ� ��֧ԧڧ���� �� �ܧѧҧ֧ݧ�
while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
USART_ClearFlag(USART1, USART_FLAG_TC); // ����ڧ�ѧ֧� ��ݧѧ� ��ܧ�ߧ�ѧߧڧ� ��֧�֧էѧ��

// ���ѧ֧� �ѧ��ѧ�ѧ�ߧ�ާ� �����-�ҧڧ�� �է�ݧ֧�֧�� �է� ���� (~200 �ާܧ� �ߧ� 9600 �ҧ��)
Delay_Us(200); 

    // 3. ����٧ӧ�ѧ�ѧ֧� RS-485 �ߧ� ���������� (DE+RE = 0)
    GPIO_WriteBit(GPIOD, GPIO_Pin_4, Bit_RESET);
    Delay_Us(500); 

    // КРИТИЧНО: правильная очистка ORE/NE/FE/PE на CH32 — sequence read STATR -> read DATAR.
    // USART_ClearFlag для этих флагов не работает (HAL пропускает только TC/RXNE/LBD/CTS).
    // Если ORE остался от собственной TX-эхо/шума при переключении DE — RXNE дальше не выставится.
    (void)USART1->STATR;
    (void)USART1->DATAR;

    // 4. ���֧�֧��էڧ� �� �ҧ֧٧��ѧ�ߧ�ާ� ���ڧקާ� ���ӧ֧�� ��� Python-��ڧާ�ݧ�����
    uint32_t total_us = 500000;     // ��ҧ�ڧ� ��ѧۧާѧ�� 500 �ާ�
uint32_t inter_us = 0;            // �ާ֧اҧѧۧ��ӧ��� ��ѧۧާѧ�� (�٧ѧ��ݧߧ�֧��� ����ݧ� ��֧�ӧ�ԧ� �ҧѧۧ��)
uint8_t got_any = 0;
rx_cnt = 0;

while (rx_cnt < 8 && total_us > 0) {
    if (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) != RESET) {
        rx_buf[rx_cnt++] = USART_ReceiveData(USART1);
        got_any = 1;
        inter_us = 5000;         // 5 �ާ� ��ڧ�ڧߧ� = �ܧ�ߧ֧� ���֧ۧާ� (T3.5 �ߧ� 9600 �� 3.65 �ާ�)
    } else {
        // ����ާ֧��/��ҧ���� �� ��ҧ�ѧ���ӧѧ֧� �� �ߧѧ�ڧߧѧ֧� ���ڧק� �٧ѧߧ�ӧ�
        if (USART_GetFlagStatus(USART1, USART_FLAG_ORE) != RESET ||
            USART_GetFlagStatus(USART1, USART_FLAG_FE) != RESET ||
            USART_GetFlagStatus(USART1, USART_FLAG_NE) != RESET)
        {
            // sequence read STATR -> read DATAR сбрасывает ORE/NE/FE/PE
            (void)USART1->STATR;
            (void)USART1->DATAR;
            rx_cnt = 0;
            got_any = 0;
            inter_us = 0;
        }

        Delay_Us(100);             // 100 �ާܧ� �� �� 10 ��ѧ� �ҧ�����֧� �ӧ�֧ާ֧ߧ� �ҧѧۧ��, ORE �ߧ� �ӧ�٧ߧڧܧߧ֧�
        total_us -= 100;

        // ����ݧ� ��ا� ����-��� ���ڧߧ�ݧ� �� �ߧѧ����ڧݧ� ��ڧ�ڧߧ� �� ���֧ۧ� �٧ѧܧ�ߧ�ڧݧ��
        if (got_any) {
            if (inter_us <= 100) break;
            inter_us -= 100;
        }
    }

    // ����էܧѧ�ާݧڧӧѧ֧� �ӧѧ��է�� ���ڧާ֧�ߧ� ��ѧ� �� 50 �ާ�
    if ((total_us % 50000) == 0) IWDG_Feed();
}
    
if (rx_cnt == 5 && rx_buf[0] == slave_id && (rx_buf[1] & 0x80)) {
    printf("[UART-LOG]: Modbus EXCEPTION code=0x%02X\r\n", rx_buf[2]);
    return 0;
}

    // 5. �����ӧ֧��֧� �ӧѧݧڧէߧ���� ���ӧ֧�� �����֧��
    // 5. ������������������ ���������������� �� ������������������������ ���������� �� UART
    if (rx_cnt > 0) 
    {
        // �����ӧ�էڧ� �� �ާ�ߧڧ��� WCH-Link ��ܧ�ݧ�ܧ� �ҧѧۧ� ��֧ѧݧ�ߧ� �է�ݧ֧�֧ݧ� �է� ����
        printf("\r\n[UART-LOG]: ����ڧߧ��� �ҧѧۧ�: %d �ڧ� 8\r\n", rx_cnt);
        
        if (rx_cnt == 8) 
        {
            printf("[UART-LOG]: �����֧�: %02X %02X %02X %02X %02X %02X %02X %02X\r\n", 
                   rx_buf[0], rx_buf[1], rx_buf[2], rx_buf[3], 
                   rx_buf[4], rx_buf[5], rx_buf[6], rx_buf[7]);

            uint16_t rx_crc = Calculate_CRC16(rx_buf, 6);
            uint8_t exp_low = (uint8_t)(rx_crc & 0xFF);
            uint8_t exp_high = (uint8_t)(rx_crc >> 8);

            printf("[UART-LOG]: ���ӧ֧��֧� CRC. �� ��ѧܧ֧��: %02X %02X | ���ѧ��֧�: %02X %02X\r\n", 
                   rx_buf[6], rx_buf[7], exp_low, exp_high);

            if (rx_buf[0] == slave_id && rx_buf[1] == 0x06 && 
                rx_buf[6] == exp_high && rx_buf[7] == exp_low) 
            {
                printf("[UART-LOG]: === ������������������������ ��������������! ===\r\n\r\n");
                return 1; 
            }
            else 
            {
                printf("[UART-LOG]: !!! ������������: ���ѧߧߧ��� �ڧݧ� CRC �ߧ� ���ӧ�ѧݧ� !!!\r\n\r\n");
            }
        }
    }
    else 
    {
        // ����ݧ� �ӧ��ҧ�� �ߧڧ�֧ԧ� �ߧ� ���ڧݧ֧�֧ݧ� �٧� �ӧ�֧ާ� ��ѧۧާѧ���
        printf("\r\n[UART-LOG]: ������������: ���ѧۧާѧ�� ��اڧէѧߧڧ� ���ӧ֧��. �����֧� �����.\r\n");
    }

    return 0; 
}








// --- �������� �������������� ������������ �������������������� ---
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

//     // ���ѧ٧�ӧ��� ��ѧ��֧� �ߧѧ���ا֧ߧڧ� (1 LSB = 0.25 �ާ�)
//     uint32_t base_vbus_mv = (uint32_t)(raw_vbus / 4);
//     uint32_t base_vbat_mv = (uint32_t)(raw_vbat / 4);

//     // ���ѧݧڧҧ��ӧܧ� �ߧѧ���ا֧ߧڧ� (�� ���֧��� ��ѧէ֧ߧڧ� �ߧ� ��ݧѧ�� ���� �ߧѧԧ��٧ܧ��)
//     uint32_t vbus_mv = (uint32_t)(base_vbus_mv * 1.2673);
//     uint32_t vbat_mv = (uint32_t)(base_vbat_mv * 1.2653);
    
//     // �����էߧ�� ���� ��� ���� (1 LSB = 0.25 �ާ�, �ܧѧݧڧҧ��ӧܧ� ���ߧ��)
//     uint32_t base_iin_ma = (uint32_t)(raw_iin / 4);
//     uint32_t iin_ma      = (uint32_t)(base_iin_ma * 1.289);

//     // ���֧�֧ާ֧ߧߧѧ� �էݧ� �ڧ��ԧ�ӧ�ԧ� �٧ߧѧ�֧ߧڧ� ���ܧ� ������
//     uint32_t ibat_ma = 0;

//     //printf("====================================\r\n");
//    // printf(" VBUS: %d.%03d V\r\n", vbus_mv / 1000, vbus_mv % 1000);
//    // printf(" VBAT: %d.%03d V\r\n", vbat_mv / 1000, vbat_mv % 1000);
    
//     // ���������������������� ������������ �� �������������������� ������������ �������� ������
//     if (vbus_mv > 4000) {
//         // ���������� ��������: �� ��֧ԧڧ���� ���� ������������ (ICHG: 1 LSB ���ݧߧ�ԧ� ��֧ԧڧ���� = 0.25 �ާ�)
//         ibat_ma = (uint32_t)((raw_ibat / 4) * 1.289); 
        
//         if (ibat_ma > 30) {
//             //printf(" ����� ������ (���է֧� ��������������): +%d �ާ�\r\n", ibat_ma);
//         } else {
//             //printf(" ����� ������: ����ѧ�ڧ�ߧѧ�ߧ��� ��֧اڧ� (0 �ާ�)\r\n");
//         }
//     } else {
//         // ���������� ��������������: �� ��֧ԧڧ���� ���� �������������� (IDCHG: 1 LSB ���ݧߧ�ԧ� ��֧ԧڧ���� = 1.0 �ާ�)
//         //printf(" DEBUG RAW IDCHG REG: 0x%04X\r\n", raw_ibat);

//         // ���ѧ��֧�: ���ݧߧ�� 16-�ҧڧ�ߧ�� ��ڧ�ݧ� �ߧѧ���ާ�� ����ѧاѧ֧� ���� �� �ާ�, ��ާߧ�اѧ֧� �ߧ� �ܧ����ڧ�ڧ֧ߧ� ���ߧ��
//         ibat_ma = (uint32_t)(raw_ibat * 1.289);
        
//         //printf(" ����� ������ (���ѧ٧��� �ߧѧԧ��٧ܧ�): %d �ާ�\r\n", ibat_ma);
//     }

//     //printf(" ���ҧ�ڧ� �ӧ��է��ڧ� ���� ��ڧ��֧ާ�: %d �ާ�\r\n", iin_ma);
//     //printf("====================================\r\n");
}
