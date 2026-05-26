#include "BQ.h"


Reg_TypeDef Registers[REG_NAME_SIZE];


uint8_t ADDR_array[REG_NAME_SIZE] =
{
CHARGE_OPTION_0_ADDR,
CHARGE_CURRENT_REGISTER_ADDR,
CHARGE_VOLTAGE_REGISTER_ADDR,
OTG_VOLTAGE_REGISTER_ADDR,
OTG_CURRENT_REGISTER_ADDR,
INPUT_VOLTAGE_REGISTER_ADDR,
MINIMUM_SYSTEM_VOLTAGE_ADDR,
INPUT_CURRENT_REGISTER_ADDR,
CHARGE_OPTION_1_ADDR,
CHARGE_OPTION_2_ADDR,
CHARGE_OPTION_3_ADDR,
PROCHOT_OPTION_0_ADDR,
PROCHOT_OPTION_1_ADDR,
ADC_OPTION_ADDR,

CHARGE_STATUS_REGISTER_ADDR,
PROCHOT_STATUS_REGISTER_ADDR,
INPUT_CURRENT_LIMIT_IN_USE_ADDR,
ADCVBUS_PSYS_ADDR,
ADCIBAT_ADDR,
ADCIINCMPIN_ADDR,
ADCVSYSVBAT_ADDR
};


uint16_t INIT_array[REG_NAME_SIZE] =
{
INIT_CHARGE_OPTION_0,
INIT_CHARGE_CURRENT_REGISTER,
INIT_CHARGE_VOLTAGE_REGISTER,
INIT_OTG_VOLTAGE_REGISTER,
INIT_OTG_CURRENT_REGISTER,
INIT_INPUT_VOLTAGE_REGISTER,
INIT_MINIMUM_SYSTEM_VOLTAGE,
INIT_INPUT_CURRENT_REGISTER,
INIT_CHARGE_OPTION_1,
INIT_CHARGE_OPTION_2,
INIT_CHARGE_OPTION_3,
INIT_PROCHOT_OPTION_0,
INIT_PROCHOT_OPTION_1,
INIT_ADC_OPTION,

INIT_CHARGE_STATUS_REGISTER,
INIT_PROCHOT_STATUS_REGISTER,
INIT_INPUT_CURRENT_LIMIT_IN_USE,
INIT_ADCVBUS_PSYS,
INIT_ADCIBAT,
INIT_ADCIINCMPIN,
INIT_ADCVSYSVBAT
};


void BQ_Init(void)
{
    HAL_I2C_Init(BQ_I2C_BAUDRATE, BQ_ADDRESS);
    BQ_config();
}


void BQ_InitRegisters(void)
{
    for(uint8_t i = 0; i < REG_NAME_SIZE; ++i)
    {
        REG_Init(&Registers[i], ADDR_array[i], INIT_array[i]);
    }
}

void BQ_ADC_reInit(void)
{
    REG_setVal(ADC_OPTION, INIT_ADC_OPTION);
    BQ_TransmitData(ADC_OPTION);
    REG_setVal(CHARGE_OPTION_0, INIT_CHARGE_OPTION_0);
    BQ_TransmitData(CHARGE_OPTION_0);
}


void REG_Init(Reg_TypeDef *Registers, uint8_t MemAddress, uint16_t value)
{
    (*Registers).LOW = (uint8_t)value;
    (*Registers).HIGH = (uint8_t)(value >> 8);
    (*Registers).MemAddress = MemAddress;
}

void REG_setVal(Reg_name name, uint16_t value)
{
    Registers[name].LOW = (uint8_t)value;
    Registers[name].HIGH = (uint8_t)(value >> 8);
}


uint16_t REG_getVal(Reg_name name)
{
    uint16_t value = Registers[name].HIGH;
    value = (value << 8);
    value += Registers[name].LOW;
    return value;
}


void REG_Print(Reg_name name)
{
    uint16_t val = REG_getVal(name);
#ifdef DEBUG_ON
    printf("%02x: %04x\r\n", Registers[name].MemAddress, val);
#endif
}


void BQ_printAll(void)
{
    for(uint8_t i = 0; i < REG_NAME_SIZE; ++i)
    {
        BQ_ReceiveData(i);
        REG_Print(i);
    }
}


void BQ_TransmitData(Reg_name name)
{
    HAL_I2C_Mem_Write(I2C1, BQ_ADDRESS, Registers[name].MemAddress, &Registers[name].LOW, 1, I2C_TIMEOUT);
    HAL_I2C_Mem_Write(I2C1, BQ_ADDRESS, Registers[name].MemAddress + 1, &Registers[name].HIGH, 1, I2C_TIMEOUT);
}


void BQ_ReceiveData(Reg_name name)
{
    HAL_I2C_Mem_Read(I2C1, BQ_ADDRESS, Registers[name].MemAddress, &Registers[name].LOW, 1, I2C_TIMEOUT);
    HAL_I2C_Mem_Read(I2C1, BQ_ADDRESS, Registers[name].MemAddress + 1, &Registers[name].HIGH, 1, I2C_TIMEOUT);
}


void BQ_config(void)
{
#ifdef DEBUG_ON
    printf("BQ config...\r\n");
#endif
    BQ_InitRegisters();
    for(uint8_t i = 0; i <= ADC_OPTION; ++i)
    {
        REG_Print(i);
        BQ_TransmitData(i);
    }
#ifdef DEBUG_ON
    printf("config finish.\r\n");
#endif
}


BQ_HV_t BQ_STAT_monitor(void)
{
    uint8_t reg = 0x00;
    BQ_ReceiveData(CHARGE_STATUS_REGISTER);
    reg = Registers[CHARGE_STATUS_REGISTER].HIGH;
    reg &= (~REG_AC_STAT_MASK);
    if(reg != REG_AC_STAT)
    {
        return BQ_HV_ERR;
    }
    return BQ_HV_OK;
}


BQ_HV_t BQ_CURR_monitor(void)
{
    uint16_t reg = 0x0000;
    BQ_ReceiveData(CHARGE_CURRENT_REGISTER);
    reg = REG_getVal(CHARGE_CURRENT_REGISTER);
    if(reg != INIT_CHARGE_CURRENT_REGISTER)
    {
        return BQ_HV_ERR;
    }
    return BQ_HV_OK;
}


BQ_HV_t BQ_CHARGE_monitor(void)
{
    uint8_t reg = 0x00;
    BQ_ReceiveData(CHARGE_STATUS_REGISTER);
    reg = Registers[CHARGE_STATUS_REGISTER].HIGH;
    reg &= (~REG_FAST_CHARGE_MASK);
    if(reg != REG_FAST_CHARGE)
    {
        return BQ_HV_ERR;
    }
    return BQ_HV_OK;
}


