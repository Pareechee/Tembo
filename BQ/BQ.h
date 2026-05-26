#ifndef BQ_H
#define BQ_H


#include <ch32v00x_rcc.h>
#include "config.h"
#include "HAL_I2C.h"

#define BQ_ADDRESS 0xD6
#define BQ_I2C_BAUDRATE 80000
#define BQ_I2C_TIMEOUT 100

#define REG_AC_STAT_MASK 0x7F
#define REG_AC_STAT 0x80

#define REG_FAST_CHARGE_MASK 0xFB
#define REG_FAST_CHARGE 0x04

typedef struct
{
    uint8_t LOW;
    uint8_t HIGH;
    uint16_t MemAddress;
} Reg_TypeDef;

typedef enum
{
  BQ_OK     = 0x00U,
  BQ_ERR    = 0x01U
} BQ_StatusTypeDef;

typedef enum
{
  BQ_HV_OK  = 0x00U,
  BQ_HV_ERR = 0x01U
} BQ_HV_t;


typedef enum
{
    CHARGE_OPTION_0 = 0,
    CHARGE_CURRENT_REGISTER,
    CHARGE_VOLTAGE_REGISTER,
    OTG_VOLTAGE_REGISTER,
    OTG_CURRENT_REGISTER,
    INPUT_VOLTAGE_REGISTER,
    MINIMUM_SYSTEM_VOLTAGE,
    INPUT_CURRENT_REGISTER,
    CHARGE_OPTION_1,
    CHARGE_OPTION_2,
    CHARGE_OPTION_3,
    PROCHOT_OPTION_0,
    PROCHOT_OPTION_1,
    ADC_OPTION,

    CHARGE_STATUS_REGISTER,
    PROCHOT_STATUS_REGISTER,
    INPUT_CURRENT_LIMIT_IN_USE,
    ADCVBUS_PSYS,
    ADCIBAT,
    ADCIINCMPIN,
    ADCVSYSVBAT,

    REG_NAME_SIZE
} Reg_name;

void BQ_Init(void);
void BQ_InitRegisters(void);
void BQ_ADC_reInit(void);

void REG_Init(Reg_TypeDef *Registers, uint8_t MemAddress, uint16_t value);

void REG_setVal(Reg_name name, uint16_t value);
uint16_t REG_getVal(Reg_name name);
void REG_Print(Reg_name name);
void BQ_printAll(void);

void BQ_TransmitData(Reg_name name);
void BQ_ReceiveData(Reg_name name);
void BQ_config(void);

BQ_HV_t BQ_STAT_monitor(void);
BQ_HV_t BQ_CURR_monitor(void);
BQ_HV_t BQ_CHARGE_monitor(void);



#endif  /* BQ_H */
