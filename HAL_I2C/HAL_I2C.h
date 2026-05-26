#ifndef HAL_I2C_H_
#define HAL_I2C_H_

#define I2C_MAX_DELAY 0xFFFFFFFF

#include "SysTick.h"

typedef enum {
    I2C_OK = 0,
    I2C_ADDR_NACK,
    I2C_TIMEOUT,
    I2C_BUSY
} I2C_StatusTypeDef;

void HAL_I2C_Init(uint32_t bound, uint16_t address);

I2C_StatusTypeDef HAL_I2C_WaitOnFlagUntilTimeout(I2C_TypeDef *I2Cx, uint32_t I2C_FLAG, FlagStatus Flag, uint32_t Tickstart, uint32_t Timeout);
I2C_StatusTypeDef HAL_I2C_WaitEventFlagUntilTimeout(I2C_TypeDef *I2Cx, uint32_t I2C_EVENT, uint32_t Tickstart, uint32_t Timeout);

I2C_StatusTypeDef HAL_I2C_Transmit (I2C_TypeDef *I2Cx, uint8_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
I2C_StatusTypeDef HAL_I2C_Receive (I2C_TypeDef *I2Cx, uint8_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
I2C_StatusTypeDef HAL_I2C_Mem_Write (I2C_TypeDef *I2Cx, uint8_t DevAddress, uint8_t MemAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
I2C_StatusTypeDef HAL_IIC_Mem_Read (I2C_TypeDef *I2Cx, uint8_t DevAddress, uint8_t MemAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
I2C_StatusTypeDef HAL_I2C_Mem_Read (I2C_TypeDef *I2Cx, uint8_t DevAddress, uint8_t MemAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout);
#endif /* HAL_I2C_H_ */
