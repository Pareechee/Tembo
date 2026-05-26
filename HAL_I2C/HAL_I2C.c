#include "HAL_I2C.h"


/********************************************************************
 * @fn      I2C_Init
 *
 * @brief   Initializes the I2C peripheral.
 *
 * @return  none
 */


void HAL_I2C_Init(uint32_t bound, uint16_t address) {
    GPIO_InitTypeDef GPIO_InitStructure = { 0 };
    I2C_InitTypeDef I2C_InitTSturcture = { 0 };

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    I2C_InitTSturcture.I2C_ClockSpeed = bound;
    I2C_InitTSturcture.I2C_Mode = I2C_Mode_I2C;
    I2C_InitTSturcture.I2C_DutyCycle = I2C_DutyCycle_16_9;
    I2C_InitTSturcture.I2C_OwnAddress1 = address;
    I2C_InitTSturcture.I2C_Ack = I2C_Ack_Enable;
    I2C_InitTSturcture.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_Init(I2C1, &I2C_InitTSturcture);

    I2C_Cmd(I2C1, ENABLE);
    I2C_AcknowledgeConfig(I2C1, ENABLE);
}


I2C_StatusTypeDef HAL_I2C_WaitEventFlagUntilTimeout(I2C_TypeDef *I2Cx, uint32_t I2C_EVENT, uint32_t Tickstart, uint32_t Timeout)
{
    while(!I2C_CheckEvent(I2Cx, I2C_EVENT))
    {
        if(Timeout != I2C_MAX_DELAY)
        {
            if(((get_tick() - Tickstart) > Timeout) || (Timeout == 0U))
            {
                if(!I2C_CheckEvent(I2Cx, I2C_EVENT))
                {
                    I2C_GenerateSTOP(I2Cx, ENABLE);
                    I2C_ClearFlag(I2Cx, I2C_FLAG_BUSY);
                    return I2C_TIMEOUT;
                }
            }
        }
    }
    return I2C_OK;
}


I2C_StatusTypeDef HAL_I2C_WaitOnFlagUntilTimeout(I2C_TypeDef *I2Cx, uint32_t I2C_FLAG, FlagStatus Flag, uint32_t Tickstart, uint32_t Timeout)
{
    while(I2C_GetFlagStatus(I2Cx, I2C_FLAG) != Flag)
    {
        if(Timeout != I2C_MAX_DELAY)
        {
            if(((get_tick() - Tickstart) > Timeout) || (Timeout == 0U))
            {
                if(I2C_GetFlagStatus(I2Cx, I2C_FLAG) != Flag)
                {
                    I2C_GenerateSTOP(I2Cx, ENABLE);
                    I2C_ClearFlag(I2Cx, I2C_FLAG_BUSY);
                    return I2C_TIMEOUT;
                }
            }
        }
    }
    return I2C_OK;
}

/**
  * @brief  Transmit an amount of data in blocking mode to slave
  * @param  i2c_periph variable for i2c peripherial
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
I2C_StatusTypeDef HAL_I2C_Transmit(I2C_TypeDef *I2Cx, uint8_t DevAddress,
        uint8_t *pData, uint16_t Size, uint32_t Timeout) {
    uint32_t Tickstart = get_tick();
    uint16_t count = Size;

    /* i2c master sends start signal only when the bus is idle */
    if (HAL_I2C_WaitOnFlagUntilTimeout(I2Cx, I2C_FLAG_BUSY, RESET, Tickstart, Timeout) != I2C_OK) {return I2C_BUSY;}

    /* send the start signal */
    I2C_GenerateSTART(I2Cx, ENABLE);

    if (HAL_I2C_WaitEventFlagUntilTimeout(I2Cx, I2C_EVENT_MASTER_MODE_SELECT, Tickstart, Timeout) != I2C_OK) {return I2C_BUSY;}

    I2C_Send7bitAddress(I2Cx, DevAddress, I2C_Direction_Transmitter);

    /* address flag set means i2c slave sends ACK */
    if (HAL_I2C_WaitEventFlagUntilTimeout(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED, Tickstart, Timeout)!= I2C_OK) {return I2C_ADDR_NACK;}

    for (count = 0; count < Size; count++) {
        I2C_SendData(I2Cx, *pData);
        /* wait until transmission complete */
        if (HAL_I2C_WaitEventFlagUntilTimeout(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED, Tickstart, Timeout)!= I2C_OK) {return I2C_TIMEOUT;}
        /* point to the next byte to be written */
        pData++;
    }

    I2C_GenerateSTOP(I2Cx, ENABLE);

    return I2C_OK;
}

/**
  * @brief  Receive i2c data in blocking mode from slave
  * @param  i2c_periph variable for i2c peripherial
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  MemAddSize Size of internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
I2C_StatusTypeDef HAL_I2C_Receive(I2C_TypeDef *I2Cx, uint8_t DevAddress,uint8_t *pData, uint16_t Size, uint32_t Timeout) {
    uint32_t Tickstart = get_tick();
    uint16_t count = Size;
    /* i2c master sends start signal only when the bus is idle */
    if (HAL_I2C_WaitOnFlagUntilTimeout(I2Cx, I2C_FLAG_BUSY, RESET, Tickstart,Timeout) != I2C_OK) {return I2C_BUSY;}
    /* send the start signal */
    I2C_GenerateSTART(I2Cx, ENABLE);

    if (HAL_I2C_WaitEventFlagUntilTimeout(I2Cx, I2C_EVENT_MASTER_MODE_SELECT, Tickstart, Timeout) != I2C_OK) {return I2C_BUSY;}

    I2C_Send7bitAddress(I2Cx, DevAddress, I2C_Direction_Receiver);
    /* address flag set means i2c slave sends ACK */
    if (HAL_I2C_WaitEventFlagUntilTimeout(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED, Tickstart, Timeout) != I2C_OK) {return I2C_ADDR_NACK;}

    I2C_AcknowledgeConfig(I2Cx, ENABLE);

    for (count = 0; count < Size; count++) {

        if (count >= (Size - 2)) {
            /*before receiving last byte and after receiving byte-1 disable ACK for last byte */
            I2C_AcknowledgeConfig(I2Cx, DISABLE);
        }
        /* receive data and store in buffer */
        *pData = I2C_ReceiveData(I2Cx);
        pData++;
        /* wait until data not received */
        if(HAL_I2C_WaitOnFlagUntilTimeout(I2Cx, I2C_FLAG_RXNE, SET, Tickstart, Timeout) != I2C_OK) {return I2C_TIMEOUT;}
    }

    I2C_GenerateSTOP(I2Cx, ENABLE);

    return I2C_OK;
}

/**
  * @brief  Write an amount of data in blocking mode to a specific memory address
  * @param  i2c_periph variable for i2c peripherial
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  MemAddress Internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
I2C_StatusTypeDef HAL_I2C_Mem_Write(I2C_TypeDef *I2Cx, uint8_t DevAddress, uint8_t MemAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout) {
    uint32_t Tickstart = get_tick();
    uint16_t count = Size;
    if(HAL_I2C_WaitOnFlagUntilTimeout(I2Cx, I2C_FLAG_BUSY, RESET, Tickstart, Timeout) != I2C_OK) {return I2C_BUSY;}
    /* send the start signal */
    I2C_GenerateSTART(I2Cx, ENABLE);

    if (HAL_I2C_WaitEventFlagUntilTimeout(I2Cx, I2C_EVENT_MASTER_MODE_SELECT, Tickstart, Timeout) != I2C_OK) {return I2C_BUSY;}

    I2C_Send7bitAddress(I2Cx, DevAddress, I2C_Direction_Transmitter);

    if (HAL_I2C_WaitEventFlagUntilTimeout(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED, Tickstart, Timeout) != I2C_OK) {return I2C_ADDR_NACK;}

    I2C_SendData(I2Cx, MemAddress);

    if (HAL_I2C_WaitEventFlagUntilTimeout(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED, Tickstart, Timeout) != I2C_OK) {return I2C_TIMEOUT;}

    for (count = 0; count < Size; count++) {
            I2C_SendData(I2Cx, *pData);
            /* wait until transmission complete */
            if (HAL_I2C_WaitEventFlagUntilTimeout(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED, Tickstart, Timeout)!= I2C_OK) {return I2C_TIMEOUT;}
            /* point to the next byte to be written */
            pData++;
        }

    I2C_GenerateSTOP(I2Cx, ENABLE);
    return I2C_OK;
}


/**
  * @brief  Read an amount of data in blocking mode from a specific memory address
  * @param  i2c_periph variable for i2c peripherial
  * @param  DevAddress Target device address: The device 7 bits address value
  *         in datasheet must be shifted to the left before calling the interface
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  */
I2C_StatusTypeDef HAL_I2C_Mem_Read (I2C_TypeDef *I2Cx, uint8_t DevAddress, uint8_t MemAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    uint32_t Tickstart = get_tick();
    uint16_t count = Size;
    if(HAL_I2C_WaitOnFlagUntilTimeout(I2Cx, I2C_FLAG_BUSY, RESET, Tickstart, Timeout) != I2C_OK) {return I2C_BUSY;}

    I2C_GenerateSTART(I2Cx, ENABLE);

    if (HAL_I2C_WaitEventFlagUntilTimeout(I2Cx, I2C_EVENT_MASTER_MODE_SELECT, Tickstart, Timeout) != I2C_OK) {return I2C_BUSY;}

    I2C_Send7bitAddress(I2Cx, DevAddress, I2C_Direction_Transmitter);

    if (HAL_I2C_WaitEventFlagUntilTimeout(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED, Tickstart, Timeout) != I2C_OK) {return I2C_ADDR_NACK;}

    I2C_SendData(I2Cx, MemAddress);

    if (HAL_I2C_WaitEventFlagUntilTimeout(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED, Tickstart, Timeout) != I2C_OK) {return I2C_TIMEOUT;}

    //??
    I2C_GenerateSTART(I2Cx, ENABLE);

    if (HAL_I2C_WaitEventFlagUntilTimeout(I2Cx, I2C_EVENT_MASTER_MODE_SELECT, Tickstart, Timeout) != I2C_OK) {return I2C_BUSY;}

    I2C_Send7bitAddress(I2Cx, DevAddress, I2C_Direction_Receiver);

    if (HAL_I2C_WaitEventFlagUntilTimeout(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED, Tickstart, Timeout) != I2C_OK) {return I2C_ADDR_NACK;}

    if (HAL_I2C_WaitEventFlagUntilTimeout(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED, Tickstart, Timeout) != I2C_OK) {return I2C_TIMEOUT;}

    I2C_AcknowledgeConfig( I2Cx, ENABLE );

    for (count = 0; count < Size; count++) {

        if (count >= (Size - 2))
        {
            /*before receiving last byte and after receiving byte-1 disable ACK for last byte*/
            I2C_AcknowledgeConfig(I2Cx, DISABLE);
        }
        /*receive data and store in buffer*/
        *pData = I2C_ReceiveData(I2Cx);
        pData++;
        /*wait until data not received*/
        if (HAL_I2C_WaitOnFlagUntilTimeout(I2Cx, I2C_FLAG_RXNE, SET, Tickstart, Timeout) != I2C_OK) {return I2C_TIMEOUT;}
    }

    I2C_GenerateSTOP(I2Cx, ENABLE);

    return I2C_OK;
}

I2C_StatusTypeDef HAL_I2C_Read_Protected(I2C_TypeDef *I2Cx, uint8_t device_address, uint8_t register_address, uint8_t *data, uint16_t Size, uint32_t Timeout)
{
    uint32_t Tickstart = get_tick();
    if(HAL_I2C_WaitOnFlagUntilTimeout(I2Cx, I2C_FLAG_BUSY, RESET, Tickstart, Timeout) != I2C_OK) {return I2C_BUSY;}

    I2C_GenerateSTART(I2Cx, ENABLE);

    if (HAL_I2C_WaitEventFlagUntilTimeout(I2Cx, I2C_EVENT_MASTER_MODE_SELECT, Tickstart, Timeout) != I2C_OK) {return I2C_BUSY;}

    I2C_Send7bitAddress(I2Cx, device_address, I2C_Direction_Transmitter);

    if (HAL_I2C_WaitEventFlagUntilTimeout(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED, Tickstart, Timeout) != I2C_OK) {return I2C_BUSY;}

    I2C_SendData(I2Cx, register_address);

    if (HAL_I2C_WaitEventFlagUntilTimeout(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED, Tickstart, Timeout) != I2C_OK) {return I2C_BUSY;}

    I2C_GenerateSTART(I2Cx, ENABLE);

    if (HAL_I2C_WaitEventFlagUntilTimeout(I2Cx, I2C_EVENT_MASTER_MODE_SELECT, Tickstart, Timeout) != I2C_OK) {return I2C_BUSY;}

    I2C_Send7bitAddress(I2C1, device_address, I2C_Direction_Receiver);

    if (HAL_I2C_WaitEventFlagUntilTimeout(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED, Tickstart, Timeout) != I2C_OK) {return I2C_BUSY;}

    if (HAL_I2C_WaitEventFlagUntilTimeout(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED, Tickstart, Timeout) != I2C_OK) {return I2C_BUSY;}

    if(HAL_I2C_WaitOnFlagUntilTimeout(I2Cx, I2C_FLAG_RXNE, SET, Tickstart, Timeout) != I2C_OK) {return I2C_BUSY;}
    *data = I2C_ReceiveData(I2Cx);

    I2C_GenerateSTOP(I2Cx, ENABLE);
    return I2C_OK;
}
