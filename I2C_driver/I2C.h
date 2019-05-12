/*
 * I2C.h
 *
 *  Created on: Feb 19, 2019
 *      Author: Administrator
 */

#ifndef I2C_H_
#define I2C_H_

void I2C_init(I2CObject_t *ptI2CObject);

uint8_t I2C_scan(I2CObject_t *ptI2CObject);

I2CStatus_t I2C_writeRegByte(I2CObject_t *ptI2CObject, uint8_t ui8SlaveAddr,
                             uint8_t ui8RegAddr, uint8_t ui8RegData);

I2CStatus_t I2C_readRegByte(I2CObject_t *ptI2CObject, uint8_t ui8SlaveAddr,
                            uint8_t ui8RegAddr, uint8_t *pui8RegData);

I2CStatus_t I2C_writeRegBytes(I2CObject_t *ptI2CObject, uint8_t ui8SlaveAddr,
                              void *pvRegAddr, uint32_t ui32RegAddrLen,
                              void *pvRegData, uint32_t ui32RegDataLen);

I2CStatus_t I2C_readRegBytes(I2CObject_t *ptI2CObject, uint8_t ui8SlaveAddr,
                             void *pvRegAddr, uint32_t ui32RegAddrLen,
                             void *pvRegData, uint32_t ui32RegDataLen);

inline void I2C_writeSlaveAddr(I2CObject_t *ptI2CObject, uint8_t ui8SlaveAddr,
                               I2CDirection_t tI2CDirection)
{
    I2CMasterSlaveAddrSet(ptI2CObject->ui32I2cAddrBase, ui8SlaveAddr,
                          tI2CDirection); /* 0 > transmit /\ 1 > receive */
}

inline void I2C_writeSlaveData(I2CObject_t *ptI2CObject, uint8_t ui8Data)
{
    I2CMasterDataPut(ptI2CObject->ui32I2cAddrBase, ui8Data);
}

inline uint8_t I2C_readSlaveData(I2CObject_t *ptI2CObject)
{
    return I2CMasterDataGet(ptI2CObject->ui32I2cAddrBase);
}

inline void I2C_sendCommand(I2CObject_t *ptI2CObject, I2CCommand_t tI2CCommand)
{
    switch (tI2CCommand)
    {
//    case I2C_SINGLE_SEND: /* same as I2C_SINGLE_RECEIVE */
    case I2C_SINGLE_RECEIVE:
    case I2C_BURST_SEND_START:
    case I2C_BURST_RECEIVE_START:
        while (I2CMasterBusBusy(ptI2CObject->ui32I2cAddrBase)) /* Wait if there are any masters */
        {

        }
        break;
    }

    I2CMasterControl(ptI2CObject->ui32I2cAddrBase, tI2CCommand);
    while (I2CMasterBusy(ptI2CObject->ui32I2cAddrBase))
    {

    }
}

inline I2CStatus_t I2C_getError(I2CObject_t *ptI2CObject)
{
    return (I2CStatus_t) I2CMasterErr(ptI2CObject->ui32I2cAddrBase);
}

void delayMillis(uint32_t ui32Millis);

#endif /* I2C_H_ */
