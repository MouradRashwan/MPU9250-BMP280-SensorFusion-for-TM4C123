/*
 * I2C.c
 *
 *  Created on: Feb 19, 2019
 *      Author: Administrator
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "I2C_driver.h"

void I2C_init(I2CObject_t *ptI2CObject)
{
    SysCtlPeripheralEnable(ptI2CObject->ui32PortClkBase);
    while (!SysCtlPeripheralReady(ptI2CObject->ui32PortClkBase))
    {
    }
    GPIOPinConfigure(ptI2CObject->ui32PortPctlSCL);
    GPIOPinTypeI2CSCL(ptI2CObject->ui32PortAddrBase,
                      ptI2CObject->ui32PortPinSCL);
    GPIOPinConfigure(ptI2CObject->ui32PortPctlSDA);
    GPIOPinTypeI2C(ptI2CObject->ui32PortAddrBase, ptI2CObject->ui32PortPinSDA);

    SysCtlPeripheralEnable(ptI2CObject->ui32I2cClkBase);
    while (!SysCtlPeripheralReady(ptI2CObject->ui32I2cClkBase))
    {
    }
    I2CMasterInitExpClk(ptI2CObject->ui32I2cAddrBase, SysCtlClockGet(), true); /* false = 100Kbps || true = 400Kbps */
}

uint8_t I2C_scan(I2CObject_t *ptI2CObject)
{
    I2CStatus_t tI2CStatus;
    uint8_t ui8DeviceAddr, ui8DeviceCount = 0;

    for (ui8DeviceAddr = 0; ui8DeviceAddr < I2C_DEVICES_MAX; ui8DeviceAddr++)
    {
        I2CMasterSlaveAddrSet(ptI2CObject->ui32I2cAddrBase, ui8DeviceAddr,
                              TRANSMIT);
        while (I2CMasterBusBusy(ptI2CObject->ui32I2cAddrBase)) /* Wait if there are any masters */
        {

        }

        I2CMasterDataPut(ptI2CObject->ui32I2cAddrBase, 0);
        I2CMasterControl(ptI2CObject->ui32I2cAddrBase, I2C_SINGLE_SEND);
        while (I2CMasterBusy(ptI2CObject->ui32I2cAddrBase)) /* Wait until operation is done */
        {

        }

        tI2CStatus = (I2CStatus_t) I2CMasterErr(ptI2CObject->ui32I2cAddrBase);
        if (tI2CStatus != I2C_SUCCESS) /* Check if there are any errors */
        {
            I2CMasterControl(ptI2CObject->ui32I2cAddrBase,
                             I2C_BURST_SEND_ERROR_STOP); /* [STOP] */
            while (I2CMasterBusy(ptI2CObject->ui32I2cAddrBase))
            {

            }
        }
        else
        {
            ui8DeviceCount += 1;
        }
    }

    return ui8DeviceCount;
}

I2CStatus_t I2C_writeRegByte(I2CObject_t *ptI2CObject, uint8_t ui8SlaveAddr,
                             uint8_t ui8RegAddr, uint8_t ui8RegData)
{
    I2CStatus_t tI2CStatus;

    I2CMasterSlaveAddrSet(ptI2CObject->ui32I2cAddrBase, ui8SlaveAddr, TRANSMIT);
    while (I2CMasterBusBusy(ptI2CObject->ui32I2cAddrBase)) /* Wait if there are any masters */
    {

    }

    I2CMasterDataPut(ptI2CObject->ui32I2cAddrBase, ui8RegAddr);
    I2CMasterControl(ptI2CObject->ui32I2cAddrBase, I2C_BURST_SEND_START);
    while (I2CMasterBusy(ptI2CObject->ui32I2cAddrBase)) /* Wait until operation is done */
    {

    }

    tI2CStatus = (I2CStatus_t) I2CMasterErr(ptI2CObject->ui32I2cAddrBase);
    if (tI2CStatus != I2C_SUCCESS) /* Check if there are any errors */
    {
        I2CMasterControl(ptI2CObject->ui32I2cAddrBase,
                         I2C_BURST_SEND_ERROR_STOP); /* [STOP] */
        while (I2CMasterBusy(ptI2CObject->ui32I2cAddrBase))
        {

        }
        return tI2CStatus;
    }

    I2CMasterDataPut(ptI2CObject->ui32I2cAddrBase, ui8RegData);
    I2CMasterControl(ptI2CObject->ui32I2cAddrBase, I2C_BURST_SEND_FINISH);
    while (I2CMasterBusy(ptI2CObject->ui32I2cAddrBase)) /* Wait until operation is done */
    {

    }

    tI2CStatus = (I2CStatus_t) I2CMasterErr(ptI2CObject->ui32I2cAddrBase);
    if (tI2CStatus != I2C_SUCCESS) /* Check if there are any errors */
    {
        I2CMasterControl(ptI2CObject->ui32I2cAddrBase,
                         I2C_BURST_SEND_ERROR_STOP); /* [STOP] */
        while (I2CMasterBusy(ptI2CObject->ui32I2cAddrBase))
        {

        }
        return tI2CStatus;
    }

    return tI2CStatus;
}

I2CStatus_t I2C_readRegByte(I2CObject_t *ptI2CObject, uint8_t ui8SlaveAddr,
                            uint8_t ui8RegAddr, uint8_t *pui8RegData)
{
    I2CStatus_t tI2CStatus;

    I2CMasterSlaveAddrSet(ptI2CObject->ui32I2cAddrBase, ui8SlaveAddr, TRANSMIT);
    while (I2CMasterBusBusy(ptI2CObject->ui32I2cAddrBase)) /* Wait if there are any masters */
    {

    }

    I2CMasterDataPut(ptI2CObject->ui32I2cAddrBase, ui8RegAddr);
    I2CMasterControl(ptI2CObject->ui32I2cAddrBase, I2C_SINGLE_SEND);
    while (I2CMasterBusy(ptI2CObject->ui32I2cAddrBase)) /* Wait until operation is done */
    {

    }

    tI2CStatus = (I2CStatus_t) I2CMasterErr(ptI2CObject->ui32I2cAddrBase);
    if (tI2CStatus != I2C_SUCCESS) /* Check if there are any errors */
    {
        I2CMasterControl(ptI2CObject->ui32I2cAddrBase,
                         I2C_BURST_SEND_ERROR_STOP); /* [STOP] */
        while (I2CMasterBusy(ptI2CObject->ui32I2cAddrBase))
        {

        }
        return tI2CStatus;
    }

    I2CMasterSlaveAddrSet(ptI2CObject->ui32I2cAddrBase, ui8SlaveAddr, RECEIVE);
    while (I2CMasterBusBusy(ptI2CObject->ui32I2cAddrBase)) /* Wait if there are any masters */
    {

    }

    I2CMasterControl(ptI2CObject->ui32I2cAddrBase, I2C_SINGLE_RECEIVE);
    while (I2CMasterBusy(ptI2CObject->ui32I2cAddrBase)) /* Wait until operation is done */
    {

    }

    tI2CStatus = (I2CStatus_t) I2CMasterErr(ptI2CObject->ui32I2cAddrBase);
    if (tI2CStatus != I2C_SUCCESS) /* Check if there are any errors */
    {
        I2CMasterControl(ptI2CObject->ui32I2cAddrBase,
                         I2C_BURST_SEND_ERROR_STOP); /* [STOP] */
        while (I2CMasterBusy(ptI2CObject->ui32I2cAddrBase))
        {

        }
        return tI2CStatus;
    }

    *pui8RegData = I2CMasterDataGet(ptI2CObject->ui32I2cAddrBase);

    return tI2CStatus;
}

I2CStatus_t I2C_writeRegBytes(I2CObject_t *ptI2CObject, uint8_t ui8SlaveAddr,
                              void *pvRegAddr, uint32_t ui32RegAddrLen,
                              void *pvRegData, uint32_t ui32RegDataLen)
{
    uint32_t i;
    I2CStatus_t tI2CStatus;
    I2CCommand_t tI2CCommand;

    I2CMasterSlaveAddrSet(ptI2CObject->ui32I2cAddrBase, ui8SlaveAddr, TRANSMIT);
    while (I2CMasterBusBusy(ptI2CObject->ui32I2cAddrBase)) /* Wait if there are any masters */
    {

    }

    for (i = 0; i < ui32RegAddrLen; i++)
    {
        if (i == 0) /* first byte to send */
        {
            tI2CCommand = I2C_BURST_SEND_START;
        }
        else
        {
            tI2CCommand = I2C_BURST_SEND_CONT;
        }

        I2CMasterDataPut(ptI2CObject->ui32I2cAddrBase,
                         ((uint8_t*) pvRegAddr)[i]);
        I2CMasterControl(ptI2CObject->ui32I2cAddrBase, tI2CCommand);
        while (I2CMasterBusy(ptI2CObject->ui32I2cAddrBase)) /* Wait until operation is done */
        {

        }

        tI2CStatus = (I2CStatus_t) I2CMasterErr(ptI2CObject->ui32I2cAddrBase);
        if (tI2CStatus != I2C_SUCCESS) /* Check if there are any errors */
        {
            I2CMasterControl(ptI2CObject->ui32I2cAddrBase,
                             I2C_BURST_SEND_ERROR_STOP); /* [STOP] */
            while (I2CMasterBusy(ptI2CObject->ui32I2cAddrBase))
            {

            }
            return tI2CStatus;
        }
    }

    for (i = 0; i < ui32RegDataLen; i++)
    {
        if (i == (ui32RegDataLen - 1)) /* last byte to send */
        {
            tI2CCommand = I2C_BURST_SEND_FINISH;
        }
        else
        {
            tI2CCommand = I2C_BURST_SEND_CONT;
        }

        I2CMasterDataPut(ptI2CObject->ui32I2cAddrBase,
                         ((uint8_t*) pvRegData)[i]);
        I2CMasterControl(ptI2CObject->ui32I2cAddrBase, tI2CCommand);
        while (I2CMasterBusy(ptI2CObject->ui32I2cAddrBase)) /* Wait until operation is done */
        {

        }

        tI2CStatus = (I2CStatus_t) I2CMasterErr(ptI2CObject->ui32I2cAddrBase);
        if (tI2CStatus != I2C_SUCCESS) /* Check if there are any errors */
        {
            I2CMasterControl(ptI2CObject->ui32I2cAddrBase,
                             I2C_BURST_SEND_ERROR_STOP); /* [STOP] */
            while (I2CMasterBusy(ptI2CObject->ui32I2cAddrBase))
            {

            }
            return tI2CStatus;
        }
    }

    return tI2CStatus;
}

I2CStatus_t I2C_readRegBytes(I2CObject_t *ptI2CObject, uint8_t ui8SlaveAddr,
                             void *pvRegAddr, uint32_t ui32RegAddrLen,
                             void *pvRegData, uint32_t ui32RegDataLen)
{
    uint32_t i;
    I2CStatus_t tI2CStatus;
    I2CCommand_t tI2CCommand;

    I2CMasterSlaveAddrSet(ptI2CObject->ui32I2cAddrBase, ui8SlaveAddr, TRANSMIT);
    while (I2CMasterBusBusy(ptI2CObject->ui32I2cAddrBase)) /* Wait if there are any masters */
    {

    }

    for (i = 0; i < ui32RegAddrLen; i++)
    {
        if (i == 0) /* first byte to send */
        {
            tI2CCommand = I2C_BURST_SEND_START;
            if (ui32RegAddrLen == 1)
            {
                tI2CCommand = I2C_SINGLE_SEND;
            }
        }
        else if (i == (ui32RegAddrLen - 1)) /* last byte to send */
        {
            tI2CCommand = I2C_BURST_SEND_FINISH;
        }
        else
        {
            tI2CCommand = I2C_BURST_SEND_CONT;
        }

        I2CMasterDataPut(ptI2CObject->ui32I2cAddrBase,
                         ((uint8_t*) pvRegAddr)[i]);
        I2CMasterControl(ptI2CObject->ui32I2cAddrBase, tI2CCommand);
        while (I2CMasterBusy(ptI2CObject->ui32I2cAddrBase)) /* Wait until operation is done */
        {

        }

        tI2CStatus = (I2CStatus_t) I2CMasterErr(ptI2CObject->ui32I2cAddrBase);
        if (tI2CStatus != I2C_SUCCESS) /* Check if there are any errors */
        {
            I2CMasterControl(ptI2CObject->ui32I2cAddrBase,
                             I2C_BURST_SEND_ERROR_STOP); /* [STOP] */
            while (I2CMasterBusy(ptI2CObject->ui32I2cAddrBase))
            {

            }
            return tI2CStatus;
        }
    }

    I2CMasterSlaveAddrSet(ptI2CObject->ui32I2cAddrBase, ui8SlaveAddr, RECEIVE);
    while (I2CMasterBusBusy(ptI2CObject->ui32I2cAddrBase)) /* Wait if there are any masters */
    {

    }

    for (i = 0; i < ui32RegDataLen; i++)
    {
        if (i == 0) /* first byte to receive */
        {
            tI2CCommand = I2C_BURST_RECEIVE_START;
            if (ui32RegDataLen == 1)
            {
                tI2CCommand = I2C_SINGLE_RECEIVE;
            }
        }
        else if (i == (ui32RegDataLen - 1)) /* last byte to receive */
        {
            tI2CCommand = I2C_BURST_RECEIVE_FINISH;
        }
        else
        {
            tI2CCommand = I2C_BURST_RECEIVE_CONT;
        }

        I2CMasterControl(ptI2CObject->ui32I2cAddrBase, tI2CCommand);
        while (I2CMasterBusy(ptI2CObject->ui32I2cAddrBase)) /* Wait until operation is done */
        {

        }

        tI2CStatus = (I2CStatus_t) I2CMasterErr(ptI2CObject->ui32I2cAddrBase);
        if (tI2CStatus != I2C_SUCCESS) /* Check if there are any errors */
        {
            I2CMasterControl(ptI2CObject->ui32I2cAddrBase,
                             I2C_BURST_RECEIVE_ERROR_STOP); /* [STOP] */
            while (I2CMasterBusy(ptI2CObject->ui32I2cAddrBase))
            {

            }
            return tI2CStatus;
        }

        ((uint8_t*) pvRegData)[i] = I2CMasterDataGet(
                ptI2CObject->ui32I2cAddrBase);
    }

    return tI2CStatus;
}

void delayMillis(uint32_t ui32Millis)
{
    uint32_t ix;
    uint32_t ui32Period = (SysCtlClockGet() / 1000U) / 3U;
    for (ix = 0; ix < ui32Millis; ix++)
    {
        SysCtlDelay(ui32Period);
    }
}
