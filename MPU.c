/*
 * MPU.c
 *
 *  Created on: Mar 17, 2019
 *      Author: Administrator
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include <math.h>

#include "MPU_driver.h"

static I2CObject_t g_tI2CObject = { .ui32I2cClkBase = MPU_I2C_CLK_BASE,
                                    .ui32I2cAddrBase = MPU_I2C_ADDR_BASE,
                                    .ui32PortClkBase = MPU_I2C_PORT_CLK,
                                    .ui32PortAddrBase = MPU_I2C_PORT_ADDR,
                                    .ui32PortPinSCL = MPU_I2C_SCL_PIN,
                                    .ui32PortPinSDA = MPU_I2C_SDA_PIN,
                                    .ui32PortPctlSCL = MPU_I2C_SCL_PCTL,
                                    .ui32PortPctlSDA = MPU_I2C_SDA_PCTL };

int32_t g_i32BMP280Tfine;
BMP280Dig_t g_tBMP280Dig;

MPU9250Abias_t g_tAccelBias = { 0.123657227f, 0.0778198242f, -0.166381836f };
MPU9250Gbias_t g_tGyroBias = { 0.358778626f, -0.396946579f, 1.78625953f };

AK8963Mcalib_t g_tMagCalib = { 1.1953125f, 1.1953125f, 1.15234375f };
AK8963Mbias_t g_tMagBias = { 292.134979f, 227.614365f, 84.662796f };
AK8963MscaleCal_t g_tMagScaleCal = { 0.484848469f, 1.96491218f, 2.33333325f };

uint8_t MPU_I2C_initScan(void)
{
    uint8_t ui8DevicesNum;

    /* I2C initialize */
    I2C_init(&g_tI2CObject);
    /* Scan I2C bus to get the no.of devices connected */
    ui8DevicesNum = I2C_scan(&g_tI2CObject);

    return ui8DevicesNum;
}

MPUStatus_t MPU9250_getID(uint8_t *pui8ID)
{
    if (I2C_readRegByte(&g_tI2CObject, MPU9250_I2C_ADDR,
    MPU9250_WHO_AM_I,
                        pui8ID) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

    return MPU_SUCCESS;
}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
// Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
MPUStatus_t MPU9250_SelfTest(float adDeviations[6])
{
    uint8_t ii, ui8RegAddr;
    uint8_t aui8RawData[6];
    uint8_t selfTest[6];
    float factoryTrim[6];
    int32_t gAvg[3], aAvg[3], aSTAvg[3], gSTAvg[3];

    for (ii = 0; ii < 3; ii++)
    {
        gAvg[ii] = 0;
        aAvg[ii] = 0;
        aSTAvg[ii] = 0;
        gSTAvg[ii] = 0;
    }

//    writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00); // Set gyro sample rate to 1 kHz
    if (I2C_writeRegByte(&g_tI2CObject, MPU9250_I2C_ADDR,
    MPU9250_SMPLRT_DIV,
                         0x00) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

//    writeByte(MPU9250_ADDRESS, CONFIG, 0x02); // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
    if (I2C_writeRegByte(&g_tI2CObject, MPU9250_I2C_ADDR,
    MPU9250_CONFIG,
                         0x02) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

//    writeByte(MPU9250_ADDRESS, GYRO_CONFIG, FS << 3); // Set full scale range for the gyro to 250 dps
    if (I2C_writeRegByte(&g_tI2CObject, MPU9250_I2C_ADDR,
    MPU9250_GYRO_CONFIG,
                         (MPU9250_GYRO_FULL_SCALE << 3)) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

//    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
    if (I2C_writeRegByte(&g_tI2CObject, MPU9250_I2C_ADDR,
    MPU9250_ACCEL_CONFIG2,
                         0x02) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

//    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, FS << 3); // Set full scale range for the accelerometer to 2 g
    if (I2C_writeRegByte(&g_tI2CObject, MPU9250_I2C_ADDR,
    MPU9250_ACCEL_CONFIG,
                         (MPU9250_ACCEL_FULL_SCALE << 3)) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

    for (ii = 0; ii < 200; ii++)
    {  // get average current values of gyro and acclerometer

//        readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &aui8RawData[0]); // Read the six raw data registers into data array
        ui8RegAddr = MPU9250_ACCEL_XOUT_H;
        if (I2C_readRegBytes(&g_tI2CObject, MPU9250_I2C_ADDR, &ui8RegAddr, 1,
                             aui8RawData, 6) != I2C_SUCCESS)
        {
            return MPU_ERROR;
        }

        aAvg[0] += (int16_t) (((int16_t) aui8RawData[0] << 8) | aui8RawData[1]); // Turn the MSB and LSB into a signed 16-bit value
        aAvg[1] += (int16_t) (((int16_t) aui8RawData[2] << 8) | aui8RawData[3]);
        aAvg[2] += (int16_t) (((int16_t) aui8RawData[4] << 8) | aui8RawData[5]);

//        readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &aui8RawData[0]); // Read the six raw data registers sequentially into data array
        ui8RegAddr = MPU9250_GYRO_XOUT_H;
        if (I2C_readRegBytes(&g_tI2CObject, MPU9250_I2C_ADDR, &ui8RegAddr, 1,
                             aui8RawData, 6) != I2C_SUCCESS)
        {
            return MPU_ERROR;
        }

        gAvg[0] += (int16_t) (((int16_t) aui8RawData[0] << 8) | aui8RawData[1]); // Turn the MSB and LSB into a signed 16-bit value
        gAvg[1] += (int16_t) (((int16_t) aui8RawData[2] << 8) | aui8RawData[3]);
        gAvg[2] += (int16_t) (((int16_t) aui8RawData[4] << 8) | aui8RawData[5]);
    }

    for (ii = 0; ii < 3; ii++)
    {  // Get average of 200 values and store as average current readings
        aAvg[ii] /= 200;
        gAvg[ii] /= 200;
    }

// Configure the accelerometer for self-test
//    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
    if (I2C_writeRegByte(&g_tI2CObject, MPU9250_I2C_ADDR,
    MPU9250_ACCEL_CONFIG,
                         0xE0) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

//    writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
    if (I2C_writeRegByte(&g_tI2CObject, MPU9250_I2C_ADDR,
    MPU9250_GYRO_CONFIG,
                         0xE0) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

    delayMillis(25);  // Delay a while to let the device stabilize

    for (ii = 0; ii < 200; ii++)
    {  // get average self-test values of gyro and acclerometer

//        readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &aui8RawData[0]); // Read the six raw data registers into data array
        ui8RegAddr = MPU9250_ACCEL_XOUT_H;
        if (I2C_readRegBytes(&g_tI2CObject, MPU9250_I2C_ADDR, &ui8RegAddr, 1,
                             aui8RawData, 6) != I2C_SUCCESS)
        {
            return MPU_ERROR;
        }

        aSTAvg[0] +=
                (int16_t) (((int16_t) aui8RawData[0] << 8) | aui8RawData[1]); // Turn the MSB and LSB into a signed 16-bit value
        aSTAvg[1] +=
                (int16_t) (((int16_t) aui8RawData[2] << 8) | aui8RawData[3]);
        aSTAvg[2] +=
                (int16_t) (((int16_t) aui8RawData[4] << 8) | aui8RawData[5]);

//        readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &aui8RawData[0]); // Read the six raw data registers sequentially into data array
        ui8RegAddr = MPU9250_GYRO_XOUT_H;
        if (I2C_readRegBytes(&g_tI2CObject, MPU9250_I2C_ADDR, &ui8RegAddr, 1,
                             aui8RawData, 6) != I2C_SUCCESS)
        {
            return MPU_ERROR;
        }

        gSTAvg[0] +=
                (int16_t) (((int16_t) aui8RawData[0] << 8) | aui8RawData[1]); // Turn the MSB and LSB into a signed 16-bit value
        gSTAvg[1] +=
                (int16_t) (((int16_t) aui8RawData[2] << 8) | aui8RawData[3]);
        gSTAvg[2] +=
                (int16_t) (((int16_t) aui8RawData[4] << 8) | aui8RawData[5]);
    }

    for (ii = 0; ii < 3; ii++)
    {  // Get average of 200 values and store as average self-test readings
        aSTAvg[ii] /= 200;
        gSTAvg[ii] /= 200;
    }

    // Configure the gyro and accelerometer for normal operation
//    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00);
    if (I2C_writeRegByte(&g_tI2CObject, MPU9250_I2C_ADDR,
    MPU9250_ACCEL_CONFIG,
                         0x00) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

//    writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00);
    if (I2C_writeRegByte(&g_tI2CObject, MPU9250_I2C_ADDR,
    MPU9250_GYRO_CONFIG,
                         0x00) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

    delayMillis(25);  // Delay a while to let the device stabilize

    // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
//    selfTest[0] = readByte(MPU9250_ADDRESS, SELF_TEST_X_ACCEL); // X-axis accel self-test results
    if (I2C_readRegByte(&g_tI2CObject, MPU9250_I2C_ADDR,
    MPU9250_SELF_TEST_X_ACCEL,
                        &selfTest[0]) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

//    selfTest[1] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_ACCEL); // Y-axis accel self-test results
    if (I2C_readRegByte(&g_tI2CObject, MPU9250_I2C_ADDR,
    MPU9250_SELF_TEST_Y_ACCEL,
                        &selfTest[1]) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

//    selfTest[2] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_ACCEL); // Z-axis accel self-test results
    if (I2C_readRegByte(&g_tI2CObject, MPU9250_I2C_ADDR,
    MPU9250_SELF_TEST_Z_ACCEL,
                        &selfTest[2]) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

//    selfTest[3] = readByte(MPU9250_ADDRESS, SELF_TEST_X_GYRO); // X-axis gyro self-test results
    if (I2C_readRegByte(&g_tI2CObject, MPU9250_I2C_ADDR,
    MPU9250_SELF_TEST_X_GYRO,
                        &selfTest[3]) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

//    selfTest[4] = readByte(MPU9250_ADDRESS, SELF_TEST_Y_GYRO); // Y-axis gyro self-test results
    if (I2C_readRegByte(&g_tI2CObject, MPU9250_I2C_ADDR,
    MPU9250_SELF_TEST_Y_GYRO,
                        &selfTest[4]) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

//    selfTest[5] = readByte(MPU9250_ADDRESS, SELF_TEST_Z_GYRO); // Z-axis gyro self-test results
    if (I2C_readRegByte(&g_tI2CObject, MPU9250_I2C_ADDR,
    MPU9250_SELF_TEST_Z_GYRO,
                        &selfTest[5]) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

    // Retrieve factory self-test value from self-test code reads
    factoryTrim[0] = (float) (2620 / 1 << MPU9250_ACCEL_FULL_SCALE)
            * (powf(1.01, ((float) selfTest[0] - 1.0))); // FT[Xa] factory trim calculation
    factoryTrim[1] = (float) (2620 / 1 << MPU9250_ACCEL_FULL_SCALE)
            * (powf(1.01, ((float) selfTest[1] - 1.0))); // FT[Ya] factory trim calculation
    factoryTrim[2] = (float) (2620 / 1 << MPU9250_ACCEL_FULL_SCALE)
            * (powf(1.01, ((float) selfTest[2] - 1.0))); // FT[Za] factory trim calculation
    factoryTrim[3] = (float) (2620 / 1 << MPU9250_GYRO_FULL_SCALE)
            * (powf(1.01, ((float) selfTest[3] - 1.0))); // FT[Xg] factory trim calculation
    factoryTrim[4] = (float) (2620 / 1 << MPU9250_GYRO_FULL_SCALE)
            * (powf(1.01, ((float) selfTest[4] - 1.0))); // FT[Yg] factory trim calculation
    factoryTrim[5] = (float) (2620 / 1 << MPU9250_GYRO_FULL_SCALE)
            * (powf(1.01, ((float) selfTest[5] - 1.0))); // FT[Zg] factory trim calculation

    // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
    // To get percent, must multiply by 100
    for (ii = 0; ii < 3; ii++)
    {
        adDeviations[ii] = 100.0 * ((float) (aSTAvg[ii] - aAvg[ii]))
                / factoryTrim[ii] - 100.;   // Report percent differences
        adDeviations[ii + 3] = 100.0 * ((float) (gSTAvg[ii] - gAvg[ii]))
                / factoryTrim[ii + 3] - 100.; // Report percent differences
    }

    return MPU_SUCCESS;
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
MPUStatus_t MPU9250_calibrateAccelGyro(void)
{
    uint8_t ui8RegAddr;
    uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
    uint16_t ii, packet_count, fifo_count;
    int32_t gyro_bias[3] = { 0, 0, 0 }, accel_bias[3] = { 0, 0, 0 };
    uint16_t gyrosensitivity = 131;   // = 131 LSB/degrees/sec
    uint16_t accelsensitivity = 16384;  // = 16384 LSB/g

    // reset device
//    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
    if (I2C_writeRegByte(&g_tI2CObject, MPU9250_I2C_ADDR,
    MPU9250_PWR_MGMT_1,
                         0x80) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

    delayMillis(100);

    // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
    // else use the internal oscillator, bits 2:0 = 001
//    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01);
    if (I2C_writeRegByte(&g_tI2CObject, MPU9250_I2C_ADDR,
    MPU9250_PWR_MGMT_1,
                         0x01) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

//    writeByte(MPU9250_ADDRESS, PWR_MGMT_2, 0x00);
    if (I2C_writeRegByte(&g_tI2CObject, MPU9250_I2C_ADDR,
    MPU9250_PWR_MGMT_2,
                         0x00) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

    delayMillis(200);

// Configure device for bias calculation
//    writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
    if (I2C_writeRegByte(&g_tI2CObject, MPU9250_I2C_ADDR,
    MPU9250_INT_ENABLE,
                         0x00) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

//    writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
    if (I2C_writeRegByte(&g_tI2CObject, MPU9250_I2C_ADDR,
    MPU9250_FIFO_EN,
                         0x00) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

//    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Turn on internal clock source
    if (I2C_writeRegByte(&g_tI2CObject, MPU9250_I2C_ADDR,
    MPU9250_PWR_MGMT_1,
                         0x00) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

//    writeByte(MPU9250_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
    if (I2C_writeRegByte(&g_tI2CObject, MPU9250_I2C_ADDR,
    MPU9250_I2C_MST_CTRL,
                         0x00) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

//    writeByte(MPU9250_ADDRESS, USER_CTRL, 0x00); // Disable FIFO and I2C master modes
    if (I2C_writeRegByte(&g_tI2CObject, MPU9250_I2C_ADDR,
    MPU9250_USER_CTRL,
                         0x00) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

//    writeByte(MPU9250_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
    if (I2C_writeRegByte(&g_tI2CObject, MPU9250_I2C_ADDR,
    MPU9250_USER_CTRL,
                         0x0C) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

    delayMillis(15);

// Configure MPU6050 gyro and accelerometer for bias calculation
//    writeByte(MPU9250_ADDRESS, CONFIG, 0x01);   // Set low-pass filter to 188 Hz
    if (I2C_writeRegByte(&g_tI2CObject, MPU9250_I2C_ADDR,
    MPU9250_CONFIG,
                         0x01) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

//    writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
    if (I2C_writeRegByte(&g_tI2CObject, MPU9250_I2C_ADDR,
    MPU9250_SMPLRT_DIV,
                         0x00) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

//    writeByte(MPU9250_ADDRESS, GYRO_CONFIG, 0x00); // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    if (I2C_writeRegByte(&g_tI2CObject, MPU9250_I2C_ADDR,
    MPU9250_GYRO_CONFIG,
                         0x00) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

//    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity
    if (I2C_writeRegByte(&g_tI2CObject, MPU9250_I2C_ADDR,
    MPU9250_ACCEL_CONFIG,
                         0x00) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

// Configure FIFO to capture accelerometer and gyro data for bias calculation
//    writeByte(MPU9250_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO
    if (I2C_writeRegByte(&g_tI2CObject, MPU9250_I2C_ADDR,
    MPU9250_USER_CTRL,
                         0x40) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

//    writeByte(MPU9250_ADDRESS, FIFO_EN, 0x78); // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
    if (I2C_writeRegByte(&g_tI2CObject, MPU9250_I2C_ADDR,
    MPU9250_FIFO_EN,
                         0x78) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

    delayMillis(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

// At end of sample accumulation, turn off FIFO sensor read
//    writeByte(MPU9250_ADDRESS, FIFO_EN, 0x00); // Disable gyro and accelerometer sensors for FIFO
    if (I2C_writeRegByte(&g_tI2CObject, MPU9250_I2C_ADDR,
    MPU9250_FIFO_EN,
                         0x00) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

//    readBytes(MPU9250_ADDRESS, FIFO_COUNTH, 2, &data[0]); // read FIFO sample count
    ui8RegAddr = MPU9250_FIFO_COUNTH;
    if (I2C_readRegBytes(&g_tI2CObject, MPU9250_I2C_ADDR, &ui8RegAddr, 1,
                         &data[0], 2) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

    fifo_count = ((uint16_t) data[0] << 8) | data[1];
    packet_count = fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging

    for (ii = 0; ii < packet_count; ii++)
    {
        int16_t accel_temp[3] = { 0, 0, 0 }, gyro_temp[3] = { 0, 0, 0 };
//        readBytes(MPU9250_ADDRESS, FIFO_R_W, 12, &data[0]); // read data for averaging
        ui8RegAddr = MPU9250_FIFO_R_W;
        if (I2C_readRegBytes(&g_tI2CObject, MPU9250_I2C_ADDR, &ui8RegAddr, 1,
                             &data[0], 12) != I2C_SUCCESS)
        {
            return MPU_ERROR;
        }

        accel_temp[0] = (int16_t) (((int16_t) data[0] << 8) | data[1]); // Form signed 16-bit integer for each sample in FIFO
        accel_temp[1] = (int16_t) (((int16_t) data[2] << 8) | data[3]);
        accel_temp[2] = (int16_t) (((int16_t) data[4] << 8) | data[5]);
        gyro_temp[0] = (int16_t) (((int16_t) data[6] << 8) | data[7]);
        gyro_temp[1] = (int16_t) (((int16_t) data[8] << 8) | data[9]);
        gyro_temp[2] = (int16_t) (((int16_t) data[10] << 8) | data[11]);

        accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        accel_bias[1] += (int32_t) accel_temp[1];
        accel_bias[2] += (int32_t) accel_temp[2];
        gyro_bias[0] += (int32_t) gyro_temp[0];
        gyro_bias[1] += (int32_t) gyro_temp[1];
        gyro_bias[2] += (int32_t) gyro_temp[2];

    }
    accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t) packet_count;
    accel_bias[2] /= (int32_t) packet_count;
    gyro_bias[0] /= (int32_t) packet_count;
    gyro_bias[1] /= (int32_t) packet_count;
    gyro_bias[2] /= (int32_t) packet_count;

    if (accel_bias[2] > 0L)
    {
        accel_bias[2] -= (int32_t) accelsensitivity;
    }  // Remove gravity from the z-axis accelerometer bias calculation
    else
    {
        accel_bias[2] += (int32_t) accelsensitivity;
    }

// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
    data[0] = (-gyro_bias[0] / 4 >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    data[1] = (-gyro_bias[0] / 4) & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
    data[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
    data[3] = (-gyro_bias[1] / 4) & 0xFF;
    data[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
    data[5] = (-gyro_bias[2] / 4) & 0xFF;

// Push gyro biases to hardware registers
//    writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
    if (I2C_writeRegByte(&g_tI2CObject, MPU9250_I2C_ADDR,
    MPU9250_XG_OFFSET_H,
                         data[0]) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

//    writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
    if (I2C_writeRegByte(&g_tI2CObject, MPU9250_I2C_ADDR,
    MPU9250_XG_OFFSET_L,
                         data[1]) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

//    writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
    if (I2C_writeRegByte(&g_tI2CObject, MPU9250_I2C_ADDR,
    MPU9250_YG_OFFSET_H,
                         data[2]) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

//    writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
    if (I2C_writeRegByte(&g_tI2CObject, MPU9250_I2C_ADDR,
    MPU9250_YG_OFFSET_L,
                         data[3]) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

//    writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
    if (I2C_writeRegByte(&g_tI2CObject, MPU9250_I2C_ADDR,
    MPU9250_ZG_OFFSET_H,
                         data[4]) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

//    writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);
    if (I2C_writeRegByte(&g_tI2CObject, MPU9250_I2C_ADDR,
    MPU9250_ZG_OFFSET_L,
                         data[5]) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

// Output scaled gyro biases for display in the main program
    g_tGyroBias.dBiasX = (float) gyro_bias[0] / (float) gyrosensitivity;
    g_tGyroBias.dBiasY = (float) gyro_bias[1] / (float) gyrosensitivity;
    g_tGyroBias.dBiasZ = (float) gyro_bias[2] / (float) gyrosensitivity;

// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
// the accelerometer biases calculated above must be divided by 8.

    int32_t accel_bias_reg[3] = { 0, 0, 0 }; // A place to hold the factory accelerometer trim biases
//    readBytes(MPU9250_ADDRESS, XA_OFFSET_H, 2, &data[0]); // Read factory accelerometer trim values
    ui8RegAddr = MPU9250_XA_OFFSET_H;
    if (I2C_readRegBytes(&g_tI2CObject, MPU9250_I2C_ADDR, &ui8RegAddr, 1,
                         &data[0], 2) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

    accel_bias_reg[0] = (int32_t) (((int16_t) data[0] << 8) | data[1]);
//    readBytes(MPU9250_ADDRESS, YA_OFFSET_H, 2, &data[0]);
    ui8RegAddr = MPU9250_YA_OFFSET_H;
    if (I2C_readRegBytes(&g_tI2CObject, MPU9250_I2C_ADDR, &ui8RegAddr, 1,
                         &data[0], 2) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

    accel_bias_reg[1] = (int32_t) (((int16_t) data[0] << 8) | data[1]);
//    readBytes(MPU9250_ADDRESS, ZA_OFFSET_H, 2, &data[0]);
    ui8RegAddr = MPU9250_ZA_OFFSET_H;
    if (I2C_readRegBytes(&g_tI2CObject, MPU9250_I2C_ADDR, &ui8RegAddr, 1,
                         &data[0], 2) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

    accel_bias_reg[2] = (int32_t) (((int16_t) data[0] << 8) | data[1]);

    uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
    uint8_t mask_bit[3] = { 0, 0, 0 }; // Define array to hold mask bit for each accelerometer bias axis

    for (ii = 0; ii < 3; ii++)
    {
        if ((accel_bias_reg[ii] & mask))
            mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
    }

    // Construct total accelerometer bias, including calculated average accelerometer bias from above
    accel_bias_reg[0] -= (accel_bias[0] / 8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
    accel_bias_reg[1] -= (accel_bias[1] / 8);
    accel_bias_reg[2] -= (accel_bias[2] / 8);

    data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
    data[1] = (accel_bias_reg[0]) & 0xFF;
    data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
    data[3] = (accel_bias_reg[1]) & 0xFF;
    data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
    data[5] = (accel_bias_reg[2]) & 0xFF;
    data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

// Apparently this is not working for the acceleration biases in the MPU-9250
// Are we handling the temperature correction bit properly?
// Push accelerometer biases to hardware registers
    /*  writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
     writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
     writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
     writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
     writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
     writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);
     */
// Output scaled accelerometer biases for display in the main program
    g_tAccelBias.dBiasX = (float) accel_bias[0] / (float) accelsensitivity;
    g_tAccelBias.dBiasY = (float) accel_bias[1] / (float) accelsensitivity;
    g_tAccelBias.dBiasZ = (float) accel_bias[2] / (float) accelsensitivity;

    return MPU_SUCCESS;
}

MPUStatus_t MPU9250_init(void)
{
    uint8_t ui8Data;

    /* wake up device */
//    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
    if (I2C_writeRegByte(&g_tI2CObject, MPU9250_I2C_ADDR,
    MPU9250_PWR_MGMT_1,
                         0x00) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

    delayMillis(100); // Wait for all registers to reset

    /* get stable time source */
//    writeByte(MPU9250_ADDRESS, PWR_MGMT_1, 0x01); // Auto select clock source to be PLL gyroscope reference if ready else
    if (I2C_writeRegByte(&g_tI2CObject, MPU9250_I2C_ADDR,
    MPU9250_PWR_MGMT_1,
                         0x01) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

    delayMillis(200);

    /*
     * Configure Gyro and Thermometer
     * Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
     * minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
     * be higher than 1 / 0.0059 = 170 Hz
     * DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
     * With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
     *
     * 0x00 -> Bandwidth=250HZ, Delay=1ms, SampleFreq=8KHz
     * 0x03 -> Bandwidth=41HZ, Delay=6ms, SampleFreq=1KHz
     */
//    writeByte(MPU9250_ADDRESS, CONFIG, 0x03);
    if (I2C_writeRegByte(&g_tI2CObject, MPU9250_I2C_ADDR,
    MPU9250_CONFIG,
                         0x03) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

    /* Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV) */
//    writeByte(MPU9250_ADDRESS, SMPLRT_DIV, 0x04); // Use a 200 Hz sample rate; a rate consistent with the filter update rate
    if (I2C_writeRegByte(&g_tI2CObject, MPU9250_I2C_ADDR,
    MPU9250_SMPLRT_DIV,
                         0x04) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }
    // determined inset in CONFIG above

    /*
     * Set gyroscope full scale range
     * Range selects FS_SEL and GFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
     */
//    ui8Data = readByte(MPU9250_ADDRESS, GYRO_CONFIG); // get current GYRO_CONFIG register value
    if (I2C_readRegByte(&g_tI2CObject, MPU9250_I2C_ADDR,
    MPU9250_GYRO_CONFIG,
                        &ui8Data) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

    // c = c & ~0xE0; // Clear self-test bits [7:5]
    ui8Data = ui8Data & ~0x03; // Clear Fchoice bits [1:0]
    ui8Data = ui8Data & ~0x18; // Clear GFS bits [4:3]
    ui8Data = ui8Data | (MPU9250_GYRO_FULL_SCALE << 3); // Set full scale range for the gyro
    // c =| 0x00; // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG

//    writeByte(MPU9250_ADDRESS, GYRO_CONFIG, ui8Data); // Write new GYRO_CONFIG value to register
    if (I2C_writeRegByte(&g_tI2CObject, MPU9250_I2C_ADDR,
    MPU9250_GYRO_CONFIG,
                         ui8Data) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

    /* Set accelerometer full-scale range configuration */
//    ui8Data = readByte(MPU9250_ADDRESS, ACCEL_CONFIG); // get current ACCEL_CONFIG register value
    if (I2C_readRegByte(&g_tI2CObject, MPU9250_I2C_ADDR,
    MPU9250_ACCEL_CONFIG,
                        &ui8Data) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

    // c = c & ~0xE0; // Clear self-test bits [7:5]
    ui8Data = ui8Data & ~0x18;  // Clear AFS bits [4:3]
    ui8Data = ui8Data | (MPU9250_ACCEL_FULL_SCALE << 3); // Set full scale range for the accelerometer

//    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG, ui8Data); // Write new ACCEL_CONFIG register value
    if (I2C_writeRegByte(&g_tI2CObject, MPU9250_I2C_ADDR,
    MPU9250_ACCEL_CONFIG,
                         ui8Data) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

    /*
     * Set accelerometer sample rate configuration
     * It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
     * accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
     */
//    ui8Data = readByte(MPU9250_ADDRESS, ACCEL_CONFIG2); // get current ACCEL_CONFIG2 register value
    if (I2C_readRegByte(&g_tI2CObject, MPU9250_I2C_ADDR,
    MPU9250_ACCEL_CONFIG2,
                        &ui8Data) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

    ui8Data = ui8Data & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
    ui8Data = ui8Data | 0x03; // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz

//    writeByte(MPU9250_ADDRESS, ACCEL_CONFIG2, ui8Data); // Write new ACCEL_CONFIG2 register value
    if (I2C_writeRegByte(&g_tI2CObject, MPU9250_I2C_ADDR,
    MPU9250_ACCEL_CONFIG2,
                         ui8Data) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

    /*
     * The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
     * but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting
     * Configure Interrupts and Bypass Enable
     * Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
     * clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
     * can join the I2C bus and all can be controlled by the Arduino as master
     *   writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x22);
     */
//    writeByte(MPU9250_ADDRESS, INT_PIN_CFG, 0x12); // INT is 50 microsecond pulse and any read to clear
    if (I2C_writeRegByte(&g_tI2CObject, MPU9250_I2C_ADDR,
    MPU9250_INT_PIN_CFG,
                         0x12) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

//    writeByte(MPU9250_ADDRESS, INT_ENABLE, 0x01); // Enable data ready (bit 0) interrupt
    if (I2C_writeRegByte(&g_tI2CObject, MPU9250_I2C_ADDR,
    MPU9250_INT_ENABLE,
                         0x01) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

    delayMillis(100);

    return MPU_SUCCESS;
}

MPUStatus_t AK8963_getID(uint8_t *pui8ID)
{
    if (I2C_readRegByte(&g_tI2CObject, AK8963_I2C_ADDR,
    WHO_AM_I_AK8963,
                        pui8ID) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

    return MPU_SUCCESS;
}

MPUStatus_t AK8963_init(void)
{
    // First extract the factory calibration for each magnetometer axis
    uint8_t aui8Calib[3];  // x/y/z gyro calibration data stored here
    uint8_t ui8RegAddr;

//    writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
    if (I2C_writeRegByte(&g_tI2CObject, AK8963_I2C_ADDR,
    AK8963_CNTL,
                         0x00) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

    delayMillis(10);

//    writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x0F); // Enter Fuse ROM access mode
    if (I2C_writeRegByte(&g_tI2CObject, AK8963_I2C_ADDR,
    AK8963_CNTL,
                         0x0F) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

    delayMillis(10);

//    readBytes(AK8963_ADDRESS, AK8963_ASAX, 3, &aui8Calib[0]); // Read the x-, y-, and z-axis calibration values
    ui8RegAddr = AK8963_ASAX;
    if (I2C_readRegBytes(&g_tI2CObject, AK8963_I2C_ADDR, &ui8RegAddr, 1,
                         aui8Calib, 3) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

    g_tMagCalib.dCalibX = (float) (aui8Calib[0] - 128) / 256. + 1.; // Return x-axis sensitivity adjustment values, etc.
    g_tMagCalib.dCalibY = (float) (aui8Calib[1] - 128) / 256. + 1.;
    g_tMagCalib.dCalibZ = (float) (aui8Calib[2] - 128) / 256. + 1.;

//    writeByte(AK8963_ADDRESS, AK8963_CNTL, 0x00); // Power down magnetometer
    if (I2C_writeRegByte(&g_tI2CObject, AK8963_I2C_ADDR,
    AK8963_CNTL,
                         0x00) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

    delayMillis(10);
    // Configure the magnetometer for continuous read and highest resolution
    // set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
    // and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
//    writeByte(AK8963_ADDRESS, AK8963_CNTL, Mscale << 4 | 0x06); // Set magnetometer data resolution and sample ODR
    // 0x02 for 8 Hz, 0x06 for 100 Hz continuous magnetometer data read
    if (I2C_writeRegByte(&g_tI2CObject, AK8963_I2C_ADDR,
    AK8963_CNTL,
                         (AK8963_MAG_FULL_SCALE << 4 | AK8963_MAG_MODE))
            != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

    delayMillis(10);

    return MPU_SUCCESS;
}

/*
 * Before calling this function -> Print this line.
 * Serial.println("Mag Calibration: Wave device in a figure eight until done!");
 *
 * After calling this function -> Print this line.
 * Serial.println("Mag Calibration done!");
 */
MPUStatus_t AK8963_calibrateMag(void)
{
    uint16_t ii, jj, sample_count = 0;
    float avg_rad;
    MPU9AxisRawData_t tMPU9AxisRawData;
    int32_t mag_bias[3], mag_scale[3];
    int16_t mag_max[3], mag_min[3], mag_temp[3];

    for (ii = 0; ii < 3; ii++)
    {
        mag_bias[ii] = 0;
        mag_scale[ii] = 0;
        mag_temp[ii] = 0;
    }

    for (ii = 0; ii < 3; ii++)
    {
        mag_max[ii] = INT16_MIN;
    }

    for (ii = 0; ii < 3; ii++)
    {
        mag_min[ii] = INT16_MAX;
    }

    delayMillis(4000);

    // shoot for ~fifteen seconds of mag data
    if (AK8963_MAG_MODE == MDR_8HZ)
    {
        sample_count = 128; // at 8 Hz ODR, new mag data is available every 125ms
    }
    if (AK8963_MAG_MODE == MDR_100HZ)
    {
        sample_count = 1500; // at 100 Hz ODR, new mag data is available every 10ms
    }

    for (ii = 0; ii < sample_count; ii++)
    {
        /* Read mag data */
        if (AK8963_readMagRawData(&tMPU9AxisRawData) != MPU_SUCCESS)
        {
            return MPU_ERROR;
        }

        mag_temp[0] = tMPU9AxisRawData.i16MagX;
        mag_temp[1] = tMPU9AxisRawData.i16MagY;
        mag_temp[2] = tMPU9AxisRawData.i16MagZ;

        for (jj = 0; jj < 3; jj++)
        {
            if (mag_temp[jj] > mag_max[jj])
                mag_max[jj] = mag_temp[jj];
            if (mag_temp[jj] < mag_min[jj])
                mag_min[jj] = mag_temp[jj];
        }

        if (AK8963_MAG_MODE == MDR_8HZ)
        {
            delayMillis(135); // at 8 Hz ODR, new mag data is available every 125 ms
        }

        if (AK8963_MAG_MODE == MDR_100HZ)
        {
            delayMillis(12); // at 100 Hz ODR, new mag data is available every 10 ms
        }
    }

//    Serial.println("mag x min/max:"); Serial.println(mag_max[0]); Serial.println(mag_min[0]);
//    Serial.println("mag y min/max:"); Serial.println(mag_max[1]); Serial.println(mag_min[1]);
//    Serial.println("mag z min/max:"); Serial.println(mag_max[2]); Serial.println(mag_min[2]);

    // Get hard iron correction
    mag_bias[0] = (mag_max[0] + mag_min[0]) / 2; // get average x mag bias in counts
    mag_bias[1] = (mag_max[1] + mag_min[1]) / 2; // get average y mag bias in counts
    mag_bias[2] = (mag_max[2] + mag_min[2]) / 2; // get average z mag bias in counts

    g_tMagBias.dBiasX = (float) mag_bias[0] * AK8963_getMagRes()
            * g_tMagCalib.dCalibX; // save mag biases in G for main program
    g_tMagBias.dBiasY = (float) mag_bias[1] * AK8963_getMagRes()
            * g_tMagCalib.dCalibY;
    g_tMagBias.dBiasZ = (float) mag_bias[2] * AK8963_getMagRes()
            * g_tMagCalib.dCalibZ;

    // Get soft iron correction estimate
    mag_scale[0] = (mag_max[0] - mag_min[0]) / 2; // get average x axis max chord length in counts
    mag_scale[1] = (mag_max[1] - mag_min[1]) / 2; // get average y axis max chord length in counts
    mag_scale[2] = (mag_max[2] - mag_min[2]) / 2; // get average z axis max chord length in counts

    avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
    avg_rad /= 3.0;

    g_tMagScaleCal.dScaleX = avg_rad / ((float) mag_scale[0]);
    g_tMagScaleCal.dScaleY = avg_rad / ((float) mag_scale[1]);
    g_tMagScaleCal.dScaleZ = avg_rad / ((float) mag_scale[2]);

    return MPU_SUCCESS;
}

MPUStatus_t BMP280_getID(uint8_t *pui8ID)
{
    if (I2C_readRegByte(&g_tI2CObject, BMP280_I2C_ADDR,
    BMP280_ID,
                        pui8ID) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

    return MPU_SUCCESS;
}

MPUStatus_t BMP280_init(void)
{
    uint8_t ui8RegAddr;
    uint8_t aui8Calib[24];

    /* BMP280 Reset */
    if (I2C_writeRegByte(&g_tI2CObject,
    BMP280_I2C_ADDR,
                         BMP280_RESET, 0xB6) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

    delayMillis(100);

    // Configure the BMP280
    // Set T and P oversampling rates and sensor mode
//    writeByte(BMP280_ADDRESS, BMP280_CTRL_MEAS, tBMP280Tosr << 5 | tBMP280Posr << 2 | tBMP280Mode);
    if (I2C_writeRegByte(
            &g_tI2CObject,
            BMP280_I2C_ADDR,
            BMP280_CTRL_MEAS,
            (BMP280_OVER_SAMPLING_T << 5 | BMP280_OVER_SAMPLING_P << 2
                    | BMP280_MODE)) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

    // Set standby time interval in normal mode and bandwidth
//    writeByte(BMP280_ADDRESS, BMP280_CONFIG, tBMP280SBy << 5 | tBMP280IIRFilter << 2);
    if (I2C_writeRegByte(&g_tI2CObject, BMP280_I2C_ADDR,
    BMP280_CONFIG,
                         (BMP280_STANDBY_INTERVAL << 5 | BMP280_FILTER << 2))
            != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

    // Read and store calibration data
//    readBytes(BMP280_ADDRESS, BMP280_CALIB00, 24, &aui8Calib[0]);
    ui8RegAddr = BMP280_CALIB00;
    if (I2C_readRegBytes(&g_tI2CObject, BMP280_I2C_ADDR, &ui8RegAddr, 1,
                         aui8Calib, 24) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

    g_tBMP280Dig.i16T1 =
            (int16_t) (((int16_t) aui8Calib[1] << 8) | aui8Calib[0]);
    g_tBMP280Dig.i16T2 =
            (int16_t) (((int16_t) aui8Calib[3] << 8) | aui8Calib[2]);
    g_tBMP280Dig.i16T3 =
            (int16_t) (((int16_t) aui8Calib[5] << 8) | aui8Calib[4]);
    g_tBMP280Dig.i16P1 =
            (int16_t) (((int16_t) aui8Calib[7] << 8) | aui8Calib[6]);
    g_tBMP280Dig.i16P2 =
            (int16_t) (((int16_t) aui8Calib[9] << 8) | aui8Calib[8]);
    g_tBMP280Dig.i16P3 = (int16_t) (((int16_t) aui8Calib[11] << 8)
            | aui8Calib[10]);
    g_tBMP280Dig.i16P4 = (int16_t) (((int16_t) aui8Calib[13] << 8)
            | aui8Calib[12]);
    g_tBMP280Dig.i16P5 = (int16_t) (((int16_t) aui8Calib[15] << 8)
            | aui8Calib[14]);
    g_tBMP280Dig.i16P6 = (int16_t) (((int16_t) aui8Calib[17] << 8)
            | aui8Calib[16]);
    g_tBMP280Dig.i16P7 = (int16_t) (((int16_t) aui8Calib[19] << 8)
            | aui8Calib[18]);
    g_tBMP280Dig.i16P8 = (int16_t) (((int16_t) aui8Calib[21] << 8)
            | aui8Calib[20]);
    g_tBMP280Dig.i16P9 = (int16_t) (((int16_t) aui8Calib[23] << 8)
            | aui8Calib[22]);

    return MPU_SUCCESS;
}

float MPU9250_getAccelRes(void)
{
    float dRes;

    switch (MPU9250_ACCEL_FULL_SCALE)
    {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
        dRes = 2.0 / 32768.0;
        break;
    case AFS_4G:
        dRes = 4.0 / 32768.0;
        break;
    case AFS_8G:
        dRes = 8.0 / 32768.0;
        break;
    case AFS_16G:
        dRes = 16.0 / 32768.0;
        break;
    }

    return dRes;
}

float MPU9250_getGyroRes(void)
{
    float dRes;

    switch (MPU9250_GYRO_FULL_SCALE)
    {
    // Possible gyro scales (and their register bit settings) are:
    // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
    // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
        dRes = 250.0 / 32768.0;
        break;
    case GFS_500DPS:
        dRes = 500.0 / 32768.0;
        break;
    case GFS_1000DPS:
        dRes = 1000.0 / 32768.0;
        break;
    case GFS_2000DPS:
        dRes = 2000.0 / 32768.0;
        break;
    }

    return dRes;
}

float AK8963_getMagRes(void)
{
    float dRes;

    switch (AK8963_MAG_FULL_SCALE)
    {
    // Possible magnetometer scales (and their register bit settings) are:
    // 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
        dRes = 10.0 * 4912.0 / 8190.0; // Proper scale to return milliGauss
        break;
    case MFS_16BITS:
        dRes = 10.0 * 4912.0 / 32760.0; // Proper scale to return milliGauss
        break;
    }

    return dRes;
}

float MPU9250_getGyroMeasError(void)
{
    return MPU9250_GYRO_MEAS_ERROR;
}

DataStatus_t MPU9250_isDataReady(void)
{
    uint8_t ui8RegData;

//    readByte(MPU9250_ADDRESS, MPU9250_INT_STATUS) & 0x01
    if (I2C_readRegByte(&g_tI2CObject, MPU9250_I2C_ADDR, MPU9250_INT_STATUS,
                        &ui8RegData) != I2C_SUCCESS)
    {
        return DATA_BUSY;
    }

    return (DataStatus_t) (ui8RegData & 0x01U);
}

DataStatus_t AK8963_isDataReady(void)
{
    uint8_t ui8RegData;

    //    readByte(AK8963_ADDRESS, AK8963_ST1) & 0x01
    if (I2C_readRegByte(&g_tI2CObject, AK8963_I2C_ADDR, AK8963_ST1, &ui8RegData)
            != I2C_SUCCESS)
    {
        return DATA_BUSY;
    }

    return (DataStatus_t) (ui8RegData & 0x01U);
}

MPUStatus_t MPU9250_readAccelGyroRawData(MPU9AxisRawData_t *ptMPU9AxisRawData)
{
    uint8_t ui8RegAddr;
    uint8_t aui8RawData[7];

    //    readBytes(MPU9250_ADDRESS, ACCEL_XOUT_H, 6, &aui8RawData[0]); // Read the six raw data registers into data array
    ui8RegAddr = MPU9250_ACCEL_XOUT_H;
    if (I2C_readRegBytes(&g_tI2CObject, MPU9250_I2C_ADDR, &ui8RegAddr, 1,
                         aui8RawData, 6) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

    ptMPU9AxisRawData->i16AccelX = (int16_t) (((int16_t) aui8RawData[0] << 8)
            | aui8RawData[1]); // Turn the MSB and LSB into a signed 16-bit value
    ptMPU9AxisRawData->i16AccelY = (int16_t) (((int16_t) aui8RawData[2] << 8)
            | aui8RawData[3]);
    ptMPU9AxisRawData->i16AccelZ = (int16_t) (((int16_t) aui8RawData[4] << 8)
            | aui8RawData[5]);

    //    readBytes(MPU9250_ADDRESS, GYRO_XOUT_H, 6, &aui8RawData[0]); // Read the six raw data registers sequentially into data array
    ui8RegAddr = MPU9250_GYRO_XOUT_H;
    if (I2C_readRegBytes(&g_tI2CObject, MPU9250_I2C_ADDR, &ui8RegAddr, 1,
                         aui8RawData, 6) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

    ptMPU9AxisRawData->i16GyroX = (int16_t) (((int16_t) aui8RawData[0] << 8)
            | aui8RawData[1]); // Turn the MSB and LSB into a signed 16-bit value
    ptMPU9AxisRawData->i16GyroY = (int16_t) (((int16_t) aui8RawData[2] << 8)
            | aui8RawData[3]);
    ptMPU9AxisRawData->i16GyroZ = (int16_t) (((int16_t) aui8RawData[4] << 8)
            | aui8RawData[5]);

    return MPU_SUCCESS;
}

MPUStatus_t AK8963_readMagRawData(MPU9AxisRawData_t *ptMPU9AxisRawData)
{
    uint8_t ui8RegAddr;
    uint8_t aui8RawData[7]; // x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition

    //        readBytes(AK8963_ADDRESS, AK8963_XOUT_L, 7, &aui8RawData[0]); // Read the six raw data and ST2 registers sequentially into data array
    ui8RegAddr = AK8963_XOUT_L;
    if (I2C_readRegBytes(&g_tI2CObject, AK8963_I2C_ADDR, &ui8RegAddr, 1,
                         &aui8RawData[0], 7) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

    if (!(aui8RawData[6] & 0x08)) // Check if magnetic sensor overflow set, if not then report data
    {
        ptMPU9AxisRawData->i16MagX = (int16_t) (((int16_t) aui8RawData[1] << 8)
                | aui8RawData[0]); // Turn the MSB and LSB into a signed 16-bit value
        ptMPU9AxisRawData->i16MagY = (int16_t) (((int16_t) aui8RawData[3] << 8)
                | aui8RawData[2]); // Data stored as little Endian
        ptMPU9AxisRawData->i16MagZ = (int16_t) (((int16_t) aui8RawData[5] << 8)
                | aui8RawData[4]);
    }

    return MPU_SUCCESS;
}

void MPU9250_get9AxisInUnits(MPU9AxisInUnits_t *ptMPU9AxisInUnits,
                             MPU9AxisRawData_t *ptMPU9AxisRawData)
{
    float dAccelRes, dGyroRes, dMagRes;

    // Now we'll calculate the accleration value into actual g's
    //    ax = (float) MPU9250Data[0] * aRes - accelBias[0]; // get actual g value, this depends on scale being set
    //    ay = (float) MPU9250Data[1] * aRes - accelBias[1];
    //    az = (float) MPU9250Data[2] * aRes - accelBias[2];

    dAccelRes = MPU9250_getAccelRes();

    ptMPU9AxisInUnits->dAccelX = (float) ptMPU9AxisRawData->i16AccelX
            * dAccelRes - g_tAccelBias.dBiasX;
    ptMPU9AxisInUnits->dAccelY = (float) ptMPU9AxisRawData->i16AccelY
            * dAccelRes - g_tAccelBias.dBiasY;
    ptMPU9AxisInUnits->dAccelZ = (float) ptMPU9AxisRawData->i16AccelZ
            * dAccelRes - g_tAccelBias.dBiasZ;

    // Calculate the gyro value into actual degrees per second
    //    gx = (float) MPU9250Data[4] * gRes; // get actual gyro value, this depends on scale being set
    //    gy = (float) MPU9250Data[5] * gRes;
    //    gz = (float) MPU9250Data[6] * gRes;

    dGyroRes = MPU9250_getGyroRes();

    ptMPU9AxisInUnits->dGyroX = (float) ptMPU9AxisRawData->i16GyroX * dGyroRes
            - g_tGyroBias.dBiasX; // get actual gyro value, this depends on scale being set
    ptMPU9AxisInUnits->dGyroY = (float) ptMPU9AxisRawData->i16GyroY * dGyroRes
            - g_tGyroBias.dBiasY;
    ptMPU9AxisInUnits->dGyroZ = (float) ptMPU9AxisRawData->i16GyroZ * dGyroRes
            - g_tGyroBias.dBiasZ;

    ptMPU9AxisInUnits->dGyroX *= (PI / 180.0f); /* convert to rad/sec */
    ptMPU9AxisInUnits->dGyroY *= (PI / 180.0f);
    ptMPU9AxisInUnits->dGyroZ *= (PI / 180.0f);

    //    mx = (float) magCount[0] * mRes * magCalibration[0] - magBias[0]; // get actual magnetometer value, this depends on scale being set
    //    my = (float) magCount[1] * mRes * magCalibration[1] - magBias[1];
    //    mz = (float) magCount[2] * mRes * magCalibration[2] - magBias[2];
    //    mx *= magScale[0];
    //    my *= magScale[1];
    //    mz *= magScale[2];

    dMagRes = AK8963_getMagRes();

    ptMPU9AxisInUnits->dMagX = (float) ptMPU9AxisRawData->i16MagX * dMagRes
            * g_tMagCalib.dCalibX - g_tMagBias.dBiasX; // get actual magnetometer value, this depends on scale being set
    ptMPU9AxisInUnits->dMagY = (float) ptMPU9AxisRawData->i16MagY * dMagRes
            * g_tMagCalib.dCalibY - g_tMagBias.dBiasY;
    ptMPU9AxisInUnits->dMagZ = (float) ptMPU9AxisRawData->i16MagZ * dMagRes
            * g_tMagCalib.dCalibZ - g_tMagBias.dBiasZ;
    ptMPU9AxisInUnits->dMagX *= g_tMagScaleCal.dScaleX;
    ptMPU9AxisInUnits->dMagY *= g_tMagScaleCal.dScaleY;
    ptMPU9AxisInUnits->dMagZ *= g_tMagScaleCal.dScaleZ;
}

void MPU9250_get9AxisInUnitsModified(MPU9AxisInUnits_t *ptMPU9AxisInUnits2,
                                     MPU9AxisInUnits_t *ptMPU9AxisInUnits1)
{
    ptMPU9AxisInUnits2->dAccelX = -ptMPU9AxisInUnits1->dAccelX;
    ptMPU9AxisInUnits2->dAccelY = ptMPU9AxisInUnits1->dAccelY;
    ptMPU9AxisInUnits2->dAccelZ = ptMPU9AxisInUnits1->dAccelZ;
    ptMPU9AxisInUnits2->dGyroX = ptMPU9AxisInUnits1->dGyroX;
    ptMPU9AxisInUnits2->dGyroY = -ptMPU9AxisInUnits1->dGyroY;
    ptMPU9AxisInUnits2->dGyroZ = -ptMPU9AxisInUnits1->dGyroZ;
    ptMPU9AxisInUnits2->dMagX = ptMPU9AxisInUnits1->dMagY;
    ptMPU9AxisInUnits2->dMagY = -ptMPU9AxisInUnits1->dMagX;
    ptMPU9AxisInUnits2->dMagZ = ptMPU9AxisInUnits1->dMagZ;
}

MPUStatus_t BMP280_readPressRawData(BMP280Press_t *ptPressRawData)
{
    uint8_t ui8RegAddr;
    uint8_t aui8RawData[3];  // 20-bit pressure register data stored here

//    readBytes(BMP280_ADDRESS, BMP280_PRESS_MSB, 3, &aui8RawData[0]);
    ui8RegAddr = BMP280_PRESS_MSB;
    if (I2C_readRegBytes(&g_tI2CObject, BMP280_I2C_ADDR, &ui8RegAddr, 1,
                         aui8RawData, 3) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

    ptPressRawData->i32Pressure = (int32_t) (((int32_t) aui8RawData[0] << 16
            | (int32_t) aui8RawData[1] << 8 | aui8RawData[2]) >> 4);

    return MPU_SUCCESS;
}

void BMP280_getPressInMBar(BMP280PressInMBar_t *ptBMP280PressInMBar,
                           BMP280Press_t *ptPressRawData)
{
    uint64_t var1, var2, p;

    var1 = ((uint64_t) g_i32BMP280Tfine) - 128000;
    var2 = var1 * var1 * (uint64_t) g_tBMP280Dig.i16P6;
    var2 = var2 + ((var1 * (uint64_t) g_tBMP280Dig.i16P5) << 17);
    var2 = var2 + (((uint64_t) g_tBMP280Dig.i16P4) << 35);
    var1 = ((var1 * var1 * (uint64_t) g_tBMP280Dig.i16P3) >> 8)
            + ((var1 * (uint64_t) g_tBMP280Dig.i16P2) << 12);
    var1 = (((((uint64_t) 1) << 47) + var1)) * ((uint64_t) g_tBMP280Dig.i16P1)
            >> 33;
    if (var1 == 0)
    {
        /* avoid exception caused by division by zero */
        ptBMP280PressInMBar->dPressure = 0;
        return;
    }
    p = 1048576 - ptPressRawData->i32Pressure;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((uint64_t) g_tBMP280Dig.i16P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((uint64_t) g_tBMP280Dig.i16P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((uint64_t) g_tBMP280Dig.i16P7) << 4);
    ptBMP280PressInMBar->dPressure = p / 25600.0;
}

void BMP280_getAltitudeInM(BMP280AltitudeInM_t *ptBMP280AltitudeInM,
                           BMP280PressInMBar_t *ptBMP280PressInMBar)
{
    ptBMP280AltitudeInM->dAltitude = 145366.45f
            * (1.0f
                    - powf((ptBMP280PressInMBar->dPressure / 1013.25f),
                           0.190284f)); /* Altitude in feet */
    ptBMP280AltitudeInM->dAltitude /= 3.28284; /* Altitude in meter */
}

MPUStatus_t BMP280_readTempRawData(BMP280Temp_t *ptTempRawData)
{
    uint8_t ui8RegAddr;
    uint8_t aui8RawData[3];  // 20-bit pressure register data stored here

//    readBytes(BMP280_ADDRESS, BMP280_TEMP_MSB, 3, &aui8RawData[0]);
    ui8RegAddr = BMP280_TEMP_MSB;
    if (I2C_readRegBytes(&g_tI2CObject, BMP280_I2C_ADDR, &ui8RegAddr, 1,
                         aui8RawData, 3) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

    ptTempRawData->i32Temp = (int32_t) (((int32_t) aui8RawData[0] << 16
            | (int32_t) aui8RawData[1] << 8 | aui8RawData[2]) >> 4);

    return MPU_SUCCESS;
}

BMP280TempInC_t BMP280_getTempInC(BMP280Temp_t *ptTempRawData)
{
    int32_t var1, var2, T;
    BMP280TempInC_t BMP280TempInC;

    var1 = ((((ptTempRawData->i32Temp >> 3)
            - ((int32_t) g_tBMP280Dig.i16T1 << 1)))
            * ((int32_t) g_tBMP280Dig.i16T2)) >> 11;
    var2 = (((((ptTempRawData->i32Temp >> 4) - ((int32_t) g_tBMP280Dig.i16T1))
            * ((ptTempRawData->i32Temp >> 4) - ((int32_t) g_tBMP280Dig.i16T1)))
            >> 12) * ((int32_t) g_tBMP280Dig.i16T3)) >> 14;
    g_i32BMP280Tfine = var1 + var2;
    T = (g_i32BMP280Tfine * 5 + 128) >> 8;
    BMP280TempInC.dTemp = T / 100.0;

    return BMP280TempInC;
}

MPUStatus_t MPU9250_readTempRawData(MPU9250Temp_t *ptMPU9250Temp)
{
    uint8_t ui8RegAddr;
    uint8_t aui8RawData[2];  // x/y/z gyro register data stored here

//    readBytes(MPU9250_ADDRESS, TEMP_OUT_H, 2, &aui8RawData[0]); // Read the two raw data registers sequentially into data array
    ui8RegAddr = MPU9250_TEMP_OUT_H;
    if (I2C_readRegBytes(&g_tI2CObject, MPU9250_I2C_ADDR, &ui8RegAddr, 1,
                         aui8RawData, 2) != I2C_SUCCESS)
    {
        return MPU_ERROR;
    }

    ptMPU9250Temp->i16Temp = (int16_t) (((int16_t) aui8RawData[0] << 8)
            | aui8RawData[1]); // Turn the MSB and LSB into a 16-bit value

    return MPU_SUCCESS;
}

void MPU9250_getCalibParam(MPUCalibParam_t *ptMPUCalibParam)
{
    ptMPUCalibParam->tAccelBias = g_tAccelBias;
    ptMPUCalibParam->tGyroBias = g_tGyroBias;
    ptMPUCalibParam->tMagBias = g_tMagBias;
    ptMPUCalibParam->tMagCalib = g_tMagCalib;
    ptMPUCalibParam->tMagScaleCal = g_tMagScaleCal;
}

