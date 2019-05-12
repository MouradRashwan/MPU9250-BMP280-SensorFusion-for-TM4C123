/*
 * MPU.h
 *
 *  Created on: Mar 17, 2019
 *      Author: Administrator
 */

#ifndef MPU_H_
#define MPU_H_

uint8_t MPU_I2C_initScan(void);

MPUStatus_t MPU9250_getID(uint8_t *pui8ID);

MPUStatus_t MPU9250_SelfTest(float adDeviations[6]);

MPUStatus_t MPU9250_calibrateAccelGyro(void);

MPUStatus_t MPU9250_init(void);

MPUStatus_t AK8963_getID(uint8_t *pui8ID);

MPUStatus_t AK8963_init(void);

MPUStatus_t AK8963_calibrateMag(void);

MPUStatus_t BMP280_getID(uint8_t *pui8ID);

MPUStatus_t BMP280_init(void);

float MPU9250_getAccelRes(void);

float MPU9250_getGyroRes(void);

float AK8963_getMagRes(void);

float MPU9250_getGyroMeasError(void);

DataStatus_t MPU9250_isDataReady(void);

DataStatus_t AK8963_isDataReady(void);

MPUStatus_t MPU9250_readAccelGyroRawData(MPU9AxisRawData_t *ptMPU9AxisRawData);

MPUStatus_t AK8963_readMagRawData(MPU9AxisRawData_t *ptMPU9AxisRawData);

void MPU9250_get9AxisInUnits(MPU9AxisInUnits_t *ptMPU9AxisInUnits,
                             MPU9AxisRawData_t *ptMPU9AxisRawData);

void MPU9250_get9AxisInUnitsModified(MPU9AxisInUnits_t *ptMPU9AxisInUnits2,
                                     MPU9AxisInUnits_t *ptMPU9AxisInUnits1);

MPUStatus_t BMP280_readPressRawData(BMP280Press_t *ptPressRawData);

void BMP280_getPressInMBar(BMP280PressInMBar_t *ptBMP280PressInMBar,
                           BMP280Press_t *ptPressRawData);

void BMP280_getAltitudeInM(BMP280AltitudeInM_t *ptBMP280AltitudeInM,
                           BMP280PressInMBar_t *ptBMP280PressInMBar);

MPUStatus_t BMP280_readTempRawData(BMP280Temp_t *ptTempRawData);

BMP280TempInC_t BMP280_getTempInC(BMP280Temp_t *ptTempRawData);

MPUStatus_t MPU9250_readTempRawData(MPU9250Temp_t *ptMPU9250Temp);

void MPU9250_getCalibParam(MPUCalibParam_t *ptMPUCalibParam);

#endif /* MPU_H_ */
