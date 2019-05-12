/*
 * MPU_hw.h
 *
 *  Created on: Mar 17, 2019
 *      Author: Administrator
 */

#ifndef MPU_CONFIG_H_
#define MPU_CONFIG_H_

#define MPU9250_ACCEL_FULL_SCALE    AFS_2G  /* can be one of enum values of [MPU9250Ascale_t] in file [MPU.defs.h] */
#define MPU9250_GYRO_FULL_SCALE     GFS_250DPS /* can be one of enum values of [MPU9250Gscale_t] in file [MPU.defs.h] */
#define MPU9250_GYRO_MEAS_ERROR     (PI * (4.0f / 180.0f)); // gyroscope measurement error in rad/s (start at 40 deg/s)

#define AK8963_MAG_FULL_SCALE       MFS_16BITS /* can be one of enum values of [AK8963Mscale_t] in file [MPU.defs.h] */
#define AK8963_MAG_MODE             MDR_100HZ /* can be one of enum values of [AK8963Mmode_t] in file [MPU.defs.h] */

#define BMP280_MODE                 NORMAL /* can be one of enum values of [BMP280Mode_t] in file [MPU.defs.h] */
#define BMP280_FILTER               BW0_042ODR /* can be one of enum values of [BMP280IIRFilter_t] in file [MPU.defs.h] */
#define BMP280_STANDBY_INTERVAL     T_62_5ms /* can be one of enum values of [BMP280SBy_t] in file [MPU.defs.h] */
#define BMP280_OVER_SAMPLING_T      T_OSR_02 /* can be one of enum values of [BMP280Tosr_t] in file [MPU.defs.h] */
#define BMP280_OVER_SAMPLING_P      P_OSR_16 /* can be one of enum values of [BMP280Posr_t] in file [MPU.defs.h] */

#define MPU_I2C_ADDR_BASE           I2C0_BASE
#define MPU_I2C_CLK_BASE            SYSCTL_PERIPH_I2C0
#define MPU_I2C_PORT_ADDR           GPIO_PORTB_BASE
#define MPU_I2C_PORT_CLK            SYSCTL_PERIPH_GPIOB
#define MPU_I2C_SCL_PIN             GPIO_PIN_2
#define MPU_I2C_SDA_PIN             GPIO_PIN_3
#define MPU_I2C_SCL_PCTL            GPIO_PB2_I2C0SCL
#define MPU_I2C_SDA_PCTL            GPIO_PB3_I2C0SDA

#endif /* MPU_CONFIG_H_ */

