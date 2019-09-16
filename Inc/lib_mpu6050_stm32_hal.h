// include file for functions to abstract away some of the I2C mechanics of interfacing to
// the MPU6050 3-axis accelerometer
// Uses the STM32 I2C HAL

#include <stdbool.h>
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_def.h"

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __LIB_MPU6060_STM32_HAL_H
#define __LIB_MPU6060_STM32_HAL_H

//HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint8_t *pData, uint16_t Size, uint32_t Timeout)

typedef struct {
	int16_t x;
	int16_t y;
	int16_t z;
} accel_vect_t;

//const uint16_t I2C_DEV_ADDR_MPU6050 = 0x00D0;
#define I2C_DEV_ADDR_MPU6050 0x00D0

// https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/MPU6050.h

HAL_StatusTypeDef mpu6050_writereg_simple(I2C_HandleTypeDef *hi2c,
		                           bool a0,
								   uint8_t regaddress,
								   uint8_t writeval);

HAL_StatusTypeDef mpu6050_readreg_simple(I2C_HandleTypeDef *hi2c,
		                           bool a0,
								   uint8_t regaddress,
								   uint8_t *readval);

HAL_StatusTypeDef mpu6050_get_accel_vect(I2C_HandleTypeDef *hi2c,
		                           bool a0,
								   accel_vect_t *accel_vect);

#endif
