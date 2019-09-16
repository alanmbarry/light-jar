/*
 * lib_mpu6050_stm32_hal.h
 *
 *  Created on: 10 Sep 2019
 *      Author: User
 */

#include <stdbool.h>
#include "lib_mpu6050_stm32_hal.h"
#include "stm32f1xx_hal_def.h"


HAL_StatusTypeDef mpu6050_writereg_simple(I2C_HandleTypeDef *hi2c,
		                           bool a0,
								   uint8_t regaddress,
								   uint8_t writeval)
{

}

HAL_StatusTypeDef mpu6050_readreg_simple(I2C_HandleTypeDef *hi2c,
		                           bool a0,
								   uint8_t regaddress,
								   uint8_t *readval)
{
   HAL_StatusTypeDef ReturnStatus;
   uint16_t DeviceAddress;
   if(a0 == 1)
   {
	   DeviceAddress = I2C_DEV_ADDR_MPU6050 | (0x02);
   }
   else
   {
	   DeviceAddress = I2C_DEV_ADDR_MPU6050;
   }

   ReturnStatus = HAL_I2C_Master_Transmit(hi2c, DeviceAddress, &regaddress, 1, 20);
   if(ReturnStatus == HAL_OK)
   {
	   ReturnStatus = HAL_I2C_Master_Receive(hi2c, DeviceAddress, readval, 1, 20);
   }

   return ReturnStatus;
}
