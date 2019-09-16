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
	   HAL_StatusTypeDef ReturnStatus;
	   uint16_t DeviceAddress;
	   uint8_t writebuf[2];

	   if(a0 == 1)
	   {
		   DeviceAddress = I2C_DEV_ADDR_MPU6050 | (0x02);
	   }
	   else
	   {
		   DeviceAddress = I2C_DEV_ADDR_MPU6050;
	   }

	   writebuf[0] = regaddress;
	   writebuf[1] = writeval;

	   ReturnStatus = HAL_I2C_Master_Transmit(hi2c, DeviceAddress, writebuf, 2, 20);

	   return ReturnStatus;

}

HAL_StatusTypeDef mpu6050_readreg_simple(I2C_HandleTypeDef *hi2c,
		                           bool a0,
								   uint8_t regaddress,
								   uint8_t *readval)
{
   HAL_StatusTypeDef ReturnStatus;
   uint16_t DeviceAddress;
   if(a0 == true)
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


HAL_StatusTypeDef mpu6050_get_accel_vect(I2C_HandleTypeDef *hi2c,
		                           bool a0,
								   accel_vect_t *accel_vect)
{
   HAL_StatusTypeDef ReturnStatus;
   uint16_t DeviceAddress;
   uint8_t accel_dat_readbuf[6];
   uint8_t regaddress = 0x3B;
   if(a0 == true)
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
	   ReturnStatus = HAL_I2C_Master_Receive(hi2c, DeviceAddress, accel_dat_readbuf, 6, 20);
   }
   if(ReturnStatus == HAL_OK)
   {
	   *((uint8_t*)accel_vect + 0) = accel_dat_readbuf[1];
	   *((uint8_t*)accel_vect + 1) = accel_dat_readbuf[0];
	   *((uint8_t*)accel_vect + 2) = accel_dat_readbuf[3];
	   *((uint8_t*)accel_vect + 3) = accel_dat_readbuf[2];
	   *((uint8_t*)accel_vect + 4) = accel_dat_readbuf[5];
	   *((uint8_t*)accel_vect + 5) = accel_dat_readbuf[4];
   }
   return ReturnStatus;

}
