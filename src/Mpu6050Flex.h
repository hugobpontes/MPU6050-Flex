/**
 ********************************************************************************
 * @file    Mpu6050Flex.h
 * @author  Hugo Pontes
 * @date    12/04/2023
 * @brief   Flexible mpu6050 that can be used on any microcontroller
 ********************************************************************************
 */

#ifndef MPU6050FLEX_H_
#define MPU6050FLEX_H_

#include <stdint.h>

/*I2C Address of the MPU6050*/
#define MPU6050_I2C_ADDRESS 0x68


/**
 * @brief Enum containing possible return values for IO functions
 */
typedef enum
{
    IO_SUCCESS,
	IO_FAILURE,
} IOStatus_t;
/**
 * @brief Enum containing all mpu6050 register addresses that the Mpu6050Flex library interacts with
 */
typedef enum
{
    REG_WHO_AM_I = 0x75,
} Mpu6050Register_t;

/**
 * @brief A type representing the signature that IO functions should have.
 */
typedef IOStatus_t (*IOFunc_t)(uint8_t,uint32_t,uint8_t*);

void Mpu6050Flex_SetIOWrite(IOFunc_t pWriteFunc);
void Mpu6050Flex_SetIORead(IOFunc_t pReadFunc);
IOFunc_t Mpu6050Flex_GetIOWrite();
IOFunc_t Mpu6050Flex_GetIORead();
uint8_t Mpu6050Flex_WhoAmI();

#endif /* MPU6050FLEX_H_ */

