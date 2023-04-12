/**
 ********************************************************************************
 * @file    Mpu6050.h
 * @author  Hugo Pontes
 * @date    12/04/2023
 * @brief   Flexible mpu6050 that can be used on any microcontroller
 ********************************************************************************
 */

#ifndef MPU6050_H_
#define MPU6050_H_

#include <stdint.h>

/**
 * @brief Enum containing possible return values for IO functions
 */
typedef enum
{
    IO_SUCCESS,
	IO_FAILURE,
} IOState_t;

/**
 * @brief A type representing the signature that IO functions should have.
 */
typedef IOState_t (*IOFunc_t)(uint8_t,uint32_t,uint8_t*);

void Mpu6050_SetIOWrite(IOFunc_t pWriteFunc);
void Mpu6050_SetIORead(IOFunc_t pReadFunc);
IOFunc_t Mpu6050_GetIOWrite();
IOFunc_t Mpu6050_GetIORead();

#endif /* MPU6050_H_ */

