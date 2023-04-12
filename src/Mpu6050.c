/**
 ********************************************************************************
 * @file    Mpu6050.c
 * @author  Hugo Pontes
 * @date    12/04/2023
 * @brief   Flexible mpu6050 that can be used on any microcontroller
 ********************************************************************************
 */

#include "Mpu6050.h"
#include <stdint.h>

/**
 * @brief Type representing a structure containing all configurable parameters in the Mpu6050 library.
 */
typedef struct Mpu6050ConfigStruct
{
	IOFunc_t pIOWrite;
	IOFunc_t pIORead;
} Mpu6050Config_t;


static Mpu6050Config_t Mpu6050Config =
{
	.pIORead = 0,
	.pIOWrite = 0
};

/**
 * @brief Injects the IO Write function that this library uses
 *
 * @param pWriteFunc Function Pointer that points to the IO Write function to use
 */
void Mpu6050_SetIOWrite(IOFunc_t pWriteFunc)
{
	Mpu6050Config.pIOWrite = pWriteFunc;
}
/**
 * @brief Injects the IO Read function that this library uses
 *
 * @param pWriteFunc Function Pointer that points to the IO Read function to use
 */
void Mpu6050_SetIORead(IOFunc_t pReadFunc)
{
	Mpu6050Config.pIORead = pReadFunc;
}
/**
 * @brief Gets a pointer to the currently set IO write function pointer
 *
 * @return Pointer to the currently set IO write function pointer
 */
IOFunc_t Mpu6050_GetIOWrite()
{
	return Mpu6050Config.pIOWrite;
}
/**
 * @brief Gets a pointer to the currently set IO read function pointer
 *
 * @return Pointer to the currently set IO read function pointer
 */
IOFunc_t Mpu6050_GetIORead()
{
	return Mpu6050Config.pIORead;
}


