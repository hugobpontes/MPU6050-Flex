/**
 ********************************************************************************
 * @file    Mpu6050Flex.c
 * @author  Hugo Pontes
 * @date    12/04/2023
 * @brief   Flexible mpu6050 that can be used on any microcontroller
 ********************************************************************************
 */

#include <Mpu6050Flex.h>
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
	.pIOWrite = 0,
};

/**
 * @brief Injects the IO Write function that this library uses
 *
 * @param pWriteFunc Function Pointer that points to the IO Write function to use
 */
void Mpu6050Flex_SetIOWrite(IOFunc_t pWriteFunc)
{
	Mpu6050Config.pIOWrite = pWriteFunc;
}
/**
 * @brief Injects the IO Read function that this library uses
 *
 * @param pWriteFunc Function Pointer that points to the IO Read function to use
 */
void Mpu6050Flex_SetIORead(IOFunc_t pReadFunc)
{
	Mpu6050Config.pIORead = pReadFunc;
}
/**
 * @brief Gets a pointer to the currently set IO write function pointer
 *
 * @return Pointer to the currently set IO write function pointer
 */
IOFunc_t Mpu6050Flex_GetIOWrite()
{
	return Mpu6050Config.pIOWrite;
}
/**
 * @brief Gets a pointer to the currently set IO read function pointer
 *
 * @return Pointer to the currently set IO read function pointer
 */
IOFunc_t Mpu6050Flex_GetIORead()
{
	return Mpu6050Config.pIORead;
}
/**
 * @brief Requests the i2c address of the mpu6050 device via i2c, used to verify working i2c interface
 *
 * @return I2C Address of the MPU6050
 */
uint8_t Mpu6050Flex_WhoAmI()
{
	uint8_t Response;
	Mpu6050Config.pIORead(REG_WHO_AM_I,1,&Response);

	return Response;
}

