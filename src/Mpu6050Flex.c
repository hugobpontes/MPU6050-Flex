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
#include <stdbool.h>

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

static MPU6050Flex_Status_t Mpu6050Flex_WriteRegisterSegment(uint8_t Register,uint8_t SegmentMask, uint8_t Value);
static bool Mpu6050Flex_OptionFitsInMask(uint8_t Option, uint8_t Mask);

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
 * @return I2C Address of the MPU6050 or 0xFF if it wasnt possible to obtain a valid address
 */
uint8_t Mpu6050Flex_WhoAmI()
{
	uint8_t Response;

	if (Mpu6050Config.pIORead(REG_WHO_AM_I,1,&Response) != IO_SUCCESS)
	{
		Response = 0xFF;
	}

	return Response;
}

MPU6050Flex_Status_t Mpu6050Flex_ConfigSampleRateDivider(uint8_t Division)
{
	MPU6050Flex_Status_t Status = MPU6050FLEX_SUCCESS;

	if (Mpu6050Config.pIOWrite(REG_SAMPLE_RATE_DIVIDER,1,&Division) != IO_SUCCESS)
	{
		Status = MPU6050FLEX_FAILURE;
	}

	return Status;
}

MPU6050Flex_Status_t Mpu6050Flex_ConfigDigitalLowPassFilter(uint8_t ConfigOption)
{
	MPU6050Flex_Status_t Status = MPU6050FLEX_SUCCESS;

	if (Mpu6050Flex_OptionFitsInMask(ConfigOption,MPU6050FLEX_DLPF_CFG_MSK))
	{
		Status = Mpu6050Flex_WriteRegisterSegment(REG_CONFIGURATION,MPU6050FLEX_DLPF_CFG_MSK,ConfigOption);
	}
	else
	{
		Status = MPU6050FLEX_FAILURE;
	}

	return Status;
}

MPU6050Flex_Status_t Mpu6050Flex_ConfigGyroFullScaleRange(uint8_t ConfigOption)
{
	MPU6050Flex_Status_t Status = MPU6050FLEX_SUCCESS;

	if (Mpu6050Flex_OptionFitsInMask(ConfigOption,MPU6050FLEX_GYRO_FS_SEL_MSK))
	{
		Status = Mpu6050Flex_WriteRegisterSegment(REG_GYRO_CONFIG,MPU6050FLEX_GYRO_FS_SEL_MSK,ConfigOption);
	}
	else
	{
		Status = MPU6050FLEX_FAILURE;
	}

	return Status;
}

MPU6050Flex_Status_t Mpu6050Flex_ConfigAccFullScaleRange(uint8_t ConfigOption)
{
	MPU6050Flex_Status_t Status = MPU6050FLEX_SUCCESS;

	if (Mpu6050Flex_OptionFitsInMask(ConfigOption,MPU6050FLEX_ACC_FS_SEL_MSK))
	{
		Status = Mpu6050Flex_WriteRegisterSegment(REG_ACCEL_CONFIG,MPU6050FLEX_ACC_FS_SEL_MSK,ConfigOption);
	}
	else
	{
		Status = MPU6050FLEX_FAILURE;
	}

	return Status;
}


MPU6050Flex_Status_t Mpu6050Flex_WakeUp()
{
	uint8_t Zero = 0;
	MPU6050Flex_Status_t Status = MPU6050FLEX_SUCCESS;

	if (Mpu6050Config.pIOWrite(REG_SAMPLE_RATE_DIVIDER,1,&Zero) != IO_SUCCESS)
	{
		Status = MPU6050FLEX_FAILURE;
	}
	return Status;
}

static MPU6050Flex_Status_t Mpu6050Flex_WriteRegisterSegment(uint8_t Register,uint8_t SegmentMask, uint8_t Value)
{
	uint8_t CurrentRegValue;
	uint8_t WriteValue;

	MPU6050Flex_Status_t Status = MPU6050FLEX_SUCCESS;

	if (Mpu6050Config.pIORead(Register,1,&CurrentRegValue) == IO_SUCCESS)
			{
				WriteValue = (CurrentRegValue & (~SegmentMask)) | Value;
				if (Mpu6050Config.pIOWrite(Register,1,&WriteValue) != IO_SUCCESS)
				{
					Status = MPU6050FLEX_FAILURE;
				}
			}
			else
			{
				Status = MPU6050FLEX_FAILURE;
			}

	return Status;
}

static bool Mpu6050Flex_OptionFitsInMask(uint8_t Option, uint8_t Mask)
{
	return !(Option & (~Mask));
}
