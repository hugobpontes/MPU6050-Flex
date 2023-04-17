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

static MPU6050Flex_Status_t Mpu6050Flex_ReplaceRegisterSegment(uint8_t Register,uint8_t SegmentMask, uint8_t Value);
static MPU6050Flex_Status_t Mpu6050Flex_WriteFullRegister(uint8_t Register, uint8_t Value);
static bool Mpu6050Flex_ValueFitsInMask(uint8_t Option, uint8_t Mask);
static MPU6050Flex_Status_t Mpu6050Flex_UpdateParameter(uint8_t ParameterValue, uint8_t ParameterMask, uint8_t RegisterAddress);

/**
 * @brief Injects the IO Write function that this library uses
 *
 * @param pWriteFunc: Function Pointer that points to the IO Write function to use
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
/**
 * @brief Updates the SMPLRT_DIV parameter of the SMPRT_DIV register, configuring the sample rate divider
 *
 * @param Division: Value to set in the SMPLRT_DIV parameter of the SMPRT_DIV register, representing by how much the gyro rate must be
 * divided to obtain the sample rate (div = 1+SMPLRT_DIV)
 *
 * @return Status code to access operation success
 */
MPU6050Flex_Status_t Mpu6050Flex_ConfigSampleRateDivider(uint8_t Division)
{
	MPU6050Flex_Status_t Status;

	Status = Mpu6050Flex_UpdateParameter(Division,MPU6050FLEX_SMPLRT_DIV_MSK,REG_SMPRT_DIV);

	return Status;
}
/**
 * @brief Updates the MPU6050FLEX_DLPF_CFG parameter of the CONFIG register, configuring the digital filter as specified
 * in the register map document (section 4.3)
 *
 * @param ConfigOption: Configuration option as specified in the appropriate register map section,
 * if value isn't part of the MPU6050Flex_DLPF_Options_t enum, this function will return an error
 *
 * @return Status code to access operation success
 */
MPU6050Flex_Status_t Mpu6050Flex_ConfigDigitalLowPassFilter(MPU6050Flex_DLPF_Options_t ConfigOption)
{
	MPU6050Flex_Status_t Status;

	Status = Mpu6050Flex_UpdateParameter(ConfigOption,MPU6050FLEX_DLPF_CFG_MSK,REG_CONFIG);

	return Status;
}
/**
 * @brief Updates the FS_SEL parameter of the GYRO_CONFIG register, configuring the gyro full scale range
 *
 * @param ConfigOption: Configuration option as specified in the appropriate register map section,
 * if value isn't part of the MPU6050Flex_GYRO_FS_SEL_Options_t enum, this function will return an error:
 *  MPU6050FLEX_GYRO_FS_SEL_25		: Full Scale Range +- 25 deg
 *	MPU6050FLEX_GYRO_FS_SEL_500 	: Full Scale Range +- 500 deg
 *	MPU6050FLEX_GYRO_FS_SEL_1000	: Full Scale Range +- 1000 deg
 *	MPU6050FLEX_GYRO_FS_SEL_2000	: Full Scale Range +- 2000 deg
 *
 * @return Status code to access operation success
 */
MPU6050Flex_Status_t Mpu6050Flex_ConfigGyroFullScaleRange(MPU6050Flex_GYRO_FS_SEL_Options_t ConfigOption)
{
	MPU6050Flex_Status_t Status;

	Status = Mpu6050Flex_UpdateParameter(ConfigOption,MPU6050FLEX_FS_SEL_MSK,REG_GYRO_CONFIG);

	return Status;
}
/**
 * @brief Updates the AFS_SEL parameter of the ACCEL_CONFIG register, configuring the accelerometer full scale range
 *
 * @param ConfigOption: Configuration option as specified in the appropriate register map section,
 * if value isn't part of the MPU6050Flex_ACC_FS_SEL_Options_t enum, this function will return an error:
 *  MPU6050FLEX_ACC_FS_SEL_2	: Full Scale Range +- 2g
 *	MPU6050FLEX_ACC_FS_SEL_4	: Full Scale Range +- 4g
 *	MPU6050FLEX_ACC_FS_SEL_8	: Full Scale Range +- 8g
 *	MPU6050FLEX_ACC_FS_SEL_16	: Full Scale Range +- 16g
 *
 * @return Status code to access operation success
 */
MPU6050Flex_Status_t Mpu6050Flex_ConfigAccFullScaleRange(MPU6050Flex_ACC_FS_SEL_Options_t ConfigOption)
{
	MPU6050Flex_Status_t Status;

	Status = Mpu6050Flex_UpdateParameter(ConfigOption,MPU6050FLEX_AFS_SEL_MSK,REG_ACCEL_CONFIG);

	return Status;
}
/**
 * @brief Reads a given register and changes a segment of it based on a parameter
 * value and mask that fit within that register.
 *
 * @param RegisterAddress: Register address of where the segment to be replaced is
 * @param SegmentMask: Mask to indicate which segment of the register is to be replaced
 * @param Value: Value to write in the replaced segment
 *
 * @return Status code to access operation success
 */
static MPU6050Flex_Status_t Mpu6050Flex_ReplaceRegisterSegment(uint8_t RegisterAddress,uint8_t SegmentMask, uint8_t Value)
{
	uint8_t CurrentRegValue;
	uint8_t WriteValue;

	MPU6050Flex_Status_t Status = MPU6050FLEX_SUCCESS;

	if (Mpu6050Config.pIORead(RegisterAddress,1,&CurrentRegValue) == IO_SUCCESS)
	{
		//Remove current parameter value and add specified value
		WriteValue = (CurrentRegValue & (~SegmentMask)) | Value;
		if (Mpu6050Config.pIOWrite(RegisterAddress,1,&WriteValue) != IO_SUCCESS)
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
/**
 * @brief Writes a byte to a given register
 *
 * @param RegisterAddress: Register address to be written to
 * @param Value: Value to write in the referenced register
 *
 * @return Status code to access operation success
 */
static MPU6050Flex_Status_t Mpu6050Flex_WriteFullRegister(uint8_t RegisterAddress, uint8_t Value)
{
	MPU6050Flex_Status_t Status = MPU6050FLEX_SUCCESS;

	if (Mpu6050Config.pIOWrite(RegisterAddress,1,&Value) != IO_SUCCESS)
	{
		Status = MPU6050FLEX_FAILURE;
	}

	return Status;
}

/**
 * @brief Simple inline-like function to asses if a given value fits within a given mask
 *
 * @param Value: Value being checked
 * @param Mask: Mask to check the value against
 *
 * @return Boolean variable indicating whether the value fits (true) or not (false)
 */
static bool Mpu6050Flex_ValueFitsInMask(uint8_t Value, uint8_t Mask)
{
	return !(Value & (~Mask));
}
/**
 * @brief Updates a given parameter based on a parameter mask, value and register address.
 *
 * @param ParameterValue: Value to set a given parameter to
 * @param ParameterMask: Parameter maks
 * @param RegisterAddress: Register address of where parameter is defined in MPU6050.
 *
 * @return Status code to access operation success
 */
static MPU6050Flex_Status_t Mpu6050Flex_UpdateParameter(uint8_t ParameterValue, uint8_t ParameterMask, uint8_t RegisterAddress)
{

	MPU6050Flex_Status_t Status = MPU6050FLEX_SUCCESS;

	if (ParameterMask == 0xFF)
	{
		/*If mask is 0xFF there is no need
		 * to read the current register value, so that step is bypassed
		 * in Mpu6050Flex_WriteFullRegister. */
		Mpu6050Flex_WriteFullRegister( RegisterAddress, ParameterValue);
	}
	else
	{
		/*Else we need to read it to know current value,
		 * so Mpu6050Flex_ReplaceRegisterSegment is called  */
		if (Mpu6050Flex_ValueFitsInMask(ParameterValue,ParameterMask))
		{
			Status = Mpu6050Flex_ReplaceRegisterSegment(RegisterAddress,ParameterMask,ParameterValue);
		}
		else
		{
			Status = MPU6050FLEX_FAILURE;
		}
	}
	return Status;
}





//Test function: dont bother commenting, will be replaced by set power mode function
MPU6050Flex_Status_t Mpu6050Flex_WakeUp()
{
	uint8_t Zero = 0;
	MPU6050Flex_Status_t Status = MPU6050FLEX_SUCCESS;

	if (Mpu6050Config.pIOWrite(REG_SMPRT_DIV,1,&Zero) != IO_SUCCESS)
	{
		Status = MPU6050FLEX_FAILURE;
	}
	return Status;
}
