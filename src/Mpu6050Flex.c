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
#include <string.h>
#include <stddef.h>
/**
 * @brief Type representing a structure containing the current set scale between accelerometer and
 * gyro measurements and the physical units (g and deg/s) in which they're represented (scale = FullScaleRange/((2^16)-1))
 */
typedef struct Mpu6050ScaleStruct
{
	float AccScale;
	float GyroScale;
} Mpu6050Flex_Scale_t;
/**
 * @brief Type representing a structure containing all configurable parameters in the Mpu6050 library.
 *
 */
typedef struct Mpu6050ConfigStruct
{
	IOFunc_t pIOWrite; /**< Pointer to the function used to write bytes to the mpu6050 registers */
	IOFunc_t pIORead;  /**< Pointer to the function used to read bytes from the mpu6050 registers */
	DelayFunc_t pDelay;/**< Pointer to the function used delay a given amount of ms */
} Mpu6050Flex_Config_t;

#define CALIBRATION_DURATION 3000 		/**< Duration of calibration period */
#define CALIBRATION_ITERATIONS_PO2 2	/**< Number to which 2 is raised to indicate nmbr of samples to be taken within calibration sec */
#define POW_2(n) (1 << (n))				/**< Macro to compute power of two */
#define CALIBRATION_DELAY 3000/POW_2(CALIBRATION_ITERATIONS_PO2) /**< Calibration delay to match above settings */

#define SCALE_MPU6050FLEX_GYRO_FS_SEL_250 	(500/((2^16)-1))
#define SCALE_MPU6050FLEX_GYRO_FS_SEL_500 	(1000/((2^16)-1))
#define SCALE_MPU6050FLEX_GYRO_FS_SEL_1000 	(2000/((2^16)-1))
#define SCALE_MPU6050FLEX_GYRO_FS_SEL_2000 	(4000/((2^16)-1))
#define SCALE_MPU6050FLEX_ACC_FS_SEL_2		(4/((2^16)-1))
#define SCALE_MPU6050FLEX_ACC_FS_SEL_4		(8/((2^16)-1))
#define SCALE_MPU6050FLEX_ACC_FS_SEL_8		(16/((2^16)-1))
#define SCALE_MPU6050FLEX_ACC_FS_SEL_16		(32/((2^16)-1))

/*Static function declarations*/
static MPU6050Flex_Status_t Mpu6050Flex_ReplaceRegisterSegment(uint8_t Register,uint8_t SegmentMask, uint8_t Value);
static MPU6050Flex_Status_t Mpu6050Flex_WriteFullRegister(uint8_t Register, uint8_t Value);
static bool Mpu6050Flex_ValueFitsInMask(uint8_t Option, uint8_t Mask);
static MPU6050Flex_Status_t Mpu6050Flex_UpdateParameter(uint8_t ParameterValue, uint8_t ParameterMask, uint8_t RegisterAddress);
static void Mpu6050Flex_AccumulateFullImuDataStruct(Mpu6050Flex_FullImuData32_t* pDest, Mpu6050Flex_ImuRawData_t* pAccData, Mpu6050Flex_ImuRawData_t* pGyroData);
static void Mpu6050Flex_DivideFullImuDataStruct(Mpu6050Flex_FullImuRawData_t* pDest, Mpu6050Flex_FullImuData32_t* pOrig, uint8_t RightShift);
static void Mpu6050Flex_SubtractImuDataStruct(Mpu6050Flex_ImuRawData_t* pA, Mpu6050Flex_ImuRawData_t* pB);

/**
 * @brief Calibration offsets struct used for the imu data readings
 */
static Mpu6050Flex_FullImuRawData_t Mpu6050ImuDataOffset = {0};
/**
 * @brief Mpu6050 Configuration struct
 */
static Mpu6050Flex_Config_t Mpu6050Config =
{
	.pIORead = NULL,
	.pIOWrite = NULL,
	.pDelay = NULL,
};
/**
 * @brief Mpu6050 Configuration struct
 */
static Mpu6050Flex_Scale_t Mpu6050Scale =
{
	.AccScale = 	SCALE_MPU6050FLEX_ACC_FS_SEL_2,
	.GyroScale = 	SCALE_MPU6050FLEX_GYRO_FS_SEL_250,
};
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
 * @brief Injects the Delay in ms function that this library uses
 *
 * @param pDelay Function Pointer that points to the delay function to use
 */
void Mpu6050Flex_SetDelay(DelayFunc_t pDelay)
{
	Mpu6050Config.pDelay = pDelay;
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
 * @brief Gets a pointer to the currently set delay in ms function pointer
 *
 * @return Pointer to the currently set delay function pointer
 */
DelayFunc_t Mpu6050Flex_GetDelay()
{
	return Mpu6050Config.pDelay;
}
/**
 * @brief Requests the i2c address of the mpu6050 device via i2c, used to verify working i2c interface
 *
 * @return I2C Address of the MPU6050 or 0xFF if it wasnt possible to obtain a valid address
 */
uint8_t Mpu6050Flex_WhoAmI()
{
	uint8_t Mpu6050Address = 0xFF;

	if (Mpu6050Config.pIORead)
	{
		if (Mpu6050Config.pIORead(REG_WHO_AM_I,1,&Mpu6050Address) != IO_SUCCESS)
		{
			Mpu6050Address = 0xFF;
		}
	}
	return Mpu6050Address;
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
 * Also configures the local gyro scale variable
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

	if (Status == MPU6050FLEX_SUCCESS)
	{
		switch (ConfigOption)
		{
		case MPU6050FLEX_GYRO_FS_SEL_250:
			Mpu6050Scale.GyroScale = SCALE_MPU6050FLEX_GYRO_FS_SEL_250;
			break;
		case MPU6050FLEX_GYRO_FS_SEL_500:
			Mpu6050Scale.GyroScale = SCALE_MPU6050FLEX_GYRO_FS_SEL_500;
			break;
		case MPU6050FLEX_GYRO_FS_SEL_1000:
			Mpu6050Scale.GyroScale = SCALE_MPU6050FLEX_GYRO_FS_SEL_1000;
			break;
		case MPU6050FLEX_GYRO_FS_SEL_2000:
			Mpu6050Scale.GyroScale = SCALE_MPU6050FLEX_GYRO_FS_SEL_2000;
			break;
		}
	}

	return Status;
}
/**
 * @brief Updates the AFS_SEL parameter of the ACCEL_CONFIG register, configuring the accelerometer full scale range.
 * Also configures the local accelerometer scale variable
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

	if (Status == MPU6050FLEX_SUCCESS)
		{
			switch (ConfigOption)
			{
			case MPU6050FLEX_ACC_FS_SEL_2:
				Mpu6050Scale.AccScale = SCALE_MPU6050FLEX_ACC_FS_SEL_2;
				break;
			case MPU6050FLEX_ACC_FS_SEL_4:
				Mpu6050Scale.AccScale = SCALE_MPU6050FLEX_ACC_FS_SEL_4;
				break;
			case MPU6050FLEX_ACC_FS_SEL_8:
				Mpu6050Scale.AccScale = SCALE_MPU6050FLEX_ACC_FS_SEL_8;
				break;
			case MPU6050FLEX_ACC_FS_SEL_16:
				Mpu6050Scale.AccScale = SCALE_MPU6050FLEX_ACC_FS_SEL_16;
				break;
			}
		}

	return Status;
}
/**
 * @brief Reads a given register,changes a segment of it based on a parameter
 * value and mask that fit within that register, and writes the updated value back.
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

	if (Mpu6050Config.pIORead && Mpu6050Config.pIOWrite)
	{
		if (Mpu6050Config.pIORead(RegisterAddress,1,&CurrentRegValue) == IO_SUCCESS)
		{
			/*Bitwise operation to remove current parameter value
			 * from read byte and to replace it with value specified in function arguments*/
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

	if (Mpu6050Config.pIOWrite)
	{
		if (Mpu6050Config.pIOWrite(RegisterAddress,1,&Value) != IO_SUCCESS)
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
		Status = Mpu6050Flex_WriteFullRegister( RegisterAddress, ParameterValue);
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
/**
 * @brief Verifies whether a given register is valid for a burst read of imu data (accel data or gyro data).
 * Only REG_ACCEL_XOUT_H and REG_GYRO_XOUT_H are valid for this purpose
 *
 * @param DataRegister: Register to be checked for validity
 *
 * @return Boolean value of whether or not register is valid for reading Imu Data
 */
static bool Mpu6050Flex_IsValidImuDataRegister(uint8_t DataRegister)
{
	bool ret = false;
	if ((DataRegister == REG_ACCEL_XOUT_H) || (DataRegister == REG_GYRO_XOUT_H))
	{
		ret = true;
	}
	return ret;
}
/**
 * @brief Reads 6 bytes of Imu data (gyro or accel data) from a given register and
 * returns a imu data struct with read data
 *
 * @param DataRegister: Register in which data read should start
 *
 * @return Struct containing imu data read or empty struct if any of the function steps fails
 */
static Mpu6050Flex_ImuRawData_t Mpu6050Flex_GetImuData(uint8_t DataRegister)
{
	uint8_t SerializedData[6];
	Mpu6050Flex_ImuRawData_t ImuData = {0};

	if (Mpu6050Config.pIORead)
	{
		if (Mpu6050Flex_IsValidImuDataRegister(DataRegister))
		{
			if (Mpu6050Config.pIORead(DataRegister,6,SerializedData) == IO_SUCCESS)
			{
				/*Copy serialized bytes into struct */
				memcpy(&(ImuData.RawDataX),SerializedData,2);
				memcpy(&(ImuData.RawDataY),SerializedData+2,2);
				memcpy(&(ImuData.RawDataZ),SerializedData+4,2);
			}
		}
	}
	return ImuData;
}
/**
 * @brief Reads 6 bytes of accelerometer IMU data and
 * returns a imu data struct with read data
 *
 * @return Struct containing raw acceleration imu data read or empty struct if any of the underlying function steps fails
 */
Mpu6050Flex_ImuRawData_t Mpu6050Flex_GetRawAccelData()
{
	return Mpu6050Flex_GetImuData(REG_ACCEL_XOUT_H);
}
/**
 * @brief Reads 6 bytes of accelerometer IMU data, subtracts previously computed calibration offset, and
 * returns a imu data struct with calibrated data
 *
 * @return Struct containing calibrated acceleration imu data read or empty struct if any of the underlying function steps fails
 */
Mpu6050Flex_ImuRawData_t Mpu6050Flex_GetAccelData()
{
	Mpu6050Flex_ImuRawData_t RetData;

	RetData = Mpu6050Flex_GetImuData(REG_ACCEL_XOUT_H);
	Mpu6050Flex_SubtractImuDataStruct(&RetData,&Mpu6050ImuDataOffset.AccData);

	return RetData;
}
/**
 * @brief Reads 6 bytes of gyro IMU data and
 * returns a imu data struct with read data
 *
 * @return Struct containing raw gyro imu data read or empty struct if any of the underlying function steps fails
 */
Mpu6050Flex_ImuRawData_t Mpu6050Flex_GetRawGyroData()
{
	return Mpu6050Flex_GetImuData(REG_GYRO_XOUT_H);
}
/**
 * @brief Reads 6 bytes of gyro IMU data, subtracts previously computed calibration offset, and
 * returns a imu data struct with calibrated data
 *
 * @return Struct containing calibrated gyro imu data read or empty struct if any of the underlying function steps fails
 */
Mpu6050Flex_ImuRawData_t Mpu6050Flex_GetGyroData()
{
	Mpu6050Flex_ImuRawData_t RetData;

	RetData = Mpu6050Flex_GetImuData(REG_GYRO_XOUT_H);
	Mpu6050Flex_SubtractImuDataStruct(&RetData,&Mpu6050ImuDataOffset.GyroData);

	return RetData;
}
/**
 * @brief Function used to be obtain the current calibration offset values for the accelerometer and gyro readings
 *
 * @return Struct containing the current calibration offset values for the accelerometer and gyro readings
 */
Mpu6050Flex_FullImuRawData_t Mpu6050Flex_GetImuDataOffsets()
{
	return Mpu6050ImuDataOffset;
}
/**
 * @brief Subtracts the fields of a given Mpu6050Flex_ImuRawData_t struct
 * struct by the fields of another Mpu6050Flex_ImuRawData_t struct
 *
 * @param pA: Pointer to the target Mpu6050Flex_ImuRawData_t struct
 * @param pB: Pointer to the Mpu6050Flex_ImuRawData_t struct that will be structed for the target
 *
 */
static void Mpu6050Flex_SubtractImuDataStruct(Mpu6050Flex_ImuRawData_t* pA, Mpu6050Flex_ImuRawData_t* pB)
{
	pA->RawDataX = pA->RawDataX - pB->RawDataX;
	pA->RawDataY = pA->RawDataY - pB->RawDataY;
	pA->RawDataZ = pA->RawDataZ - pB->RawDataZ;
}
/**
 * @brief Adds the fields of two (accel and gyro) given Mpu6050Flex_ImuRawData_t struct
 * to a target Mpu6050Flex_FullImuData32_t struct containing gyro and accel data.
 * This function is nominally used to accumulate imu data readings in a struct with 32 bit elements
 * that can go over the 16bit limit of the default struct when calculating reading average for calibration purposes
 *
 * @param pDest: Pointer to the target Mpu6050Flex_FullImuData32_t struct in which data is accumulated
 * @param pAccData: Pointer to a Mpu6050Flex_ImuRawData_t struct containing accel data. Nominally pointer to a just read set of accel readings
 * @param pGyroData: Pointer to a Mpu6050Flex_ImuRawData_t struct containing gyro data. Nominally pointer to a just read set of accel readings
 *
 */
static void Mpu6050Flex_AccumulateFullImuDataStruct(Mpu6050Flex_FullImuData32_t* pDest, Mpu6050Flex_ImuRawData_t* pAccData, Mpu6050Flex_ImuRawData_t* pGyroData)
{
	pDest->AccData.RawDataX 	=  pDest->AccData.RawDataX + pAccData->RawDataX;
	pDest->AccData.RawDataY 	=  pDest->AccData.RawDataY + pAccData->RawDataY;
	pDest->AccData.RawDataZ 	=  pDest->AccData.RawDataZ + pAccData->RawDataZ;
	pDest->GyroData.RawDataX 	=  pDest->GyroData.RawDataX + pGyroData->RawDataX;
	pDest->GyroData.RawDataY 	=  pDest->GyroData.RawDataY + pGyroData->RawDataY;
	pDest->GyroData.RawDataZ 	=  pDest->GyroData.RawDataZ + pGyroData->RawDataZ;

}
/**
 * @brief Divides a Full (gyro+accel) IMU struct by a given power of two, and writes the result in a target IMU Full data struct.
 * This function is Used to average out accumulated IMU data readings
 *
 * @param pDest: Pointer to the target Mpu6050Flex_FullImuRawData_t struct in which divided/average data is written
 * @param pOrig: Pointer to struct that will be divided
 * @param Rightshift: Number to which 2 is raised to to indicate divisor/total samples
 * (ex. RightShift = 2 -> Struct will be divided by 2^2 = 4)
 *
 */
static void Mpu6050Flex_DivideFullImuDataStruct(Mpu6050Flex_FullImuRawData_t* pDest, Mpu6050Flex_FullImuData32_t* pOrig, uint8_t RightShift)
{
	pDest->AccData.RawDataX =  pOrig->AccData.RawDataX >> RightShift;
	pDest->AccData.RawDataY =  pOrig->AccData.RawDataY >> RightShift;
	pDest->AccData.RawDataZ =  pOrig->AccData.RawDataZ >> RightShift;
	pDest->GyroData.RawDataX =  pOrig->GyroData.RawDataX >> RightShift;
	pDest->GyroData.RawDataY =  pOrig->GyroData.RawDataY >> RightShift;
	pDest->GyroData.RawDataZ =  pOrig->GyroData.RawDataZ >> RightShift;
}
/**
 * @brief Obtains a  Full (gyro+accel) calibration offset IMU data struct that will be used later to compute calibration data.
 * To do so, a given amount of samples defined earlier (must be a power of two) is obtained in an amount of time defined earlier
 * and accumulated in a 32 bit Full IMU data struct,and later the accumulated data is averaged out and written in a 16 bit IMU
 * data struct, to give the average calibration data offset.
 * The IMU must be stationary and in a resting position for the specified amount of time for the calibration to work.
 *
 * @return Status code to access operation success
 */
MPU6050Flex_Status_t Mpu6050Flex_Calibrate()
{
	Mpu6050Flex_FullImuData32_t TempFullImuData = {0};
	Mpu6050Flex_ImuRawData_t TempAccData;
	Mpu6050Flex_ImuRawData_t TempGyroData;

	uint8_t idx = 0;
	for (idx=0;idx<POW_2(CALIBRATION_ITERATIONS_PO2);idx++)
	{
		/*Obtain new samples*/
		TempAccData  = Mpu6050Flex_GetRawAccelData();
		TempGyroData = Mpu6050Flex_GetRawGyroData();

		/*Accumulate data that will be averaged out later */
		Mpu6050Flex_AccumulateFullImuDataStruct(&TempFullImuData,&TempAccData,&TempGyroData);
		Mpu6050Config.pDelay(CALIBRATION_DELAY);

	}

	/*Compute average data offsets for both gyro and accelerometer data ~
	 * and write it in static imu data offset struct*/
	Mpu6050Flex_DivideFullImuDataStruct(&Mpu6050ImuDataOffset,&TempFullImuData,CALIBRATION_ITERATIONS_PO2);

	MPU6050Flex_Status_t Status = MPU6050FLEX_SUCCESS;

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
