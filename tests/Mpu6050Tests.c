#include "Mpu6050Flex.h"
#include "unity.h"
#include "unity_fixture.h"

#include "Mpu6050_MockIO.h"
#include <stdbool.h>
/**
 * @brief Compares two Mpu6050Flex_ImuData_t structs
 *
 * @param pA First Mpu6050Flex_ImuData_t to be compared
 * @param pB Second Mpu6050Flex_ImuData_t to be compared
 *
 * @return boolean value indicating if structs are equal
 */
static bool CompareDataStructs(Mpu6050Flex_ImuData_t* pA, Mpu6050Flex_ImuData_t* pB)
{
	bool ret = true;

	if (pA->DataX != pB->DataX)
	{
		ret = false;
	}
	else
	{
		if (pA->DataY != pB->DataY)
		{
				ret = false;
		}
		else
		{
			if (pA->DataZ != pB->DataZ)
			{
				ret = false;
			}
		}
	}


	if (ret == false)
	{
		printf("\n A: DataX = 0x%.2x, DataY = 0x%.2x, DataZ = 0x%.2x \n",pA->DataX,pA->DataY,pA->DataZ);
		printf("\n B: DataX = 0x%.2x, DataY = 0x%.2x, DataZ = 0x%.2x \n",pB->DataX,pB->DataY,pB->DataZ);
	}

	return ret;
}
/**
 * @brief Sets up mock expectations for an update operation on
 * a parameter in a register (read current value and write updated value)
 *
 * @param ExpReadValue value of the register to be updated before the update
 * @param ParamMask mask of the parameter to be updated
 * @param ConfigOption parameter option to be set
 * @param RegisterAddress address of the register to be changed
 *
 */
static void SetupParameterUpdateExpectations(uint8_t ExpReadValue,
										uint8_t ParamMask,
										uint8_t ConfigOption,
										uint8_t RegisterAddress)
{

	/*Expect a read to obtain current register value*/
	MockMpu6050IO_ExpectReadAndReturn(RegisterAddress,1,&ExpReadValue);

	uint8_t ExpWrite = (ExpReadValue & (~ParamMask)) | ConfigOption;

	/*Expect a write to update register value considering new parameter*/
	MockMpu6050IO_ExpectWrite(RegisterAddress,1,&ExpWrite);
}
/**
 * @brief Sets up mock expectations for an Imu Data Read (expect a read of 6 imu data bytes)
 * and assign expected values to a provided expected data struct
 * that will be compared to returned data
 *
 * @param pExpData IMU data struct where expected values are set
 * @param DataReg register address where 6 bytes should be read
 *
 */
static void SetupDataFunctionExpectations(Mpu6050Flex_ImuData_t* pExpData, uint8_t DataReg)
{
	uint8_t DataOut[6] = {0xBB,0xAA,0xDD,0xCC,0xFF,0xEE};

	MockMpu6050IO_ExpectReadAndReturn(DataReg,6,DataOut);

	pExpData->DataX = 0xAABB;
	pExpData->DataY = 0xCCDD;
	pExpData->DataZ = 0xEEFF;
}
/**
 * @brief Sets up mock expectations for an Imu Calibrated Data Read (expect calibration reads and a read of 6 imu data bytes)
 * and assign expected values to a provided expected data struct that will be compared to returned data
 *
 * @param pExpData IMU data struct where expected values are set
 * @param DataReg register address where 6 bytes should be read
 *
 */
static void SetupCalibratedDataFunctionExpectations(Mpu6050Flex_ImuData_t* pExpData, uint8_t DataReg)
{
	uint8_t DataOut[6] = {0xBB,0xAA,0xDD,0xCC,0xFF,0xEE};

	/*Expected read of 6 data bytes*/
	MockMpu6050IO_ExpectReadAndReturn(DataReg,6,DataOut);

	Mpu6050Flex_FullImuData_t ImuDataOffset;

	ImuDataOffset = Mpu6050Flex_GetImuDataOffsets();

	/*Assign expected return values in struct, considering currently set calibration offset */
	if (DataReg == REG_GYRO_XOUT_H)
	{
		pExpData->DataX = 0xAABB - ImuDataOffset.GyroData.DataX;
		pExpData->DataY = 0xCCDD - ImuDataOffset.GyroData.DataY;
		pExpData->DataZ = 0xEEFF - ImuDataOffset.GyroData.DataZ;
	}
	else if (REG_ACCEL_XOUT_H)
	{
		pExpData->DataX = 0xAABB - ImuDataOffset.AccData.DataX;
		pExpData->DataY = 0xCCDD - ImuDataOffset.AccData.DataY;
		pExpData->DataZ = 0xEEFF - ImuDataOffset.AccData.DataZ;
	}
}

/**
 * @brief Sets up mock expectations for an Imu Calibration (expect calibration reads and a get of current milliseconds)
 */
static void SetupCalibrationExpectations()
{
	uint8_t AccData1[6] = {0x00,0x70,0x00,0x72,0x01,0x7F}; //1st set of readings
	uint8_t AccData2[6] = {0x00,0x71,0x00,0x73,0x01,0x7F}; //2nd set of readings
	uint8_t AccData3[6] = {0x00,0x71,0x00,0x73,0x01,0x7F}; //3rd set of readings
	uint8_t AccData4[6] = {0x00,0x72,0x00,0x74,0x01,0x7F}; //4th set of readings

	uint8_t GyroData1[6] = {0x00,0x77,0x00,0x70,0x05,0x7E}; //1st set of readings
	uint8_t GyroData2[6] = {0x00,0x78,0x00,0x71,0x05,0x7E}; //2nd set of readings
	uint8_t GyroData3[6] = {0x00,0x78,0x00,0x71,0x05,0x7E}; //3rd set of readings
	uint8_t GyroData4[6] = {0x00,0x79,0x00,0x72,0x05,0x7E}; //4th set of readings

	MockMpu6050IO_ExpectReadAndReturn(REG_ACCEL_XOUT_H,6,AccData1);
	MockMpu6050IO_ExpectReadAndReturn(REG_GYRO_XOUT_H,6,GyroData1);
	MockMpu6050IO_ExpectReadAndReturn(REG_ACCEL_XOUT_H,6,AccData2);
	MockMpu6050IO_ExpectReadAndReturn(REG_GYRO_XOUT_H,6,GyroData2);
	MockMpu6050IO_ExpectReadAndReturn(REG_ACCEL_XOUT_H,6,AccData3);
	MockMpu6050IO_ExpectReadAndReturn(REG_GYRO_XOUT_H,6,GyroData3);
	MockMpu6050IO_ExpectReadAndReturn(REG_ACCEL_XOUT_H,6,AccData4);
	MockMpu6050IO_ExpectReadAndReturn(REG_GYRO_XOUT_H,6,GyroData4);
	MockMpu6050IO_ExpectGetMsAndReturn(25);
}

/**
 * @brief Test group referring to functions used prior to mpu6050 operation
 */
TEST_GROUP(Mpu6050SetupTests);

TEST_SETUP(Mpu6050SetupTests)
{
	MockMpu6050IO_Create(20);
}

TEST_TEAR_DOWN(Mpu6050SetupTests)
{
	MockMpu6050IO_VerifyComplete();
	MockMpu6050IO_Destroy();
}

/**
 * @brief Tests that Mpu6050Flex_SetIOWrite sets IO write function pointer appropriately
 */
TEST(Mpu6050SetupTests,SetWriteIOSetsCorrectly)
{
	Mpu6050Flex_SetIOWrite(MockMpu6050IO_Write);

	TEST_ASSERT_POINTERS_EQUAL(MockMpu6050IO_Write,Mpu6050Flex_GetIOWrite());
}
/**
 * @brief Tests that Mpu6050Flex_SetIORead sets IO read function pointer appropriately
 */
TEST(Mpu6050SetupTests,SetReadIOSetsCorrectly)
{
	Mpu6050Flex_SetIORead(MockMpu6050IO_ReadAndReturn);

	TEST_ASSERT_POINTERS_EQUAL(MockMpu6050IO_ReadAndReturn,Mpu6050Flex_GetIORead());
}
/**
 * @brief Tests that Mpu6050Flex_SetDelay sets delay function pointer appropriately
 */
TEST(Mpu6050SetupTests,SetDelaySetsCorrectly)
{
	Mpu6050Flex_SetDelay(MockMpu6050_Delay);

	TEST_ASSERT_POINTERS_EQUAL(MockMpu6050_Delay,Mpu6050Flex_GetDelay());
}
/**
 * @brief Tests that Mpu6050Flex_SetGetMs sets Get Milliseconds function pointer appropriately
 */
TEST(Mpu6050SetupTests,SetGetMsSetsCorrectly)
{
	Mpu6050Flex_SetGetMs(MockMpu6050_GetMs);

	TEST_ASSERT_POINTERS_EQUAL(MockMpu6050_GetMs,Mpu6050Flex_GetGetMs());
}
///----------------------------------------------------------------------------
/**
 * @brief Test group referring to general Mpu6050 functions
 */
TEST_GROUP(Mpu6050Tests);

TEST_SETUP(Mpu6050Tests)
{
	MockMpu6050IO_Create(20);
	Mpu6050Flex_SetIORead(MockMpu6050IO_ReadAndReturn);
	Mpu6050Flex_SetIOWrite(MockMpu6050IO_Write);
	Mpu6050Flex_SetDelay(MockMpu6050_Delay);
	Mpu6050Flex_SetGetMs(MockMpu6050_GetMs);
}

TEST_TEAR_DOWN(Mpu6050Tests)
{
	MockMpu6050IO_VerifyComplete();
	MockMpu6050IO_Destroy();
}
/**
 * @brief Tests that Mpu6050Flex_WhoAmI follows correct sequence of interactions with the mpu6050 device
 */
TEST(Mpu6050Tests,ReadWhoAmIFollowsSequence)
{
	uint8_t Output = 0x68;
	MockMpu6050IO_ExpectReadAndReturn(0x75,1,&Output);

	TEST_ASSERT_EQUAL_UINT8(0x68,Mpu6050Flex_WhoAmI());
}
/**
 * @brief Tests that Mpu6050Flex_ConfigSampleRateDivider follows correct sequence of interactions with the mpu6050 device
 */
TEST(Mpu6050Tests,ConfigureSampleRateDividerFollowsSequence)
{
	uint8_t Input = 0x00;

	MockMpu6050IO_ExpectWrite(0x19,1,&Input);

	TEST_ASSERT_EQUAL(MPU6050FLEX_SUCCESS,Mpu6050Flex_ConfigSampleRateDivider(0x00));
}
/**
 * @brief Tests that Mpu6050Flex_DigitalLowPassFilter follows correct sequence of interactions with the mpu6050 device
 *  and doesn't overwrite previously set register contents
 */
TEST(Mpu6050Tests,ConfigureDigitalLowPassFilterFollowsSequenceAndDoesntOverwrite)
{

	uint8_t ConfigOption = MPU6050FLEX_DLPF_CFG2;
	uint8_t ExpReadValue = 0x10;
	uint8_t ParamMask = MPU6050FLEX_DLPF_CFG_MSK;
	uint8_t ParamReg = 0x1A;

	SetupParameterUpdateExpectations(ExpReadValue,ParamMask,ConfigOption,ParamReg);

	TEST_ASSERT_EQUAL(MPU6050FLEX_SUCCESS,Mpu6050Flex_ConfigDigitalLowPassFilter(MPU6050FLEX_DLPF_CFG2));
}
/**
 * @brief Tests that using Mpu6050Flex_ConfigDigitalLowPassFilter with bad arguments return an error status
 */
TEST(Mpu6050Tests,ConfigureDigitalLowPassFilterRejectsBadArgs)
{
	TEST_ASSERT_EQUAL(MPU6050FLEX_FAILURE,Mpu6050Flex_ConfigDigitalLowPassFilter(0x10));
}
/**
 * @brief Tests that Mpu6050Flex_ConfigureGyroFullScaleRange follows correct sequence of interactions with the mpu6050 device
 *  and doesn't overwrite previously set register contents
 */
TEST(Mpu6050Tests,ConfigureGyroFullScaleRangeFollowsSequenceAndDoesntOverwrite)
{
	uint8_t ConfigOption = MPU6050FLEX_GYRO_FS_SEL_500;
	uint8_t ExpReadValue = 0x20;
	uint8_t ParamMask = MPU6050FLEX_FS_SEL_MSK;
	uint8_t ParamReg = 0x1B;

	SetupParameterUpdateExpectations(ExpReadValue,ParamMask,ConfigOption,ParamReg);

	TEST_ASSERT_EQUAL(MPU6050FLEX_SUCCESS,Mpu6050Flex_ConfigGyroFullScaleRange(MPU6050FLEX_GYRO_FS_SEL_500));
	TEST_ASSERT_EQUAL(66,Mpu6050Flex_GetGyroScale());
}
/**
 * @brief Tests that using Mpu6050Flex_ConfigGyroFullScaleRange with bad arguments return an error status
 */
TEST(Mpu6050Tests,ConfigureGyroFullScaleRangeRejectsBadArgs)
{
	TEST_ASSERT_EQUAL(MPU6050FLEX_FAILURE,Mpu6050Flex_ConfigGyroFullScaleRange(0x20));
}
/**
 * @brief Tests that Mpu6050Flex_ConfigureAccFullScaleRange follows correct sequence of interactions with the mpu6050 device
 *  and doesn't overwrite previously set register contents
 */
TEST(Mpu6050Tests,ConfigureAccFullScaleRangeFollowsSequenceAndDoesntOverwrite)
{
	uint8_t ConfigOption = MPU6050FLEX_ACC_FS_SEL_8;
	uint8_t ExpReadValue = 0x20;
	uint8_t ParamMask = MPU6050FLEX_AFS_SEL_MSK;
	uint8_t ParamReg = 0x1C;

	SetupParameterUpdateExpectations(ExpReadValue,ParamMask,ConfigOption,ParamReg);

	TEST_ASSERT_EQUAL(MPU6050FLEX_SUCCESS,Mpu6050Flex_ConfigAccFullScaleRange(MPU6050FLEX_ACC_FS_SEL_8));
	TEST_ASSERT_EQUAL(4096,Mpu6050Flex_GetAccScale());
}
/**
 * @brief Tests that using Mpu6050Flex_ConfigAccFullScaleRange with bad arguments return an error status
 */
TEST(Mpu6050Tests,ConfigureAccFullScaleRangeRejectsBadArgs)
{
	TEST_ASSERT_EQUAL(MPU6050FLEX_FAILURE,Mpu6050Flex_ConfigAccFullScaleRange(0x20));
}
/**
 * @brief Tests that Mpu6050Flex_GetRawAccelData follows correct sequence of interactions with the mpu6050 device
 */
TEST(Mpu6050Tests,GetRawAccDataFollowsCorrectOrderAndReturnsExpectedData)
{

	Mpu6050Flex_ImuData_t ExpectedAccData;
	Mpu6050Flex_ImuData_t ActualAccData;

	SetupDataFunctionExpectations(&ExpectedAccData,0x3B);
	ActualAccData = Mpu6050Flex_GetRawAccelData();

	TEST_ASSERT(CompareDataStructs(&ExpectedAccData,&ActualAccData));
}
/**
 * @brief Tests that Mpu6050Flex_GetRawGyroData follows correct sequence of interactions with the mpu6050 device
 */
TEST(Mpu6050Tests,GetRawGyroDataFollowsCorrectOrderAndReturnsExpectedData)
{

	Mpu6050Flex_ImuData_t ExpectedGyroData;
	Mpu6050Flex_ImuData_t ActualGyroData;

	SetupDataFunctionExpectations(&ExpectedGyroData,0x43);
	ActualGyroData = Mpu6050Flex_GetRawGyroData();

	TEST_ASSERT(CompareDataStructs(&ExpectedGyroData,&ActualGyroData));
}
/**
 * @brief Tests that Mpu6050Flex_SetComplementaryFilterCoeffs
 * the internal complementary filter coefficients correctly
 */
TEST(Mpu6050Tests,SetupComplementaryFilterSetsCF)
{
	float TestGyroCoeff = 0.98;
	float TestAccCoeff = 0.02;

	TEST_ASSERT_EQUAL(MPU6050FLEX_SUCCESS,Mpu6050Flex_SetComplementaryFilterCoeffs(TestGyroCoeff, TestAccCoeff));
	TEST_ASSERT_EQUAL_FLOAT(TestGyroCoeff,Mpu6050Flex_GetGyroCFCoeff());
	TEST_ASSERT_EQUAL_FLOAT(TestAccCoeff,Mpu6050Flex_GetAccCFCoeff());
}
/**
 * @brief Tests that using Mpu6050Flex_SetComplementaryFilterCoeffs with bad arguments return an error status
 */
TEST(Mpu6050Tests,SetupComplementaryRejectsBadArgs)
{
	float TestGyroCoeff = 0.91;
	float TestAccCoeff = 0.12;

	TEST_ASSERT_EQUAL(MPU6050FLEX_FAILURE,Mpu6050Flex_SetComplementaryFilterCoeffs(TestGyroCoeff, TestAccCoeff));
}
/**
 * @brief Tests that Mpu6050Flex_Sleep follows correct sequence of interactions with the mpu6050 device
 *  and doesn't overwrite previously set register contents
 */
TEST(Mpu6050Tests,SleepFollowsSequenceAndDoesntOverwrite)
{

	uint8_t ConfigOption = MPU6050FLEX_SLEEP_SLEEP;
	uint8_t ExpReadValue = 0x10;
	uint8_t ParamMask = MPU6050FLEX_SLEEP_MSK;
	uint8_t ParamReg = 0x6B;

	SetupParameterUpdateExpectations(ExpReadValue,ParamMask,ConfigOption,ParamReg);

	TEST_ASSERT_EQUAL(MPU6050FLEX_SUCCESS,Mpu6050Flex_Sleep());
}
/**
 * @brief Tests that Mpu6050Flex_WakeUp follows correct sequence of interactions with the mpu6050 device
 *  and doesn't overwrite previously set register contents
 */
TEST(Mpu6050Tests,WakeUpFollowsSequenceAndDoesntOverwrite)
{

	uint8_t ConfigOption = MPU6050FLEX_SLEEP_WAKE;
	uint8_t ExpReadValue = 0x10;
	uint8_t ParamMask = MPU6050FLEX_SLEEP_MSK;
	uint8_t ParamReg = 0x6B;

	SetupParameterUpdateExpectations(ExpReadValue,ParamMask,ConfigOption,ParamReg);

	TEST_ASSERT_EQUAL(MPU6050FLEX_SUCCESS,Mpu6050Flex_WakeUp());
}

//------------------------------------------------------------------------------------
/**
 * @brief Test group referring to Mpu6050 functions that require the call of the calibration functions before hand.
 * This means that in each setup calibration expectations are set and met
 */
TEST_GROUP(Mpu6050CalibratedTests);

TEST_SETUP(Mpu6050CalibratedTests)
{
	MockMpu6050IO_Create(20);
	Mpu6050Flex_SetIORead(MockMpu6050IO_ReadAndReturn);
	Mpu6050Flex_SetIOWrite(MockMpu6050IO_Write);
	Mpu6050Flex_SetDelay(MockMpu6050_Delay);
	Mpu6050Flex_SetGetMs(MockMpu6050_GetMs);

	SetupCalibrationExpectations();
	Mpu6050Flex_Calibrate();

}

TEST_TEAR_DOWN(Mpu6050CalibratedTests)
{
	MockMpu6050IO_VerifyComplete();
	MockMpu6050IO_Destroy();
}

/**
 * @brief Test that Mpu6050Flex_Calibrate will write the average of the calibration readings (set in expectations)
 * in the ImuDataOffsets struct. Note that in the case of the accelerometer Z axis the average is subtracted by 0x4000 (1g equivalent)
 * so that even when staying put, we still read 1g on the Z axis.
 */
TEST(Mpu6050CalibratedTests,CalibrateWritesAverageReadDataInInternalStructure)
{

	Mpu6050Flex_FullImuData_t ActualDataStruct;
	Mpu6050Flex_FullImuData_t ExpectedDataStruct =
	{
		.GyroData.DataX 	= 0x7800,
		.GyroData.DataY 	= 0x7100,
		.GyroData.DataZ 	= 0x7E05,
		.AccData.DataX 		= 0x7100,
		.AccData.DataY 		= 0x7300,
		.AccData.DataZ 		= 0x7F01-0x4000,
	};

	ActualDataStruct = Mpu6050Flex_GetImuDataOffsets();

	TEST_ASSERT(CompareDataStructs(&ExpectedDataStruct.GyroData,&ActualDataStruct.GyroData));
	TEST_ASSERT(CompareDataStructs(&ExpectedDataStruct.AccData,&ActualDataStruct.AccData));

}
/**
 * @brief Test that Mpu6050Flex_Calibrate will write the last gyro read time (set in expectations)
 * in the LastGyroReadTime static variable.
 */
TEST(Mpu6050CalibratedTests,CalibrateSetsLastGyroReadTime)
{
	TEST_ASSERT_EQUAL(25,Mpu6050Flex_GetLastGyroReadTime());
}

/**
 * @brief Test that Mpu6050Flex_GyroData interacts with the MPU6050 device in the expected sequence
 * and that the returned calibrated data corresponds to the read data minus the calibration offset
 */
TEST(Mpu6050CalibratedTests,GetGyroDataFollowsCorrectOrderAndReturnsExpectedData)
{
	Mpu6050Flex_ImuData_t ExpectedGyroData;
	Mpu6050Flex_ImuData_t ActualGyroData;

	SetupCalibratedDataFunctionExpectations(&ExpectedGyroData,0x43);

	ActualGyroData = Mpu6050Flex_GetGyroData();

	TEST_ASSERT(CompareDataStructs(&ExpectedGyroData,&ActualGyroData));

}
/**
 * @brief Test that Mpu6050Flex_GetAccelData interacts with the MPU6050 device in the expected sequence
 * and that the returned calibrated data corresponds to the read data minus the calibration offset
 */
TEST(Mpu6050CalibratedTests,GetAccDataFollowsCorrectOrderAndReturnsExpectedData)
{
	Mpu6050Flex_ImuData_t ExpectedAccData;
	Mpu6050Flex_ImuData_t ActualAccData;

	SetupCalibratedDataFunctionExpectations(&ExpectedAccData,0x3B);

	ActualAccData = Mpu6050Flex_GetAccelData();

	TEST_ASSERT(CompareDataStructs(&ExpectedAccData,&ActualAccData));

}


TEST(Mpu6050CalibratedTests,GetAccEulerFollowsCorrectOrderAndReturnsExpectedData)
{
	TEST_ASSERT(0);
}
//Test for accel get euler
//test for gyro get euler -> remember, this takes into account past measurements (which include acceleromter contributions)
//test for get euler (calls other two and computes euler based on coeffs)

