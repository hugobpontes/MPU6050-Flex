#include "Mpu6050Flex.h"
#include "unity.h"
#include "unity_fixture.h"

#include "Mpu6050_MockIO.h"
#include <stdbool.h>

/*This needs refactoring obviously*/
/*Tests for strange conditions like sensor being saturated (or close) during
 * calibration, lack of calibration, or off-nominal attitude conditions aren't done
 * so those cases are unhandled.
 * Providing a super robust library is beyond my goal with writing this library */
uint8_t DataOut[6] = {0x22,0x11,0x44,0x33,0x66,0x55};
uint16_t DataOut16bit[3] = {0x1122,0x3344,0x5566};

uint8_t AccCalibData1[6] = {0x00,0x00,0x00,0x02,0x01,0x5F}; //1st set of readings
uint8_t AccCalibData2[6] = {0x00,0x01,0x00,0x03,0x01,0x5F}; //2nd set of readings
uint8_t AccCalibData3[6] = {0x00,0x01,0x00,0x03,0x01,0x5F}; //3rd set of readings
uint8_t AccCalibData4[6] = {0x00,0x02,0x00,0x04,0x01,0x5F}; //4th set of readings

uint8_t GyroCalibData1[6] = {0x00,0x07,0x00,0x00,0x05,0x0E}; //1st set of readings
uint8_t GyroCalibData2[6] = {0x00,0x08,0x00,0x01,0x05,0x0E}; //2nd set of readings
uint8_t GyroCalibData3[6] = {0x00,0x08,0x00,0x01,0x05,0x0E}; //3rd set of readings
uint8_t GyroCalibData4[6] = {0x00,0x09,0x00,0x02,0x05,0x0E}; //4th set of readings

uint16_t CalibOffsetData[6] = {0x0800,0x0100,0x0E05,0x0100,0x0300,0x5F01-0x4000};

uint16_t GyroDataOut16bitAfterCalib[3] = {0x1122-0x0800,0x3344-0x0100,0x5566-0x0E05};
uint16_t AccDataOut16bitAfterCalib[3] = {0x1122-0x0100,0x3344-0x0300,0x5566-(0x5F01-0x4000)};

static Mpu6050Flex_t Mpu6050;

//Expected gyro euler
//Expected acc euler
//Expected filtered euler

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

	MockMpu6050IO_ExpectReadAndReturn(DataReg,6,DataOut);

	pExpData->DataX = DataOut16bit[0];
	pExpData->DataY = DataOut16bit[1];
	pExpData->DataZ = DataOut16bit[2];
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


	/*Expected read of 6 data bytes*/
	MockMpu6050IO_ExpectReadAndReturn(DataReg,6,DataOut);

	Mpu6050Flex_FullImuData_t ImuDataOffset;

	ImuDataOffset = Mpu6050Flex_GetImuDataOffsets(Mpu6050);

	/*Assign expected return values in struct, considering currently set calibration offset */
	if (DataReg == REG_GYRO_XOUT_H)
	{
		pExpData->DataX = GyroDataOut16bitAfterCalib[0];
		pExpData->DataY = GyroDataOut16bitAfterCalib[1];
		pExpData->DataZ = GyroDataOut16bitAfterCalib[2];
	}
	else if (REG_ACCEL_XOUT_H)
	{
		pExpData->DataX = AccDataOut16bitAfterCalib[0];
		pExpData->DataY = AccDataOut16bitAfterCalib[1];
		pExpData->DataZ = AccDataOut16bitAfterCalib[2];
	}
}

/**
 * @brief Sets up mock expectations for an Imu Calibration (expect calibration reads and a get of current milliseconds)
 */
static void SetupCalibrationExpectations()
{

	MockMpu6050IO_ExpectReadAndReturn(REG_ACCEL_XOUT_H,6,AccCalibData1);
	MockMpu6050IO_ExpectReadAndReturn(REG_GYRO_XOUT_H,6,GyroCalibData1);
	MockMpu6050IO_ExpectReadAndReturn(REG_ACCEL_XOUT_H,6,AccCalibData2);
	MockMpu6050IO_ExpectReadAndReturn(REG_GYRO_XOUT_H,6,GyroCalibData2);
	MockMpu6050IO_ExpectReadAndReturn(REG_ACCEL_XOUT_H,6,AccCalibData3);
	MockMpu6050IO_ExpectReadAndReturn(REG_GYRO_XOUT_H,6,GyroCalibData3);
	MockMpu6050IO_ExpectReadAndReturn(REG_ACCEL_XOUT_H,6,AccCalibData4);
	MockMpu6050IO_ExpectReadAndReturn(REG_GYRO_XOUT_H,6,GyroCalibData4);
	MockMpu6050IO_ExpectGetMsAndReturn(25);
}

/**
 * @brief Test group referring to functions used prior to mpu6050 operation
 */
TEST_GROUP(Mpu6050SetupTests);

TEST_SETUP(Mpu6050SetupTests)
{
	MockMpu6050IO_Create(20);
	Mpu6050 = Mpu6050Flex_Create();
}

TEST_TEAR_DOWN(Mpu6050SetupTests)
{
	MockMpu6050IO_VerifyComplete();
	MockMpu6050IO_Destroy();
	Mpu6050Flex_Destroy(Mpu6050);

}

/**
 * @brief Tests that Mpu6050Flex_SetIOWrite sets IO write function pointer appropriately
 */
TEST(Mpu6050SetupTests,SetWriteIOSetsCorrectly)
{
	Mpu6050Flex_SetIOWrite(Mpu6050,MockMpu6050IO_Write);

	TEST_ASSERT_POINTERS_EQUAL(MockMpu6050IO_Write,Mpu6050Flex_GetIOWrite(Mpu6050));
}
/**
 * @brief Tests that Mpu6050Flex_SetIORead sets IO read function pointer appropriately
 */
TEST(Mpu6050SetupTests,SetReadIOSetsCorrectly)
{
	Mpu6050Flex_SetIORead(Mpu6050,MockMpu6050IO_ReadAndReturn);

	TEST_ASSERT_POINTERS_EQUAL(MockMpu6050IO_ReadAndReturn,Mpu6050Flex_GetIORead(Mpu6050));
}
/**
 * @brief Tests that Mpu6050Flex_SetDelay sets delay function pointer appropriately
 */
TEST(Mpu6050SetupTests,SetDelaySetsCorrectly)
{
	Mpu6050Flex_SetDelay(Mpu6050,MockMpu6050_Delay);

	TEST_ASSERT_POINTERS_EQUAL(MockMpu6050_Delay,Mpu6050Flex_GetDelay(Mpu6050));
}
/**
 * @brief Tests that Mpu6050Flex_SetGetMs sets Get Milliseconds function pointer appropriately
 */
TEST(Mpu6050SetupTests,SetGetMsSetsCorrectly)
{
	Mpu6050Flex_SetGetMs(Mpu6050,MockMpu6050_GetMs);

	TEST_ASSERT_POINTERS_EQUAL(MockMpu6050_GetMs,Mpu6050Flex_GetGetMs(Mpu6050));
}
///----------------------------------------------------------------------------
/**
 * @brief Test group referring to general Mpu6050 functions
 */
TEST_GROUP(Mpu6050Tests);

TEST_SETUP(Mpu6050Tests)
{
	MockMpu6050IO_Create(20);
	Mpu6050 = Mpu6050Flex_Create();

	Mpu6050Flex_SetIORead(Mpu6050,MockMpu6050IO_ReadAndReturn);
	Mpu6050Flex_SetIOWrite(Mpu6050,MockMpu6050IO_Write);
	Mpu6050Flex_SetDelay(Mpu6050,MockMpu6050_Delay);
	Mpu6050Flex_SetGetMs(Mpu6050,MockMpu6050_GetMs);
}

TEST_TEAR_DOWN(Mpu6050Tests)
{
	MockMpu6050IO_VerifyComplete();
	MockMpu6050IO_Destroy();
	Mpu6050Flex_Destroy(Mpu6050);
}
/**
 * @brief Tests that Mpu6050Flex_WhoAmI follows correct sequence of interactions with the mpu6050 device
 */
TEST(Mpu6050Tests,ReadWhoAmIFollowsSequence)
{
	uint8_t Output = 0x68;
	MockMpu6050IO_ExpectReadAndReturn(0x75,1,&Output);

	TEST_ASSERT_EQUAL_UINT8(0x68,Mpu6050Flex_WhoAmI(Mpu6050));
}
/**
 * @brief Tests that Mpu6050Flex_ConfigSampleRateDivider follows correct sequence of interactions with the mpu6050 device
 */
TEST(Mpu6050Tests,ConfigureSampleRateDividerFollowsSequence)
{
	uint8_t Input = 0x00;

	MockMpu6050IO_ExpectWrite(0x19,1,&Input);

	TEST_ASSERT_EQUAL(MPU6050FLEX_SUCCESS,Mpu6050Flex_ConfigSampleRateDivider(Mpu6050,0x00));
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

	TEST_ASSERT_EQUAL(MPU6050FLEX_SUCCESS,Mpu6050Flex_ConfigDigitalLowPassFilter(Mpu6050,MPU6050FLEX_DLPF_CFG2));
}
/**
 * @brief Tests that using Mpu6050Flex_ConfigDigitalLowPassFilter with bad arguments return an error status
 */
TEST(Mpu6050Tests,ConfigureDigitalLowPassFilterRejectsBadArgs)
{
	TEST_ASSERT_EQUAL(MPU6050FLEX_FAILURE,Mpu6050Flex_ConfigDigitalLowPassFilter(Mpu6050,0x10));
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

	TEST_ASSERT_EQUAL(MPU6050FLEX_SUCCESS,Mpu6050Flex_ConfigGyroFullScaleRange(Mpu6050,MPU6050FLEX_GYRO_FS_SEL_500));
	TEST_ASSERT_EQUAL(66,Mpu6050Flex_GetGyroScale(Mpu6050));
}
/**
 * @brief Tests that using Mpu6050Flex_ConfigGyroFullScaleRange with bad arguments return an error status
 */
TEST(Mpu6050Tests,ConfigureGyroFullScaleRangeRejectsBadArgs)
{
	TEST_ASSERT_EQUAL(MPU6050FLEX_FAILURE,Mpu6050Flex_ConfigGyroFullScaleRange(Mpu6050,0x20));
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

	TEST_ASSERT_EQUAL(MPU6050FLEX_SUCCESS,Mpu6050Flex_ConfigAccFullScaleRange(Mpu6050,MPU6050FLEX_ACC_FS_SEL_8));
	TEST_ASSERT_EQUAL(4096,Mpu6050Flex_GetAccScale(Mpu6050));
}
/**
 * @brief Tests that using Mpu6050Flex_ConfigAccFullScaleRange with bad arguments return an error status
 */
TEST(Mpu6050Tests,ConfigureAccFullScaleRangeRejectsBadArgs)
{
	TEST_ASSERT_EQUAL(MPU6050FLEX_FAILURE,Mpu6050Flex_ConfigAccFullScaleRange(Mpu6050,0x20));
}
/**
 * @brief Tests that Mpu6050Flex_GetRawAccelData follows correct sequence of interactions with the mpu6050 device
 */
TEST(Mpu6050Tests,GetRawAccDataFollowsCorrectOrderAndReturnsExpectedData)
{

	Mpu6050Flex_ImuData_t ExpectedAccData;
	Mpu6050Flex_ImuData_t ActualAccData;

	SetupDataFunctionExpectations(&ExpectedAccData,0x3B);
	ActualAccData = Mpu6050Flex_GetRawAccelData(Mpu6050);

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
	ActualGyroData = Mpu6050Flex_GetRawGyroData(Mpu6050);

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

	TEST_ASSERT_EQUAL(MPU6050FLEX_SUCCESS,Mpu6050Flex_SetComplementaryFilterCoeffs(Mpu6050,TestGyroCoeff, TestAccCoeff));
	TEST_ASSERT_EQUAL_FLOAT(TestGyroCoeff,Mpu6050Flex_GetGyroCFCoeff(Mpu6050));
	TEST_ASSERT_EQUAL_FLOAT(TestAccCoeff,Mpu6050Flex_GetAccCFCoeff(Mpu6050));
}
/**
 * @brief Tests that using Mpu6050Flex_SetComplementaryFilterCoeffs with bad arguments return an error status
 */
TEST(Mpu6050Tests,SetupComplementaryRejectsBadArgs)
{
	float TestGyroCoeff = 0.91;
	float TestAccCoeff = 0.12;

	TEST_ASSERT_EQUAL(MPU6050FLEX_FAILURE,Mpu6050Flex_SetComplementaryFilterCoeffs(Mpu6050,TestGyroCoeff, TestAccCoeff));
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

	TEST_ASSERT_EQUAL(MPU6050FLEX_SUCCESS,Mpu6050Flex_Sleep(Mpu6050));
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

	TEST_ASSERT_EQUAL(MPU6050FLEX_SUCCESS,Mpu6050Flex_WakeUp(Mpu6050));
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
	Mpu6050 = Mpu6050Flex_Create();

	Mpu6050Flex_SetIORead(Mpu6050,MockMpu6050IO_ReadAndReturn);
	Mpu6050Flex_SetIOWrite(Mpu6050,MockMpu6050IO_Write);
	Mpu6050Flex_SetDelay(Mpu6050,MockMpu6050_Delay);
	Mpu6050Flex_SetGetMs(Mpu6050,MockMpu6050_GetMs);

	SetupCalibrationExpectations();
	Mpu6050Flex_Calibrate(Mpu6050);

}

TEST_TEAR_DOWN(Mpu6050CalibratedTests)
{
	MockMpu6050IO_VerifyComplete();
	MockMpu6050IO_Destroy();
	Mpu6050Flex_Destroy(Mpu6050);
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
		.GyroData.DataX 	= CalibOffsetData[0],
		.GyroData.DataY 	= CalibOffsetData[1],
		.GyroData.DataZ 	= CalibOffsetData[2],
		.AccData.DataX 		= CalibOffsetData[3],
		.AccData.DataY 		= CalibOffsetData[4],
		.AccData.DataZ 		= CalibOffsetData[5],
	};

	ActualDataStruct = Mpu6050Flex_GetImuDataOffsets(Mpu6050);

	TEST_ASSERT(CompareDataStructs(&ExpectedDataStruct.GyroData,&ActualDataStruct.GyroData));
	TEST_ASSERT(CompareDataStructs(&ExpectedDataStruct.AccData,&ActualDataStruct.AccData));

}
/**
 * @brief Test that Mpu6050Flex_Calibrate will write the last gyro read time (set in expectations)
 * in the LastGyroReadTime static variable.
 */
TEST(Mpu6050CalibratedTests,CalibrateSetsLastGyroReadTime)
{
	TEST_ASSERT_EQUAL(25,Mpu6050Flex_GetLastGyroReadTime(Mpu6050));
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

	ActualGyroData = Mpu6050Flex_GetGyroData(Mpu6050);

	printf("Output Gyro Data: [0x%.4x][0x%.4x][0x%.4x]\n",ActualGyroData.DataX,ActualGyroData.DataY,ActualGyroData.DataZ);
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

	ActualAccData = Mpu6050Flex_GetAccelData(Mpu6050);

	printf("Output Acc Data: [0x%.4x][0x%.4x][0x%.4x]\n",ActualAccData.DataX,ActualAccData.DataY,ActualAccData.DataZ);
	TEST_ASSERT(CompareDataStructs(&ExpectedAccData,&ActualAccData));

}


TEST(Mpu6050CalibratedTests,GetEulerFollowsCorrectOrderAndReturnsExpectedData)
{

	/*Expected read of 6 data bytes*/
	MockMpu6050IO_ExpectReadAndReturn(0x43,6,DataOut);
	MockMpu6050IO_ExpectReadAndReturn(0x3B,6,DataOut);

	//test that returned euler angles are correct for data out set in the beginning of this file
	//considering last gyro time and last known attitude are set during calibration

}

//TEST: get euler sets last known attitude and last gyro read time


