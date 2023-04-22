#include "Mpu6050Flex.h"
#include "unity.h"
#include "unity_fixture.h"

#include "Mpu6050_MockIO.h"
#include <stdbool.h>

static bool CompareDataStructs(Mpu6050Flex_ImuRawData_t* pA, Mpu6050Flex_ImuRawData_t* pB)
{
	bool ret = true;

	if (pA->RawDataX != pB->RawDataX)
	{
		ret = false;
	}
	else
	{
		if (pA->RawDataY != pB->RawDataY)
		{
				ret = false;
		}
		else
		{
			if (pA->RawDataZ != pB->RawDataZ)
			{
				ret = false;
			}
		}
	}


	if (ret == false)
	{
		printf("\n A: DataX = 0x%.2x, DataY = 0x%.2x, DataZ = 0x%.2x \n",pA->RawDataX,pA->RawDataY,pA->RawDataZ);
		printf("\n B: DataX = 0x%.2x, DataY = 0x%.2x, DataZ = 0x%.2x \n",pB->RawDataX,pB->RawDataY,pB->RawDataZ);
	}

	return ret;
}

static void SetupParameterUpdateExpectations(uint8_t ExpReadValue,
										uint8_t ParamMask,
										uint8_t ConfigOption,
										uint8_t RegisterAddress)
{

	MockMpu6050IO_ExpectReadAndReturn(RegisterAddress,1,&ExpReadValue);

	uint8_t ExpWrite = (ExpReadValue & (~ParamMask)) | ConfigOption;

	MockMpu6050IO_ExpectWrite(RegisterAddress,1,&ExpWrite);
}

static void SetupDataFunctionExpectations(Mpu6050Flex_ImuRawData_t* pExpData, uint8_t DataReg)
{
	uint8_t DataOut[6] = {0xBB,0xAA,0xDD,0xCC,0xFF,0xEE};

	MockMpu6050IO_ExpectReadAndReturn(DataReg,6,DataOut);

	pExpData->RawDataX = 0xAABB;
	pExpData->RawDataY = 0xCCDD;
	pExpData->RawDataZ = 0xEEFF;
}

static void SetupCalibratedDataFunctionExpectations(Mpu6050Flex_ImuRawData_t* pExpData, uint8_t DataReg)
{
	uint8_t DataOut[6] = {0xBB,0xAA,0xDD,0xCC,0xFF,0xEE};

	MockMpu6050IO_ExpectReadAndReturn(DataReg,6,DataOut);

	Mpu6050Flex_FullImuRawData_t ImuDataOffset;

	ImuDataOffset = Mpu6050Flex_GetImuDataOffsets();

	if (DataReg == REG_GYRO_XOUT_H)
	{
		pExpData->RawDataX = 0xAABB - ImuDataOffset.GyroData.RawDataX;
		pExpData->RawDataY = 0xCCDD - ImuDataOffset.GyroData.RawDataY;
		pExpData->RawDataZ = 0xEEFF - ImuDataOffset.GyroData.RawDataZ;
	}
	else if (REG_ACCEL_XOUT_H)
	{
		pExpData->RawDataX = 0xAABB - ImuDataOffset.AccData.RawDataX;
		pExpData->RawDataY = 0xCCDD - ImuDataOffset.AccData.RawDataY;
		pExpData->RawDataZ = 0xEEFF - ImuDataOffset.AccData.RawDataZ;
	}
}


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

TEST(Mpu6050SetupTests,SetWriteIOSetsCorrectly)
{
	Mpu6050Flex_SetIOWrite(MockMpu6050IO_Write);

	TEST_ASSERT_POINTERS_EQUAL(MockMpu6050IO_Write,Mpu6050Flex_GetIOWrite());
}

TEST(Mpu6050SetupTests,SetReadIOSetsCorrectly)
{
	Mpu6050Flex_SetIORead(MockMpu6050IO_ReadAndReturn);

	TEST_ASSERT_POINTERS_EQUAL(MockMpu6050IO_ReadAndReturn,Mpu6050Flex_GetIORead());
}

TEST(Mpu6050SetupTests,SetDelaySetsCorrectly)
{
	Mpu6050Flex_SetDelay(MockMpu6050_Delay);

	TEST_ASSERT_POINTERS_EQUAL(MockMpu6050_Delay,Mpu6050Flex_GetDelay());
}

///----------------------------------------------------------------------------

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

TEST(Mpu6050Tests,ReadWhoAmIFollowsSequence)
{
	uint8_t Output = 0x68;
	MockMpu6050IO_ExpectReadAndReturn(0x75,1,&Output);

	TEST_ASSERT_EQUAL_UINT8(0x68,Mpu6050Flex_WhoAmI());
}

TEST(Mpu6050Tests,ConfigureSampleRateDividerFollowsSequence)
{
	uint8_t Input = 0x00;

	MockMpu6050IO_ExpectWrite(0x19,1,&Input);

	TEST_ASSERT_EQUAL(MPU6050FLEX_SUCCESS,Mpu6050Flex_ConfigSampleRateDivider(0x00));
}

TEST(Mpu6050Tests,ConfigureDigitalLowPassFilterFollowsSequenceAndDoesntOverwrite)
{

	uint8_t ConfigOption = MPU6050FLEX_DLPF_CFG2;
	uint8_t ExpReadValue = 0x10;
	uint8_t ParamMask = MPU6050FLEX_DLPF_CFG_MSK;
	uint8_t ParamReg = 0x1A;

	SetupParameterUpdateExpectations(ExpReadValue,ParamMask,ConfigOption,ParamReg);

	TEST_ASSERT_EQUAL(MPU6050FLEX_SUCCESS,Mpu6050Flex_ConfigDigitalLowPassFilter(MPU6050FLEX_DLPF_CFG2));
}
TEST(Mpu6050Tests,ConfigureDigitalLowPassFilterRejectsBadArgs)
{
	TEST_ASSERT_EQUAL(MPU6050FLEX_FAILURE,Mpu6050Flex_ConfigDigitalLowPassFilter(0x10));
}

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

TEST(Mpu6050Tests,ConfigureGyroFullScaleRangeRejectsBadArgs)
{
	TEST_ASSERT_EQUAL(MPU6050FLEX_FAILURE,Mpu6050Flex_ConfigGyroFullScaleRange(0x20));
}

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

TEST(Mpu6050Tests,ConfigureAccFullScaleRangeRejectsBadArgs)
{
	TEST_ASSERT_EQUAL(MPU6050FLEX_FAILURE,Mpu6050Flex_ConfigAccFullScaleRange(0x20));
}

TEST(Mpu6050Tests,GetRawAccDataFollowsCorrectOrderAndReturnsExpectedData)
{

	Mpu6050Flex_ImuRawData_t ExpectedRawAccData;
	Mpu6050Flex_ImuRawData_t ActualRawAccData;

	SetupDataFunctionExpectations(&ExpectedRawAccData,0x3B);
	ActualRawAccData = Mpu6050Flex_GetRawAccelData();

	TEST_ASSERT(CompareDataStructs(&ExpectedRawAccData,&ActualRawAccData));
}

TEST(Mpu6050Tests,GetRawGyroDataFollowsCorrectOrderAndReturnsExpectedData)
{

	Mpu6050Flex_ImuRawData_t ExpectedGyroRawData;
	Mpu6050Flex_ImuRawData_t ActualGyroRawData;

	SetupDataFunctionExpectations(&ExpectedGyroRawData,0x43);
	ActualGyroRawData = Mpu6050Flex_GetRawGyroData();

	TEST_ASSERT(CompareDataStructs(&ExpectedGyroRawData,&ActualGyroRawData));
}

TEST(Mpu6050Tests,SetupComplementaryFilterSetsCF)
{
	float TestGyroCoeff = 0.98;
	float TestAccCoeff = 0.02;

	TEST_ASSERT_EQUAL(MPU6050FLEX_SUCCESS,Mpu6050Flex_SetComplementaryFilterCoeffs(TestGyroCoeff, TestAccCoeff));
	TEST_ASSERT_EQUAL_FLOAT(TestGyroCoeff,Mpu6050Flex_GetGyroCFCoeff());
	TEST_ASSERT_EQUAL_FLOAT(TestAccCoeff,Mpu6050Flex_GetAccCFCoeff());
}

TEST(Mpu6050Tests,SetupComplementaryRejectsBadArgs)
{
	float TestGyroCoeff = 0.91;
	float TestAccCoeff = 0.12;

	TEST_ASSERT_EQUAL(MPU6050FLEX_FAILURE,Mpu6050Flex_SetComplementaryFilterCoeffs(TestGyroCoeff, TestAccCoeff));
}

TEST(Mpu6050Tests,SleepFollowsSequenceAndDoesntOverwrite)
{

	uint8_t ConfigOption = MPU6050FLEX_SLEEP_SLEEP;
	uint8_t ExpReadValue = 0x10;
	uint8_t ParamMask = MPU6050FLEX_SLEEP_MSK;
	uint8_t ParamReg = 0x6B;

	SetupParameterUpdateExpectations(ExpReadValue,ParamMask,ConfigOption,ParamReg);

	TEST_ASSERT_EQUAL(MPU6050FLEX_SUCCESS,Mpu6050Flex_Sleep());
}

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
TEST_GROUP(Mpu6050CalibratedTests);

TEST_SETUP(Mpu6050CalibratedTests)
{
	MockMpu6050IO_Create(20);
	Mpu6050Flex_SetIORead(MockMpu6050IO_ReadAndReturn);
	Mpu6050Flex_SetIOWrite(MockMpu6050IO_Write);
	Mpu6050Flex_SetDelay(MockMpu6050_Delay);
	Mpu6050Flex_SetGetMs(MockMpu6050_GetMs);
}

TEST_TEAR_DOWN(Mpu6050CalibratedTests)
{
	MockMpu6050IO_VerifyComplete();
	MockMpu6050IO_Destroy();
}


TEST(Mpu6050CalibratedTests,CalibrateWritesAverageReadDataInInternalStructure)
{

	Mpu6050Flex_FullImuRawData_t ActualDataStruct;
	Mpu6050Flex_FullImuRawData_t ExpectedDataStruct =
	{
		.GyroData.RawDataX 	= 0x7800,
		.GyroData.RawDataY 	= 0x7100,
		.GyroData.RawDataZ 	= 0x7E05,
		.AccData.RawDataX 	= 0x7100,
		.AccData.RawDataY 	= 0x7300,
		.AccData.RawDataZ 	= 0x7F01-0x4000,

	};

	Mpu6050Flex_GetImuDataOffsets();

	ActualDataStruct = Mpu6050Flex_GetImuDataOffsets();

	TEST_ASSERT(CompareDataStructs(&ExpectedDataStruct.GyroData,&ActualDataStruct.GyroData));
	TEST_ASSERT(CompareDataStructs(&ExpectedDataStruct.AccData,&ActualDataStruct.AccData));

}

TEST(Mpu6050CalibratedTests,CalibrateSetsLastGyroReadTime)
{
	TEST_ASSERT_EQUAL(25,Mpu6050Flex_GetLastGyroReadTime());
}


TEST(Mpu6050CalibratedTests,GetGyroDataFollowsCorrectOrderAndReturnsExpectedData)
{
	Mpu6050Flex_ImuRawData_t ExpectedGyroData;
	Mpu6050Flex_ImuRawData_t ActualGyroData;

	SetupCalibratedDataFunctionExpectations(&ExpectedGyroData,0x43);

	ActualGyroData = Mpu6050Flex_GetGyroData();

	TEST_ASSERT(CompareDataStructs(&ExpectedGyroData,&ActualGyroData));

}

TEST(Mpu6050CalibratedTests,GetAccDataFollowsCorrectOrderAndReturnsExpectedData)
{
	Mpu6050Flex_ImuRawData_t ExpectedAccData;
	Mpu6050Flex_ImuRawData_t ActualAccData;

	SetupCalibratedDataFunctionExpectations(&ExpectedAccData,0x3B);

	ActualAccData = Mpu6050Flex_GetAccelData();

	TEST_ASSERT(CompareDataStructs(&ExpectedAccData,&ActualAccData));

}


//Test for accel get euler
//test for gyro get euler -> remember, this takes into account past measurements (which include acceleromter contributions)
//test for get euler (calls other two and computes euler based on coeffs)

