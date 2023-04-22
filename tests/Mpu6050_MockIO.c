/*
 * Mpu6050_Mock.c
 *
 *  Created on: 10/04/2023
 *      Author: Utilizador
 */

#include <stdint.h>
#include "unity_fixture.h"
#include <string.h>
#include "Mpu6050Flex.h"


#define MAX_MPU6050_BURST_SIZE 20 //move this somewhere else, mpu6050.h? where and check for out of bounds should be inside write/read func

typedef enum
{
    WRITE_ID,
	READ_ID,
	DELAY_ID,
	GETMS_ID,
} MockFuncId_t;

typedef struct Expectations
{
  uint32_t FuncId;
  uint8_t Address;
  uint8_t Data[MAX_MPU6050_BURST_SIZE];
  uint32_t Size;
  uint32_t ReturnMs;
} Expectations_t;

static Expectations_t* ExpectationsArray = 0;
static Expectations_t CurrentFunctionCall;
static uint32_t SetExpectationsIdx;
static uint32_t GetExpectationsIdx;
static uint32_t MaxExpectationsIdx;


static void FailIfExpectationsArrayNotInitialized()
{
	if (!ExpectationsArray)
	{
		FAIL("Expectations Array Not Initialized");
	}
}
static void FailIfNoUnusedExpectations()
{
	if (GetExpectationsIdx >= SetExpectationsIdx)
	{
		FAIL("No Unused Expectations");
	}
}

static void FailIfNoRoomForExpectations()
{
	if (SetExpectationsIdx >= MaxExpectationsIdx)
	{
		FAIL("No Room for Expectations");
	}
}

static void FailIfUnexpectedData()
{
	uint8_t idx;

	for (idx = 0; idx<CurrentFunctionCall.Size; idx++)
	{
		if (CurrentFunctionCall.Data[idx] != ExpectationsArray[GetExpectationsIdx].Data[idx])
		{
			printf( "Expected 0x%.2x, Got 0x%.2x \n",ExpectationsArray[GetExpectationsIdx].Data[idx],CurrentFunctionCall.Data[idx]);
			FAIL("Unexpected Data argument");
		}
	}

}

static void FailIfUnexpectedSize()
{
	if (CurrentFunctionCall.Size != ExpectationsArray[GetExpectationsIdx].Size)
	{
		FAIL("Unexpected Size argument");
	}
}

static void FailIfUnexpectedAddress()
{
	if (CurrentFunctionCall.Address != ExpectationsArray[GetExpectationsIdx].Address)
	{
		FAIL("Unexpected Address argument");
	}
}

static void FailIfUnexpectedFunc()
{
	if (CurrentFunctionCall.FuncId != ExpectationsArray[GetExpectationsIdx].FuncId)
	{
		FAIL("Unexpected function call");
	}
}
static void FailIfNotAllExpectationsUsed()
{
	if (GetExpectationsIdx < SetExpectationsIdx)
	{
		FAIL("Not all Expectations Used");
	}
}


void MockMpu6050IO_Create (uint32_t MaxExpectations)
{
	ExpectationsArray = calloc(MaxExpectations,sizeof(Expectations_t));

	SetExpectationsIdx = 0;
	GetExpectationsIdx = 0;
	MaxExpectationsIdx = MaxExpectations;
}
void MockMpu6050IO_Destroy()
{
	if (ExpectationsArray)
	{
		free(ExpectationsArray);
	}
	ExpectationsArray = 0;
}

static void MockStandardFailures()
{
	FailIfExpectationsArrayNotInitialized();
	FailIfNoUnusedExpectations();
	FailIfUnexpectedFunc();
}

void MockMpu6050IO_ExpectWrite(uint8_t Address,uint32_t Size, uint8_t* DataPtr)
{
	uint8_t idx;

	FailIfExpectationsArrayNotInitialized();
	FailIfNoRoomForExpectations();
	ExpectationsArray[SetExpectationsIdx].FuncId = WRITE_ID;
	ExpectationsArray[SetExpectationsIdx].Size = Size;
	for (idx = 0;idx<Size;idx++)
	{
		ExpectationsArray[SetExpectationsIdx].Data[idx] = DataPtr[idx];
	}

	ExpectationsArray[SetExpectationsIdx].Address = Address;

	SetExpectationsIdx++;
}

void MockMpu6050IO_ExpectReadAndReturn(uint8_t Address,uint32_t Size, uint8_t* DataPtr)
{
	uint8_t idx;

	FailIfExpectationsArrayNotInitialized();
	FailIfNoRoomForExpectations();
	ExpectationsArray[SetExpectationsIdx].FuncId = READ_ID;
	ExpectationsArray[SetExpectationsIdx].Size = Size;
	for (idx = 0;idx<Size;idx++)
	{
		ExpectationsArray[SetExpectationsIdx].Data[idx] = DataPtr[idx];
	}

	ExpectationsArray[SetExpectationsIdx].Address = Address;

	SetExpectationsIdx++;
}

void MockMpu6050IO_ExpectGetMsAndReturn(uint32_t Return)
{
	FailIfExpectationsArrayNotInitialized();
	FailIfNoRoomForExpectations();
	ExpectationsArray[SetExpectationsIdx].FuncId = GETMS_ID;
	ExpectationsArray[SetExpectationsIdx].ReturnMs = Return;

	SetExpectationsIdx++;
}

uint32_t MockMpu6050_GetMs()
{
	CurrentFunctionCall.FuncId = GETMS_ID;
	MockStandardFailures();

	GetExpectationsIdx++;

	return ExpectationsArray[GetExpectationsIdx-1].ReturnMs;
}

IOStatus_t MockMpu6050IO_Write(uint8_t Address,uint32_t Size,uint8_t* DataPtr)
{
	uint8_t idx;

	CurrentFunctionCall.FuncId = WRITE_ID;
	CurrentFunctionCall.Address = Address;
	CurrentFunctionCall.Size = Size;

	for (idx = 0;idx<Size;idx++)
	{
		CurrentFunctionCall.Data[idx] = DataPtr[idx];
	}


	MockStandardFailures();

	FailIfUnexpectedSize();
	FailIfUnexpectedData();
	FailIfUnexpectedAddress();


	GetExpectationsIdx++;

	return IO_SUCCESS;
}


IOStatus_t MockMpu6050IO_ReadAndReturn(uint8_t Address,uint32_t Size,uint8_t* DataPtr)
{
	uint8_t idx;

	CurrentFunctionCall.FuncId = READ_ID;
	CurrentFunctionCall.Address = Address;
	CurrentFunctionCall.Size = Size;

	MockStandardFailures();

	FailIfUnexpectedSize();
	FailIfUnexpectedAddress();

	memcpy(DataPtr,ExpectationsArray[GetExpectationsIdx].Data,Size);

	GetExpectationsIdx++;

	return IO_SUCCESS;
}

void MockMpu6050_Delay(uint32_t ms)
{
	/*ideally i'd create a new expectation array for delay type
	 *like in write and read functions and check it here
	 */
}

void MockMpu6050IO_VerifyComplete()
{
	FailIfNotAllExpectationsUsed();
}
