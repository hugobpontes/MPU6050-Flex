/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include "Mpu6050Flex.h"

/*Function to be provided that writes a register address and writes a user-defined number of bytes
 * to I2C being used by MPU6050 from user defined memory location
 * Also return IO Success code as defined in Mpu6050Flex.h*/
static IOStatus_t RpiPico_I2C_Write(uint8_t Address,uint32_t Size,uint8_t* DataPtr)
{
    IOStatus_t Status = IO_SUCCESS;
    uint8_t WrittenBytes;

	uint8_t* WritePointer = (uint8_t*) malloc(Size+1);
	memcpy(WritePointer,&Address,1);
	memcpy(WritePointer+1,DataPtr,Size);

	WrittenBytes = i2c_write_blocking(i2c_default, MPU6050_I2C_ADDRESS, WritePointer, Size+1, false);

	free(WritePointer);

	if (WrittenBytes != Size+1)
	{
		Status = IO_FAILURE;
	}
	return Status;
}

/*Function to be provided that delays execution by user-defined ms */
static void RpiPico_Delay(uint32_t ms)
{
	sleep_ms(ms);
}

/*Function to be provided that obtains ms since start of execution */
static uint32_t RpiPico_GetMs()
{
	return time_us_32()/1000;
}

/*Function to be provided that writes a register address and reads a user-defined number of bytes
 * from I2C being used by MPU6050 into user defined memory location
 * Also return IO Success code as defined in Mpu6050Flex.h*/
static IOStatus_t RpiPico_I2C_Read(uint8_t Address,uint32_t Size,uint8_t* DataPtr)
{
	IOStatus_t Status = IO_SUCCESS;
	uint8_t WrittenBytes;
    uint8_t ReadBytes;

    WrittenBytes = i2c_write_blocking(i2c_default, MPU6050_I2C_ADDRESS, &Address,1, false);
	sleep_ms(1);
	if (WrittenBytes == 1)
	{
		ReadBytes = i2c_read_blocking(i2c_default, MPU6050_I2C_ADDRESS, DataPtr, Size, false);
		if (ReadBytes != Size)
		{
			Status = IO_FAILURE;
		}
	}
	return Status;
}

static void RpiPicoConfigureDefaultI2C()
{
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));
}

int main() {

    stdio_init_all();
    RpiPicoConfigureDefaultI2C();
   

    /*Declare Mpu6050 instance handle*/
	Mpu6050Flex_t Mpu6050;

	/*Allocate and create a Mpu6050 instance/object at the location pointed by Mpu6050*/
	Mpu6050 = Mpu6050Flex_Create();

	printf("----------Welcome to Mpu6050 Flex----------\n");

	/*Setup abstract functions that Mpu6050Flex uses and are tightly coupled to the underlying hardware*/
	Mpu6050Flex_SetIOWrite(Mpu6050,RpiPico_I2C_Write);
	Mpu6050Flex_SetIORead(Mpu6050,RpiPico_I2C_Read);
	Mpu6050Flex_SetGetMs(Mpu6050,RpiPico_GetMs);
	Mpu6050Flex_SetDelay(Mpu6050,RpiPico_Delay);

	printf("Waking up...\n");
	Mpu6050Flex_WakeUp(Mpu6050);
    sleep_ms(100); //As recommended, sleep a bit (>30ms) after waking up before starting operation

	/*Verify that there is a responsive MPU6050, if not abort this program*/
	uint8_t ID;
	ID = Mpu6050Flex_WhoAmI(Mpu6050);

	if (ID == MPU6050_I2C_ADDRESS)
	{
		printf("MPU6050 Found!\n");
	}
	else
	{
		printf("ERROR! MPU6050 Not Found!\n");
		assert(0);
	}

	/*Default configurations for mpu6050*/
	printf("Configuring...\n");
	Mpu6050Flex_SetComplementaryFilterCoeffs(Mpu6050,0.98, 0.02);
	Mpu6050Flex_ConfigSampleRateDivider(Mpu6050,0);
	Mpu6050Flex_ConfigDigitalLowPassFilter(Mpu6050,MPU6050FLEX_DLPF_CFG0);
	Mpu6050Flex_ConfigGyroFullScaleRange(Mpu6050,MPU6050FLEX_GYRO_FS_SEL_250);
	Mpu6050Flex_ConfigAccFullScaleRange(Mpu6050,MPU6050FLEX_ACC_FS_SEL_2);

	printf("Calibrating...\n");
	Mpu6050Flex_Calibrate(Mpu6050);
	printf("Ready!!!\n");

	/*Get and print attitude every 100 ms*/
	while (1)
	{
		Mpu6050Flex_EulerAngles_t RetEuler;
		RetEuler = Mpu6050Flex_GetEuler(Mpu6050);
		printf("Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n",RetEuler.Roll,RetEuler.Pitch,RetEuler.Yaw);

		sleep_ms(100);
	}

	/*Will never be reached but this is used to destroy Mpu6050 object/instance */
	Mpu6050Flex_Destroy(Mpu6050);

}
