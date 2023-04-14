/**
 ********************************************************************************
 * @file    Mpu6050Flex.h
 * @author  Hugo Pontes
 * @date    12/04/2023
 * @brief   Flexible mpu6050 that can be used on any microcontroller
 ********************************************************************************
 */

#ifndef MPU6050FLEX_H_
#define MPU6050FLEX_H_

#include <stdint.h>

/*I2C Address of the MPU6050*/
#define MPU6050_I2C_ADDRESS 0x68


/**
 * @brief Enum containing possible return values for IO functions
 */
typedef enum
{
    IO_SUCCESS,
	IO_FAILURE,
} IOStatus_t;

typedef enum
{
    MPU6050FLEX_SUCCESS,
	MPU6050FLEX_FAILURE,
} MPU6050Flex_Status_t;

//refer to register map to see what these options are about
typedef enum
{
    MPU6050FLEX_DLPF_CFG0 = 0x00,
	MPU6050FLEX_DLPF_CFG1 = 0x01,
	MPU6050FLEX_DLPF_CFG2 = 0x02,
	MPU6050FLEX_DLPF_CFG3 = 0x03,
	MPU6050FLEX_DLPF_CFG4 = 0x04,
	MPU6050FLEX_DLPF_CFG5 = 0x05,
	MPU6050FLEX_DLPF_CFG6 = 0x06,
	MPU6050FLEX_DLPF_CFG7 = 0x07,
} MPU6050Flex_DLPF_Options_t;

typedef enum
{
    MPU6050FLEX_GYRO_FS_SEL_250 = 0x00,
	MPU6050FLEX_GYRO_FS_SEL_500 = 0x08,
	MPU6050FLEX_GYRO_FS_SEL_1000 = 0x10,
	MPU6050FLEX_GYRO_FS_SEL_2000 = 0x18,
} MPU6050Flex_GYRO_FS_SEL_Options_t;

typedef enum
{
    MPU6050FLEX_ACC_FS_SEL_2 = 0x00,
	MPU6050FLEX_ACC_FS_SEL_4 = 0x08,
	MPU6050FLEX_ACC_FS_SEL_8 = 0x10,
	MPU6050FLEX_ACC_FS_SEL_16 = 0x18,
} MPU6050Flex_ACC_FS_SEL_Options_t;

#define MPU6050FLEX_DLPF_CFG_MSK 0x07
#define MPU6050FLEX_GYRO_FS_SEL_MSK 0x18
#define MPU6050FLEX_ACC_FS_SEL_MSK 0x18

/**
 * @brief Enum containing all mpu6050 register addresses that the Mpu6050Flex library interacts with
 */
typedef enum
{
    REG_WHO_AM_I = 0x75,
	REG_SAMPLE_RATE_DIVIDER = 0x19,
	REG_CONFIGURATION = 0x1A,
	REG_GYRO_CONFIG = 0x1B,
	REG_ACCEL_CONFIG = 0x1C,
	REG_PWR_MGMT_1 = 0x6B,
} Mpu6050Register_t;

/**
 * @brief A type representing the signature that IO functions should have.
 */
typedef IOStatus_t (*IOFunc_t)(uint8_t,uint32_t,uint8_t*);

void Mpu6050Flex_SetIOWrite(IOFunc_t pWriteFunc);
void Mpu6050Flex_SetIORead(IOFunc_t pReadFunc);
IOFunc_t Mpu6050Flex_GetIOWrite();
IOFunc_t Mpu6050Flex_GetIORead();
uint8_t Mpu6050Flex_WhoAmI();
MPU6050Flex_Status_t Mpu6050Flex_ConfigSampleRateDivider(uint8_t Division);
MPU6050Flex_Status_t Mpu6050Flex_ConfigDigitalLowPassFilter(uint8_t ConfigOption);
MPU6050Flex_Status_t Mpu6050Flex_ConfigGyroFullScaleRange(uint8_t ConfigOption);
MPU6050Flex_Status_t Mpu6050Flex_ConfigAccFullScaleRange(uint8_t ConfigOption);
MPU6050Flex_Status_t Mpu6050Flex_WakeUp();

#endif /* MPU6050FLEX_H_ */

