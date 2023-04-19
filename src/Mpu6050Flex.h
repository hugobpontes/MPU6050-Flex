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
/**
 * @brief Enum containing possible return values for Mpu6050Flex functions
 */
typedef enum
{
    MPU6050FLEX_SUCCESS,
	MPU6050FLEX_FAILURE,
} MPU6050Flex_Status_t;
/**
 * @brief Enum containing valid digital low pass filter configuration options
 */
typedef enum
{
	/*Refer to mpu6050 register map to see what these options are about*/
    MPU6050FLEX_DLPF_CFG0 = 0x00,
	MPU6050FLEX_DLPF_CFG1 = 0x01,
	MPU6050FLEX_DLPF_CFG2 = 0x02,
	MPU6050FLEX_DLPF_CFG3 = 0x03,
	MPU6050FLEX_DLPF_CFG4 = 0x04,
	MPU6050FLEX_DLPF_CFG5 = 0x05,
	MPU6050FLEX_DLPF_CFG6 = 0x06,
	MPU6050FLEX_DLPF_CFG7 = 0x07,
} MPU6050Flex_DLPF_Options_t;
/**
 * @brief Enum containing containing valid gyro full scall range options
 */
typedef enum
{
    MPU6050FLEX_GYRO_FS_SEL_250 = 0x00,
	MPU6050FLEX_GYRO_FS_SEL_500 = 0x08,
	MPU6050FLEX_GYRO_FS_SEL_1000 = 0x10,
	MPU6050FLEX_GYRO_FS_SEL_2000 = 0x18,
} MPU6050Flex_GYRO_FS_SEL_Options_t;
/**
 * @brief Enum containing containing valid accelerometer full scall range options
 */
typedef enum
{
    MPU6050FLEX_ACC_FS_SEL_2 = 0x00,
	MPU6050FLEX_ACC_FS_SEL_4 = 0x08,
	MPU6050FLEX_ACC_FS_SEL_8 = 0x10,
	MPU6050FLEX_ACC_FS_SEL_16 = 0x18,
} MPU6050Flex_ACC_FS_SEL_Options_t;

/**
 * @brief Several defines that define the masks for each parameter that can be set in the register that doesn't
 * 		  occupy the full register (in that case mask 0xFF is implied)
 */
#define MPU6050FLEX_SMPLRT_DIV_MSK 0xFF
#define MPU6050FLEX_DLPF_CFG_MSK 0x07
#define MPU6050FLEX_FS_SEL_MSK 0x18
#define MPU6050FLEX_AFS_SEL_MSK 0x18

/**
 * @brief Enum containing all mpu6050 register addresses that the Mpu6050Flex library interacts with
 */
typedef enum
{
    REG_WHO_AM_I = 0x75,
	REG_SMPRT_DIV = 0x19,
	REG_CONFIG = 0x1A,
	REG_GYRO_CONFIG = 0x1B,
	REG_ACCEL_CONFIG = 0x1C,
	REG_PWR_MGMT_1 = 0x6B,
	REG_ACCEL_XOUT_H = 0x3B,
	REG_GYRO_XOUT_H = 0x43,
} Mpu6050Register_t;

/**
 * @brief A type representing the signature that IO functions should have.
 */
typedef IOStatus_t (*IOFunc_t)(uint8_t,uint32_t,uint8_t*);
/**
 * @brief A type representing the signature that delay functions should have.
 */
typedef void (*DelayFunc_t)(uint32_t);

/**
 * @brief Struct containing generic 16 bit IMU data readings for 3 axes
 */
typedef struct Mpu6050Flex_Data
{
	uint16_t DataX;
	uint16_t DataY;
	uint16_t DataZ;
}Mpu6050Flex_ImuData_t;
/**
 * @brief Struct containing generic 32 bit IMU data readings for 3 axes
 * 32 bit structs are necessary so several 16 bit structs can be added together without overflowing when computing averages
 */
typedef struct Mpu6050Flex_Data32
{
	uint32_t DataX;
	uint32_t DataY;
	uint32_t DataZ;
}Mpu6050Flex_Data32_t;
/**
 * @brief Struct containing all IMU data readings provided by MPU6050 (gyro+accel) for 3 axes in 16 bit format
 */
typedef struct Mpu6050Flex_FullImuData
{
	Mpu6050Flex_ImuData_t GyroData;
	Mpu6050Flex_ImuData_t AccData;
}Mpu6050Flex_FullImuData_t;
/**
 * @brief Struct containing all IMU data readings provided by MPU6050 (gyro+accel) for 3 axes in 32 bit format
 * 32 bit structs are necessary so several 16 bit structs can be added together without overflowing when computing averages
 */
typedef struct Mpu6050Flex_FullImuData32
{
	Mpu6050Flex_Data32_t GyroData;
	Mpu6050Flex_Data32_t AccData;
}Mpu6050Flex_FullImuData32_t;


void Mpu6050Flex_SetIOWrite(IOFunc_t pWriteFunc);
void Mpu6050Flex_SetIORead(IOFunc_t pReadFunc);
void Mpu6050Flex_SetDelay(DelayFunc_t pDelay);
IOFunc_t Mpu6050Flex_GetIOWrite();
IOFunc_t Mpu6050Flex_GetIORead();
DelayFunc_t Mpu6050Flex_GetDelay();
uint8_t Mpu6050Flex_WhoAmI();
MPU6050Flex_Status_t Mpu6050Flex_ConfigSampleRateDivider(uint8_t Division);
MPU6050Flex_Status_t Mpu6050Flex_ConfigDigitalLowPassFilter(MPU6050Flex_DLPF_Options_t ConfigOption);
MPU6050Flex_Status_t Mpu6050Flex_ConfigGyroFullScaleRange(MPU6050Flex_GYRO_FS_SEL_Options_t ConfigOption);
MPU6050Flex_Status_t Mpu6050Flex_ConfigAccFullScaleRange(MPU6050Flex_ACC_FS_SEL_Options_t ConfigOption);
MPU6050Flex_Status_t Mpu6050Flex_WakeUp();
Mpu6050Flex_ImuData_t Mpu6050Flex_GetRawAccelData();
Mpu6050Flex_ImuData_t Mpu6050Flex_GetRawGyroData();
Mpu6050Flex_FullImuData_t Mpu6050Flex_GetImuDataOffsets();
MPU6050Flex_Status_t Mpu6050Flex_Calibrate();
Mpu6050Flex_ImuData_t Mpu6050Flex_GetAccelData();
Mpu6050Flex_ImuData_t Mpu6050Flex_GetGyroData();

#endif /* MPU6050FLEX_H_ */

