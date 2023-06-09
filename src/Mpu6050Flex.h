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
 * @brief Enum containing possible configurations of the SLEEP parameter in the POWER_MGMT1 register
 */
typedef enum
{
	/*Refer to mpu6050 register map to see what these options are about*/
	MPU6050FLEX_SLEEP_WAKE = 0x00,
    MPU6050FLEX_SLEEP_SLEEP = 0x40,
} MPU6050Flex_SLEEP_Options_t;

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
 * occupy the full register (in that case mask 0xFF is implied)
 */
#define MPU6050FLEX_SMPLRT_DIV_MSK 0xFF
#define MPU6050FLEX_DLPF_CFG_MSK 0x07
#define MPU6050FLEX_SLEEP_MSK 0x40
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
 * @param1 Address of register to be read/written
 * @param2 Number of bytes to read/write
 * @param3 Memory location where read/write data is stored/read from
 * @return Status code indicating success of operation
 */
typedef IOStatus_t (*IOFunc_t)(uint8_t,uint32_t,uint8_t*);
/**
 * @brief A type representing the signature that delay functions should have.
 * @param1 Amount of milliseconds to wait
 *
 */
typedef void (*DelayFunc_t)(uint32_t);
/**
 * @brief A type representing the signature that get milliseconds functions should have.
 * @return milliseconds since start of execution
 */
typedef uint32_t(*GetMsFunc_t)();

/**
 * @brief Struct containing generic signed 16 bit IMU data readings for 3 axes
 */
typedef struct Mpu6050Flex_ImuData
{
	int16_t DataX;
	int16_t DataY;
	int16_t DataZ;
}Mpu6050Flex_ImuData_t;

/**
 * @brief Struct containing generic signed 16 bit IMU data readings for 3 axes
 */
typedef struct Mpu6050Flex_FloatData
{
	float FloatDataX;
	float FloatDataY;
	float FloatDataZ;
}Mpu6050Flex_ImuFloatData_t;

/**
 * @brief Struct containing generic signed 32 bit IMU data readings for 3 axes
 * 32 bit structs are necessary for holding accumulated 16 bit readings that would overflow otherwise
 */
typedef struct Mpu6050Flex_ImuData32
{
	int32_t DataX;
	int32_t DataY;
	int32_t DataZ;
}Mpu6050Flex_ImuData32_t;
/**
 * @brief Struct containing all IMU data readings provided by MPU6050 (gyro+accel) for 3 axes in signed 16 bit format
 */
typedef struct Mpu6050Flex_FullImuData
{
	Mpu6050Flex_ImuData_t GyroData;
	Mpu6050Flex_ImuData_t AccData;
}Mpu6050Flex_FullImuData_t;
/**
 * @brief Struct containing all IMU data readings provided by MPU6050 (gyro+accel) for 3 axes in signed 32 bit format
 * 32 bit structs are necessary so several 16 bit structs can be added together without overflowing when computing averages
 */
typedef struct Mpu6050Flex_FullImuData32
{
	Mpu6050Flex_ImuData32_t GyroData;
	Mpu6050Flex_ImuData32_t AccData;
}Mpu6050Flex_FullImuData32_t;
/**
 * @brief Struct containing the three float Euler angles
 */
typedef struct Mpu6050Flex_EulerAngles
{
	float Roll;
	float Pitch;
	float Yaw;
}Mpu6050Flex_EulerAngles_t;

/**
 * @brief Abstract data type that acts as a handle to an Mpu6050Flex instance
 */
typedef struct Mpu6050FlexStruct* Mpu6050Flex_t;

Mpu6050Flex_t Mpu6050Flex_Create();
void Mpu6050Flex_Destroy(Mpu6050Flex_t Mpu6050Flex);
void Mpu6050Flex_SetIOWrite(Mpu6050Flex_t Mpu6050Flex,IOFunc_t pWriteFunc);
void Mpu6050Flex_SetIORead(Mpu6050Flex_t Mpu6050Flex,IOFunc_t pReadFunc);
void Mpu6050Flex_SetDelay(Mpu6050Flex_t Mpu6050Flex,DelayFunc_t pDelay);
void Mpu6050Flex_SetGetMs(Mpu6050Flex_t Mpu6050Flex,GetMsFunc_t pGetMs);
IOFunc_t Mpu6050Flex_GetIOWrite(Mpu6050Flex_t Mpu6050Flex);
IOFunc_t Mpu6050Flex_GetIORead(Mpu6050Flex_t Mpu6050Flex);
DelayFunc_t Mpu6050Flex_GetDelay(Mpu6050Flex_t Mpu6050Flex);
GetMsFunc_t Mpu6050Flex_GetGetMs(Mpu6050Flex_t Mpu6050Flex);
uint8_t Mpu6050Flex_WhoAmI(Mpu6050Flex_t Mpu6050Flex);
MPU6050Flex_Status_t Mpu6050Flex_ConfigSampleRateDivider(Mpu6050Flex_t Mpu6050Flex,uint8_t Division);
MPU6050Flex_Status_t Mpu6050Flex_ConfigDigitalLowPassFilter(Mpu6050Flex_t Mpu6050Flex,MPU6050Flex_DLPF_Options_t ConfigOption);
MPU6050Flex_Status_t Mpu6050Flex_ConfigGyroFullScaleRange(Mpu6050Flex_t Mpu6050Flex,MPU6050Flex_GYRO_FS_SEL_Options_t ConfigOption);
MPU6050Flex_Status_t Mpu6050Flex_ConfigAccFullScaleRange(Mpu6050Flex_t Mpu6050Flex,MPU6050Flex_ACC_FS_SEL_Options_t ConfigOption);
Mpu6050Flex_ImuData_t Mpu6050Flex_GetRawAccelData(Mpu6050Flex_t Mpu6050Flex);
Mpu6050Flex_ImuData_t Mpu6050Flex_GetRawGyroData(Mpu6050Flex_t Mpu6050Flex);
Mpu6050Flex_FullImuData_t Mpu6050Flex_GetImuDataOffsets(Mpu6050Flex_t Mpu6050Flex);
MPU6050Flex_Status_t Mpu6050Flex_Calibrate(Mpu6050Flex_t Mpu6050Flex);
Mpu6050Flex_ImuData_t Mpu6050Flex_GetAccelData(Mpu6050Flex_t Mpu6050Flex);
Mpu6050Flex_ImuData_t Mpu6050Flex_GetGyroData(Mpu6050Flex_t Mpu6050Flex);
float Mpu6050Flex_GetAccScale(Mpu6050Flex_t Mpu6050Flex);
float Mpu6050Flex_GetGyroScale(Mpu6050Flex_t Mpu6050Flex);
MPU6050Flex_Status_t Mpu6050Flex_SetComplementaryFilterCoeffs(Mpu6050Flex_t Mpu6050Flex,float GyroCoeff, float AccCoeff);
float Mpu6050Flex_GetGyroCFCoeff(Mpu6050Flex_t Mpu6050Flex);
float Mpu6050Flex_GetAccCFCoeff(Mpu6050Flex_t Mpu6050Flex);
MPU6050Flex_Status_t Mpu6050Flex_Sleep(Mpu6050Flex_t Mpu6050Flex);
MPU6050Flex_Status_t Mpu6050Flex_WakeUp(Mpu6050Flex_t Mpu6050Flex);
uint32_t Mpu6050Flex_GetLastKnownAttitudeTime(Mpu6050Flex_t Mpu6050Flex);
Mpu6050Flex_EulerAngles_t Mpu6050Flex_GetLastKnownAttitude(Mpu6050Flex_t Mpu6050Flex);
Mpu6050Flex_EulerAngles_t Mpu6050Flex_GetEuler(Mpu6050Flex_t Mpu6050Flex);

#endif /* MPU6050FLEX_H_ */

