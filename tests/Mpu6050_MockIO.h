/*
 * Mpu6050_Mock.h
 *
 *  Created on: 10/04/2023
 *      Author: Utilizador
 */

#ifndef MPU6050_MOCKIO_H_
#define MPU6050_MOCKIO_H_

#include <stdint.h>
#include <Mpu6050Flex.h>

void MockMpu6050IO_Create (uint32_t MaxExpectations);
void MockMpu6050IO_Destroy();
void MockMpu6050IO_ExpectWrite(uint8_t Address,uint32_t Size,uint8_t* DataPtr);
IOStatus_t MockMpu6050IO_Write(uint8_t Address,uint32_t Size,uint8_t* DataPtr);
void MockMpu6050IO_VerifyComplete();
void MockMpu6050IO_ExpectReadAndReturn(uint8_t Address,uint32_t Size, uint8_t* DataPtr);
IOStatus_t MockMpu6050IO_ReadAndReturn(uint8_t Address,uint32_t Size,uint8_t* DataPtr);
void MockMpu6050_Delay(uint32_t ms);
uint32_t MockMpu6050_GetMs();
void MockMpu6050IO_ExpectGetMsAndReturn(uint32_t Return);


#endif /* MPU6050_MOCKIO_H_ */
