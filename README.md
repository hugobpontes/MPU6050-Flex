# Mpu6050Flex
  
The MPU6050Flex is a library for the MPU6050 sensor, which is a commonly used device for measuring acceleration, angular velocity, and temperature. What sets this library apart is that it is completely abstracted for any underlying hardware, as it was designed to provide a flexible and easy-to-use interface for interacting with the MPU6050 sensor.
MPU6050Flex is built with a focus on flexibility, meaning it can be easily used on a variety of platforms such as Stm32, Arduino, Raspberry Pi, and other microcontrollers. The provided examples cover **STM32F76ZI** and **Raspberry Pi Pico.** 
MPU6050Flex is written purely in C.
# Features
The key features that Mpu6050Flex provides are
 - Letting the client set platform specific IO functions with ease
 - Allows the instantiation of multiple Mpu6050Flex "objects", so multiple mpu6050 can be used by the same client.
 - Allows basic MPU6050 configuration
 - Allows configuration of soft parameters used for attitude computation
 - Lets the client obtain raw accelerometer and gyro data
 - Lets the client obtain calibrated accelerometer and gyro data
 - Lets the client obtain Euler angles based on accelerometer and gyro data
 - The code is well documented for every function
 - The unit test suite is provided

The key API is as shown:

    Mpu6050Flex_Create();
    ### Create instance of Mpu6050 "Object"
    Mpu6050Flex_Destroy();
    ### Destroy instance of Mpu6050 "Object"
    Mpu6050Flex_SetIOWrite();
    ### Sets the client defined function for writing to the MPU6050
    Mpu6050Flex_SetIORead();
    ### Sets the client defined function for reading from the MPU6050
    Mpu6050Flex_SetDelay();
    ### Sets the client defined function for delay a given amount of ms
    Mpu6050Flex_SetGetMs();
    ### Sets the client defined function for getting timestamp
    Mpu6050Flex_WhoAmI();
    ### Requests I2C address of the mpu6050 to confirm its responsiveness
    Mpu6050Flex_ConfigSampleRateDivider();
    ### Configures the mpu6050 sample rate divider
    Mpu6050Flex_ConfigDigitalLowPassFilter();
    ### Configures the mpu6050 Digital Low Pass Filter
    Mpu6050Flex_ConfigGyroFullScaleRange();
    ### Configures the mpu6050 gyro full scale range
    Mpu6050Flex_ConfigAccFullScaleRange();
    ### Configures the mpu6050 accelerometer full scale range
    Mpu6050Flex_GetRawAccelData();
    ### Returns accelerometer raw data
    Mpu6050Flex_GetRawGyroData();
    ### Returns gyro raw data
    Mpu6050Flex_Calibrate();
    ### Generates internal calibration parameters for both acc and gyro
    Mpu6050Flex_GetAccelData(Mpu6050Flex_t Mpu6050Flex);
    ### Returns accelerometer calibrated data
    Mpu6050Flex_GetGyroData(Mpu6050Flex_t Mpu6050Flex);
    ### Returns gyro calibrated data
    Mpu6050Flex_SetComplementaryFilterCoeffs();
    ### Sets complementary filter coefficients used to compute attitude
    Mpu6050Flex_Sleep();
    ### Puts MPU6050 in sleep mode
    Mpu6050Flex_WakeUp();
    ### Puts MPU6050 off sleep mode
    Mpu6050Flex_GetEuler();
    ### Return three Euler angles representing mpu6050's attitude

MPU6050Flex was initially built for self-learning purposes and when such goals were achieved there was no point in it covering the full features that the MPU6050 offers. As a result only basic configuration, obtainal of gyro and accelerometer and attitude computation are provided. If this library gains any traction I will gladly add any missing features so that it can be used to expand it to cover the complete MPU6050 functionality. Additionally, pull requests are welcome.
# Contents
This library contains the following files in the following folders

 - **src**
	 - Mpu6050Flex.c
	 - Mpu6050Flex.h
 - **example**
	 - **stm32f767zi**
		 - stm32f767zi_example.c
	 - **rpi pico**
		 - rpi_pico_example.c
 - **tests**
	 - all_tests.c
	 - Mpu6050_MockIO.c
	 - Mpu6050_MockIO.h
	 - Mpu6050_Runner.c
	 - Mpu6050_Tests.c

### src
The source files are where the library is actually implented and what you would import to your applications.
### example
In this library two usage example showing how to import and use this library are provided: one for the stm32f767zi and another for the raspberry pi pico microcontrollers.
### tests
This library's development followed Test Driven Development using the Unity unit test harnesss - the test files are provided. The MockIO files define the mock used to easily test the hardware interface without depending and allowing one to control its output. In the Tests file the tests are defined.
# Usage
The usage of this library is simple. To use it, simply include the Mpu6050Flex.h file and then follow the steps described here:

 - Create an Mpu6050Flex instance using `Mpu6050Flex_Create();`
 - Set all four hardware dependent functions using`Mpu6050Flex_SetIOWrite() #Mpu6050Flex_SetIORead() Mpu6050Flex_SetDelay() Mpu6050Flex_SetGetMs();`
 - Wake up MPU6050 using `Mpu6050Flex_WakeUp()`
 - Delay for at least 30 ms
 -   **(optional)** Verifiy that the MPU6050 is reponsive using `Mpu6050Flex_WakeUp()` 
 - **(optional)** Configure MPU6050 and Mpu6050Flex using `Mpu6050Flex_SetComplementaryFilterCoeffs(); Mpu6050Flex_ConfigSampleRateDivider(); Mpu6050Flex_ConfigDigitalLowPassFilter(); Mpu6050Flex_ConfigGyroFullScaleRange(); Mpu6050Flex_ConfigAccFullScaleRange();`
 - **(If you only wish to obtain raw data the next steps aren't necessary)**
 - Calibrate using `Mpu6050Flex_Calibrate()`keeping the IMU in a resting position for 3 seconds (default duration, this can be edited in the .h file)
 - Get Calibrated or Euler angle data using `Mpu6050Flex_GetAccelData() Mpu6050Flex_GetGyroData()   Mpu6050Flex_GetEuler();`
## Limitations
Limitations of this library are regarding the lack of ability to use some the MPU6050 features such as its fifo or temperature reading. As mentioned earlier these will be included if this library gains any traction or I get some feedback.
## To-do
 - Increase features to cover more of what MPU6050 offers
 - Test more intensively to cover all off-nominal conditions (things like bad calibration or off nominal attitude conditions aren't covered).
## Contributing
Pull requests and any sort of feedback/requests are welcome.
## Contact
You can reach me at hpontes9@gmail.com