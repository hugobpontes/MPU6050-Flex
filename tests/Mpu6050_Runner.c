#include "unity.h"
#include "unity_fixture.h"

TEST_GROUP_RUNNER(Mpu6050SetupTests)
{
  RUN_TEST_CASE(Mpu6050SetupTests, SetWriteIOSetsCorrectly);
  RUN_TEST_CASE(Mpu6050SetupTests, SetReadIOSetsCorrectly);
  RUN_TEST_CASE(Mpu6050SetupTests, SetDelaySetsCorrectly);
  RUN_TEST_CASE(Mpu6050SetupTests, SetGetMsSetsCorrectly);
  RUN_TEST_CASE(Mpu6050SetupTests, DefaultParametersArentEmpty);
}

TEST_GROUP_RUNNER(Mpu6050Tests)
{

  RUN_TEST_CASE(Mpu6050Tests, ReadWhoAmIFollowsSequence);
  RUN_TEST_CASE(Mpu6050Tests, ConfigureSampleRateDividerFollowsSequence);
  RUN_TEST_CASE(Mpu6050Tests, ConfigureDigitalLowPassFilterFollowsSequenceAndDoesntOverwrite);
  RUN_TEST_CASE(Mpu6050Tests, ConfigureDigitalLowPassFilterRejectsBadArgs);
  RUN_TEST_CASE(Mpu6050Tests, ConfigureGyroFullScaleRangeFollowsSequenceAndDoesntOverwrite);
  RUN_TEST_CASE(Mpu6050Tests, ConfigureGyroFullScaleRangeRejectsBadArgs);
  RUN_TEST_CASE(Mpu6050Tests, ConfigureAccFullScaleRangeFollowsSequenceAndDoesntOverwrite);
  RUN_TEST_CASE(Mpu6050Tests, ConfigureAccFullScaleRangeRejectsBadArgs);
  RUN_TEST_CASE(Mpu6050Tests, GetRawAccDataFollowsCorrectOrderAndReturnsExpectedData);
  RUN_TEST_CASE(Mpu6050Tests, GetRawGyroDataFollowsCorrectOrderAndReturnsExpectedData);
  RUN_TEST_CASE(Mpu6050Tests, SetupComplementaryFilterSetsCF);
  RUN_TEST_CASE(Mpu6050Tests, SetupComplementaryRejectsBadArgs);
  RUN_TEST_CASE(Mpu6050Tests, SleepFollowsSequenceAndDoesntOverwrite);
  RUN_TEST_CASE(Mpu6050Tests, WakeUpFollowsSequenceAndDoesntOverwrite);
}

TEST_GROUP_RUNNER(Mpu6050CalibratedTests)
{
	RUN_TEST_CASE(Mpu6050CalibratedTests, CalibrateWritesAverageReadDataInInternalStructure);
	RUN_TEST_CASE(Mpu6050CalibratedTests, CalibrateSetsLastGyroReadTime);
	RUN_TEST_CASE(Mpu6050CalibratedTests, GetGyroDataFollowsCorrectOrderAndReturnsExpectedData);
	RUN_TEST_CASE(Mpu6050CalibratedTests, GetAccDataFollowsCorrectOrderAndReturnsExpectedData);
}

TEST_GROUP_RUNNER(Mpu6050AttitudeTests)
{
	RUN_TEST_CASE(Mpu6050AttitudeTests, GetEulerFollowsCorrectOrderAndReturnsExpectedData);
}
