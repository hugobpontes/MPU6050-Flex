/*
 * all_tests.c
 *
 *  Created on: Apr 2, 2023
 *      Authoasr: sUtilizador
 */

#include "unity_fixture.h"

static void RunAllTests(void)
{
  RUN_TEST_GROUP(Mpu6050SetupTests);
  RUN_TEST_GROUP(Mpu6050Tests);
  RUN_TEST_GROUP(Mpu6050CalibratedTests);
}

int main(int argc, const char * argv[])
{
  return UnityMain(argc, argv, RunAllTests);
}
