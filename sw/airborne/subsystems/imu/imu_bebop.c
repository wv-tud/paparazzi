/*
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file subsystems/imu/imu_bebop.c
 * Driver for the Bebop (2) magnetometer, accelerometer and gyroscope
 */

#include "subsystems/imu.h"
#include "subsystems/abi.h"
#include "mcu_periph/i2c.h"
#include "subsystems/imu/imu_bebop.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>


/* defaults suitable for Bebop */
#ifndef BEBOP_MAG_I2C_DEV
#define BEBOP_MAG_I2C_DEV i2c1
#endif
PRINT_CONFIG_VAR(BEBOP_MAG_I2C_DEV)

#ifndef BEBOP_MPU_I2C_DEV
#define BEBOP_MPU_I2C_DEV i2c2
#endif
PRINT_CONFIG_VAR(BEBOP_MPU_I2C_DEV)

#if !defined BEBOP_LOWPASS_FILTER && !defined  BEBOP_SMPLRT_DIV
#if (PERIODIC_FREQUENCY == 60) || (PERIODIC_FREQUENCY == 120)
/* Accelerometer: Bandwidth 44Hz, Delay 4.9ms
 * Gyroscope: Bandwidth 42Hz, Delay 4.8ms sampling 1kHz
 */
#define BEBOP_LOWPASS_FILTER MPU60X0_DLPF_42HZ
#define BEBOP_SMPLRT_DIV 9
PRINT_CONFIG_MSG("Gyro/Accel output rate is 100Hz at 1kHz internal sampling")
#elif PERIODIC_FREQUENCY == 512
/* Accelerometer: Bandwidth 260Hz, Delay 0ms
 * Gyroscope: Bandwidth 256Hz, Delay 0.98ms sampling 8kHz
 */
#define BEBOP_LOWPASS_FILTER MPU60X0_DLPF_256HZ
#define BEBOP_SMPLRT_DIV 1
PRINT_CONFIG_MSG("Gyro/Accel output rate is 1kHz at 8kHz internal sampling")
#endif
#endif
PRINT_CONFIG_VAR(BEBOP_SMPLRT_DIV)
PRINT_CONFIG_VAR(BEBOP_LOWPASS_FILTER)

PRINT_CONFIG_VAR(BEBOP_GYRO_RANGE)
PRINT_CONFIG_VAR(BEBOP_ACCEL_RANGE)

float imu_bebop_pitch_offset = 8.5;

#ifndef BEBOP_FACTORY_CALIB
#define BEBOP_FACTORY_CALIB 0
#endif
PRINT_CONFIG_VAR(BEBOP_MPU_I2C_DEV)
bool imu_bebop_factory_calib = BEBOP_FACTORY_CALIB;

struct OrientationReps imu_to_mag_bebop;    ///< IMU to magneto rotation
double gyro_x_sfe =1.0, gyro_y_sfe =1.0, gyro_z_sfe =1.0;
double accel_x_sfe =1.0, accel_y_sfe =1.0, accel_z_sfe =1.0;
double gyro_x_bias =0.0, gyro_y_bias =0.0, gyro_z_bias =0.0;
double accel_x_bias =0.0, accel_y_bias =0.0, accel_z_bias =0.0;
/** Basic Navstik IMU data */
struct ImuBebop imu_bebop;

const char* imu_bebop_config_get_field(char* line, int num);
const char* imu_bebop_config_get_field(char* line, int num)
{
    const char* tok;
    for (tok = strtok(line, " ");
            tok && *tok;
            tok = strtok(NULL, " \n"))
    {
        if (!--num)
            return tok;
    }
    return NULL;
}

void imu_bebop_read_config(void);
void imu_bebop_read_config(void)
{
  printf("Reading bebop factory calibration:\n\n");
  FILE* stream = fopen("/factory/Thermal_IMU_ortho.fact.csv", "r");
  if(stream != NULL){
    char line[1024];
    uint8_t i = 0;
    while (fgets(line, 1024, stream))
    {
      char* tmp = strdup(line);
      switch(i)
      {
        case 0:
          gyro_x_sfe = strtod(imu_bebop_config_get_field(tmp, 3), NULL);
          break;
        case 1:
          gyro_y_sfe = strtod(imu_bebop_config_get_field(tmp, 3), NULL);
          break;
        case 2:
          gyro_z_sfe = strtod(imu_bebop_config_get_field(tmp, 3), NULL);
          break;
        case 9:
          accel_x_sfe = strtod(imu_bebop_config_get_field(tmp, 3), NULL);
          break;
        case 10:
          accel_y_sfe = strtod(imu_bebop_config_get_field(tmp, 3), NULL);
          break;
        case 11:
          accel_z_sfe = strtod(imu_bebop_config_get_field(tmp, 3), NULL);
          break;
      }
      free(tmp);
      i++;
    }
    printf("gyro  sfe  x: %0.10f  y: %0.10f  z: %0.10f\n", gyro_x_sfe, gyro_y_sfe, gyro_z_sfe);
    printf("accel sfe  x: %0.10f  y: %0.10f  z: %0.10f\n", accel_x_sfe, accel_y_sfe, accel_z_sfe);
  }else{
    printf("Unable to read /factory/Thermal_IMU_ortho.fact.csv\n");
  }
  stream = fopen("/factory/Thermal_IMU_bias.fact.csv", "r");
  if(stream != NULL){
    char line[1024];
    uint8_t i = 0;
    while (fgets(line, 1024, stream))
    {
      char* tmp = strdup(line);
      switch(i)
      {
        case 1:
          gyro_x_bias = strtod(imu_bebop_config_get_field(tmp, 3), NULL);
          imu.gyro_neutral.p = (int32_t) round(RATE_BFP_OF_REAL(gyro_x_bias) * IMU_GYRO_P_SIGN * IMU_GYRO_P_SENS_DEN / ((float) IMU_GYRO_P_SENS_NUM));
          break;
        case 2:
          gyro_y_bias = strtod(imu_bebop_config_get_field(tmp, 3), NULL);
          imu.gyro_neutral.q = (int32_t) round(RATE_BFP_OF_REAL(gyro_y_bias) * IMU_GYRO_Q_SIGN * IMU_GYRO_Q_SENS_DEN / ((float) IMU_GYRO_Q_SENS_NUM));
          break;
        case 3:
          gyro_z_bias = strtod(imu_bebop_config_get_field(tmp, 3), NULL);
          imu.gyro_neutral.r = (int32_t) round(RATE_BFP_OF_REAL(gyro_z_bias) * IMU_GYRO_R_SIGN * IMU_GYRO_R_SENS_DEN / ((float) IMU_GYRO_R_SENS_NUM));
          break;
        case 4:
          accel_x_bias = strtod(imu_bebop_config_get_field(tmp, 3), NULL);
          imu.accel_neutral.x = (int32_t) round(ACCEL_BFP_OF_REAL(accel_x_bias) * IMU_ACCEL_X_SIGN * IMU_ACCEL_X_SENS_DEN / ((float) IMU_ACCEL_X_SENS_NUM));
          break;
        case 5:
          accel_y_bias = strtod(imu_bebop_config_get_field(tmp, 3), NULL);
          imu.accel_neutral.y = (int32_t) round(ACCEL_BFP_OF_REAL(accel_y_bias) * IMU_ACCEL_Y_SIGN * IMU_ACCEL_Y_SENS_DEN / ((float) IMU_ACCEL_Y_SENS_NUM));
          break;
        case 6:
          accel_z_bias = strtod(imu_bebop_config_get_field(tmp, 3), NULL);
          imu.accel_neutral.z = (int32_t) round(ACCEL_BFP_OF_REAL(accel_z_bias) * IMU_ACCEL_Z_SIGN * IMU_ACCEL_Z_SENS_DEN / ((float) IMU_ACCEL_Z_SENS_NUM));
          break;
      }
      free(tmp);
      i++;
    }
    printf("gyro  bias x: %0.10f %d  y: %0.10f %d  z: %0.10f %d\n", gyro_x_bias, imu.gyro_neutral.p, gyro_y_bias, imu.gyro_neutral.q, gyro_z_bias, imu.gyro_neutral.r);
    printf("accel bias x: %0.10f %d  y: %0.10f %d  z: %0.10f %d\n", accel_x_bias, imu.accel_neutral.x, accel_y_bias, imu.accel_neutral.y, accel_z_bias, imu.accel_neutral.z);
  }else{
    printf("Unable to read /factory/Thermal_IMU_bias.fact.csv\n");
  }
}

/**
 * Navstik IMU initializtion of the MPU-60x0 and HMC58xx
 */
void imu_bebop_init(void)
{
  if(imu_bebop_factory_calib){
    imu_bebop_read_config();
  }
  /* MPU-60X0 */
  mpu60x0_i2c_init(&imu_bebop.mpu, &(BEBOP_MPU_I2C_DEV), MPU60X0_ADDR);
  imu_bebop.mpu.config.smplrt_div = BEBOP_SMPLRT_DIV;
  imu_bebop.mpu.config.dlpf_cfg = BEBOP_LOWPASS_FILTER;
  imu_bebop.mpu.config.gyro_range = BEBOP_GYRO_RANGE;
  imu_bebop.mpu.config.accel_range = BEBOP_ACCEL_RANGE;

  /* AKM8963 */
  ak8963_init(&imu_bebop.ak, &(BEBOP_MAG_I2C_DEV), AK8963_ADDR);

#if BEBOP_VERSION2
  //the magnetometer of the bebop2 is located on the gps board,
  //which is under a slight angle
  struct FloatEulers imu_to_mag_eulers =
  { 0.0, RadOfDeg(imu_bebop_pitch_offset), 0.0 };
  orientationSetEulers_f(&imu_to_mag_bebop, &imu_to_mag_eulers);
#endif
}

/**
 * Handle all the periodic tasks of the Navstik IMU components.
 * Read the MPU60x0 every periodic call and the HMC58XX every 10th call.
 */
void imu_bebop_periodic(void)
{
  struct FloatEulers imu_to_mag_eulers =
  { 0.0, RadOfDeg(imu_bebop_pitch_offset), 0.0 };
  orientationSetEulers_f(&imu_to_mag_bebop, &imu_to_mag_eulers);

  // Start reading the latest gyroscope data
  mpu60x0_i2c_periodic(&imu_bebop.mpu);

  // AKM8963
  ak8963_periodic(&imu_bebop.ak);
}

/**
 * Handle all the events of the Navstik IMU components.
 * When there is data available convert it to the correct axis and save it in the imu structure.
 */
void imu_bebop_event(void)
{
  uint32_t now_ts = get_sys_time_usec();

  /* MPU-60x0 event taks */
  mpu60x0_i2c_event(&imu_bebop.mpu);

  if (imu_bebop.mpu.data_available) {
    /* default orientation of the MPU is upside down sor corrigate this here */
    RATES_ASSIGN(imu.gyro_unscaled, imu_bebop.mpu.data_rates.rates.p, -imu_bebop.mpu.data_rates.rates.q,
                 -imu_bebop.mpu.data_rates.rates.r);
    VECT3_ASSIGN(imu.accel_unscaled, imu_bebop.mpu.data_accel.vect.x, -imu_bebop.mpu.data_accel.vect.y,
                 -imu_bebop.mpu.data_accel.vect.z);

    imu_bebop.mpu.data_available = false;
    imu_scale_gyro(&imu);
    if(imu_bebop_factory_calib){
      imu.gyro.p = (int32_t) round(imu.gyro.p * gyro_x_sfe);
      imu.gyro.q = (int32_t) round(imu.gyro.q * gyro_y_sfe);
      imu.gyro.r = (int32_t) round(imu.gyro.r * gyro_z_sfe);
    }else{
      imu.gyro_neutral.p = 0;
      imu.gyro_neutral.q = 0;
      imu.gyro_neutral.r = 0;
    }
    imu_scale_accel(&imu);
    if(imu_bebop_factory_calib){
      imu.accel.x = (int32_t) round(imu.accel.x * accel_x_sfe);
      imu.accel.y = (int32_t) round(imu.accel.y * accel_y_sfe);
      imu.accel.z = (int32_t) round(imu.accel.z * accel_z_sfe);
    }else{
      imu.accel_neutral.x = 0;
      imu.accel_neutral.y = 0;
      imu.accel_neutral.z = 0;
    }
    AbiSendMsgIMU_GYRO_INT32(IMU_BOARD_ID, now_ts, &imu.gyro);
    AbiSendMsgIMU_ACCEL_INT32(IMU_BOARD_ID, now_ts, &imu.accel);
  }

  /* AKM8963 event task */
  ak8963_event(&imu_bebop.ak);

  if (imu_bebop.ak.data_available) {
#if BEBOP_VERSION2
    struct Int32Vect3 mag_temp;
    // In the second bebop version the magneto is turned 90 degrees
    VECT3_ASSIGN(mag_temp, -imu_bebop.ak.data.vect.x, -imu_bebop.ak.data.vect.y, imu_bebop.ak.data.vect.z);
    // Rotate the magneto
    struct Int32RMat *imu_to_mag_rmat = orientationGetRMat_i(&imu_to_mag_bebop);
    int32_rmat_vmult(&imu.mag_unscaled, imu_to_mag_rmat, &mag_temp);
#else //BEBOP regular first verion
    VECT3_ASSIGN(imu.mag_unscaled, -imu_bebop.ak.data.vect.y, imu_bebop.ak.data.vect.x, imu_bebop.ak.data.vect.z);
#endif

    imu_bebop.ak.data_available = false;
    imu_scale_mag(&imu);
    AbiSendMsgIMU_MAG_INT32(IMU_BOARD_ID, now_ts, &imu.mag);
  }
}
