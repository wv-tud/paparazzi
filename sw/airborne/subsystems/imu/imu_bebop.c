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

#define BEBOP_MPU_TEMP_AVG_CNT 100
#define BEBOP_MPU_TEMP_FILT_COEF 1024

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

float imu_bebop_pitch_offset = 10.0;

#ifndef BEBOP_FACTORY_CALIB
#define BEBOP_FACTORY_CALIB 0
#endif
PRINT_CONFIG_VAR(BEBOP_FACTORY_CALIB)
bool imu_bebop_factory_calib = BEBOP_FACTORY_CALIB;

struct OrientationReps imu_to_mag_bebop;    ///< IMU to magneto rotation
double gyro_x_sfe =1.0, gyro_y_sfe =1.0, gyro_z_sfe =1.0;
double accel_x_sfe =1.0, accel_y_sfe =1.0, accel_z_sfe =1.0;
double gyro_x_bias =0.0, gyro_y_bias =0.0, gyro_z_bias =0.0;
double gyro_x_bias_t1 =0.0, gyro_y_bias_t1 =0.0, gyro_z_bias_t1 =0.0;
double gyro_x_bias_t2 =0.0, gyro_y_bias_t2 =0.0, gyro_z_bias_t2 =0.0;
double gyro_x_bias_t3 =0.0, gyro_y_bias_t3 =0.0, gyro_z_bias_t3 =0.0;
double accel_x_bias =0.0, accel_y_bias =0.0, accel_z_bias =0.0;
double accel_x_bias_t1 =0.0, accel_y_bias_t1 =0.0, accel_z_bias_t1 =0.0;
double accel_x_bias_t2 =0.0, accel_y_bias_t2 =0.0, accel_z_bias_t2 =0.0;
double accel_x_bias_t3 =0.0, accel_y_bias_t3 =0.0, accel_z_bias_t3 =0.0;
/** Basic Navstik IMU data */
struct ImuBebop imu_bebop;
float imu_bebop_filtered_temperature = 0;

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
  printf("Reading Parrot factory calibration...\n");
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
  }else{
    printf("ERROR: Unable to read /factory/Thermal_IMU_ortho.fact.csv\n");
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
          gyro_x_bias    = strtod(imu_bebop_config_get_field(tmp, 3), NULL);
          tmp = strdup(line);
          gyro_x_bias_t1 = strtod(imu_bebop_config_get_field(tmp, 4), NULL);
          tmp = strdup(line);
          gyro_x_bias_t2 = strtod(imu_bebop_config_get_field(tmp, 5), NULL);
          tmp = strdup(line);
          gyro_x_bias_t3 = strtod(imu_bebop_config_get_field(tmp, 6), NULL);
          break;
        case 2:
          gyro_y_bias    = strtod(imu_bebop_config_get_field(tmp, 3), NULL);
          tmp = strdup(line);
          gyro_y_bias_t1 = strtod(imu_bebop_config_get_field(tmp, 4), NULL);
          tmp = strdup(line);
          gyro_y_bias_t2 = strtod(imu_bebop_config_get_field(tmp, 5), NULL);
          tmp = strdup(line);
          gyro_y_bias_t3 = strtod(imu_bebop_config_get_field(tmp, 6), NULL);
          break;
        case 3:
          gyro_z_bias    = strtod(imu_bebop_config_get_field(tmp, 3), NULL);
          tmp = strdup(line);
          gyro_z_bias_t1 = strtod(imu_bebop_config_get_field(tmp, 4), NULL);
          tmp = strdup(line);
          gyro_z_bias_t2 = strtod(imu_bebop_config_get_field(tmp, 5), NULL);
          tmp = strdup(line);
          gyro_z_bias_t3 = strtod(imu_bebop_config_get_field(tmp, 6), NULL);
          break;
        case 4:
          accel_x_bias    = strtod(imu_bebop_config_get_field(tmp, 3), NULL);
          tmp = strdup(line);
          accel_x_bias_t1 = strtod(imu_bebop_config_get_field(tmp, 4), NULL);
          tmp = strdup(line);
          accel_x_bias_t2 = strtod(imu_bebop_config_get_field(tmp, 5), NULL);
          tmp = strdup(line);
          accel_x_bias_t3 = strtod(imu_bebop_config_get_field(tmp, 6), NULL);
          break;
        case 5:
          accel_y_bias    = strtod(imu_bebop_config_get_field(tmp, 3), NULL);
          tmp = strdup(line);
          accel_y_bias_t1 = strtod(imu_bebop_config_get_field(tmp, 4), NULL);
          tmp = strdup(line);
          accel_y_bias_t2 = strtod(imu_bebop_config_get_field(tmp, 5), NULL);
          tmp = strdup(line);
          accel_y_bias_t3 = strtod(imu_bebop_config_get_field(tmp, 6), NULL);
          break;
        case 6:
          accel_z_bias    = strtod(imu_bebop_config_get_field(tmp, 3), NULL);
          tmp = strdup(line);
          accel_z_bias_t1 = strtod(imu_bebop_config_get_field(tmp, 4), NULL);
          tmp = strdup(line);
          accel_z_bias_t2 = strtod(imu_bebop_config_get_field(tmp, 5), NULL);
          tmp = strdup(line);
          accel_z_bias_t3 = strtod(imu_bebop_config_get_field(tmp, 6), NULL);
          break;
      }
      free(tmp);
      i++;
    }
    printf("Gyroscope p    : (RAW - (%7.4f) - (%9.6f) * t - (%11.8f) * t^2 - (%13.10f) * t^3) / %7.4f\n", gyro_x_bias, gyro_x_bias_t1, gyro_x_bias_t2, gyro_x_bias_t3, gyro_x_sfe);
    printf("Gyroscope q    : (RAW - (%7.4f) - (%9.6f) * t - (%11.8f) * t^2 - (%13.10f) * t^3) / %7.4f\n", gyro_y_bias, gyro_y_bias_t1, gyro_y_bias_t2, gyro_y_bias_t3, gyro_y_sfe);
    printf("Gyroscope r    : (RAW - (%7.4f) - (%9.6f) * t - (%11.8f) * t^2 - (%13.10f) * t^3) / %7.4f\n", gyro_z_bias, gyro_z_bias_t1, gyro_z_bias_t2, gyro_z_bias_t3, gyro_z_sfe);
    printf("Accelerometer x: (RAW - (%7.4f) - (%9.6f) * t - (%11.8f) * t^2 - (%13.10f) * t^3) / %7.4f\n", accel_x_bias, accel_x_bias_t1, accel_x_bias_t2, accel_x_bias_t3, accel_x_sfe);
    printf("Accelerometer y: (RAW - (%7.4f) - (%9.6f) * t - (%11.8f) * t^2 - (%13.10f) * t^3) / %7.4f\n", accel_y_bias, accel_y_bias_t1, accel_y_bias_t2, accel_y_bias_t3, accel_y_sfe);
    printf("Accelerometer z: (RAW - (%7.4f) - (%9.6f) * t - (%11.8f) * t^2 - (%13.10f) * t^3) / %7.4f\n", accel_z_bias, accel_z_bias_t1, accel_z_bias_t2, accel_z_bias_t3, accel_z_sfe);
  }else{
    printf("ERROR: Unable to read /factory/Thermal_IMU_bias.fact.csv\n");
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
void imu_scale_gyro(struct Imu *_imu)
{
  RATES_COPY(_imu->gyro_prev, _imu->gyro);
  if(imu_bebop_factory_calib){
    float temp2 = imu_bebop_filtered_temperature * imu_bebop_filtered_temperature;
    float temp3 = temp2 * imu_bebop_filtered_temperature;
    int32_t gyro_TC_bias_x = (int32_t) round(RATE_BFP_OF_REAL(gyro_x_bias + imu_bebop_filtered_temperature * gyro_x_bias_t1 + temp2 * gyro_x_bias_t2 + temp3 * gyro_x_bias_t3) * IMU_GYRO_P_SIGN *
        IMU_GYRO_P_SENS_DEN / ((float) IMU_GYRO_P_SENS_NUM));
    int32_t gyro_TC_bias_y = (int32_t) round(RATE_BFP_OF_REAL(gyro_y_bias + imu_bebop_filtered_temperature * gyro_y_bias_t1 + temp2 * gyro_y_bias_t2 + temp3 * gyro_y_bias_t3) * IMU_GYRO_Q_SIGN *
        IMU_GYRO_Q_SENS_DEN / ((float) IMU_GYRO_Q_SENS_NUM));
    int32_t gyro_TC_bias_z = (int32_t) round(RATE_BFP_OF_REAL(gyro_z_bias + imu_bebop_filtered_temperature * gyro_z_bias_t1 + temp2 * gyro_z_bias_t2 + temp3 * gyro_z_bias_t3) * IMU_GYRO_R_SIGN *
        IMU_GYRO_R_SENS_DEN / ((float) IMU_GYRO_R_SENS_NUM));

    _imu->gyro.p = ((_imu->gyro_unscaled.p - _imu->gyro_neutral.p - gyro_TC_bias_x) * IMU_GYRO_P_SIGN *
        IMU_GYRO_P_SENS_NUM) / IMU_GYRO_P_SENS_DEN / gyro_x_sfe;
    _imu->gyro.q = ((_imu->gyro_unscaled.q - _imu->gyro_neutral.q - gyro_TC_bias_y) * IMU_GYRO_Q_SIGN *
        IMU_GYRO_Q_SENS_NUM) / IMU_GYRO_Q_SENS_DEN / gyro_y_sfe;
    _imu->gyro.r = ((_imu->gyro_unscaled.r - _imu->gyro_neutral.r - gyro_TC_bias_z) * IMU_GYRO_R_SIGN *
        IMU_GYRO_R_SENS_NUM) / IMU_GYRO_R_SENS_DEN / gyro_z_sfe;
  }else{
    _imu->gyro.p = ((_imu->gyro_unscaled.p - _imu->gyro_neutral.p) * IMU_GYRO_P_SIGN *
        IMU_GYRO_P_SENS_NUM) / IMU_GYRO_P_SENS_DEN;
    _imu->gyro.q = ((_imu->gyro_unscaled.q - _imu->gyro_neutral.q) * IMU_GYRO_Q_SIGN *
        IMU_GYRO_Q_SENS_NUM) / IMU_GYRO_Q_SENS_DEN;
    _imu->gyro.r = ((_imu->gyro_unscaled.r - _imu->gyro_neutral.r) * IMU_GYRO_R_SIGN *
        IMU_GYRO_R_SENS_NUM) / IMU_GYRO_R_SENS_DEN;
  }
}

void imu_scale_accel(struct Imu *_imu)
{
  VECT3_COPY(_imu->accel_prev, _imu->accel);
  if(imu_bebop_factory_calib){
    float temp2 = imu_bebop_filtered_temperature * imu_bebop_filtered_temperature;
    float temp3 = temp2 * imu_bebop_filtered_temperature;
    int32_t accel_TC_bias_x = (int32_t) round(ACCEL_BFP_OF_REAL(accel_x_bias + imu_bebop_filtered_temperature * accel_x_bias_t1 + temp2 * accel_x_bias_t2 + temp3 * accel_x_bias_t3) * IMU_ACCEL_X_SIGN *
        IMU_ACCEL_X_SENS_DEN / ((float) IMU_ACCEL_X_SENS_NUM));
    int32_t accel_TC_bias_y = (int32_t) round(ACCEL_BFP_OF_REAL(accel_y_bias + imu_bebop_filtered_temperature * accel_y_bias_t1 + temp2 * accel_y_bias_t2 + temp3 * accel_y_bias_t3) * IMU_ACCEL_Y_SIGN *
        IMU_ACCEL_Y_SENS_DEN / ((float) IMU_ACCEL_Y_SENS_NUM));
    int32_t accel_TC_bias_z = (int32_t) round(ACCEL_BFP_OF_REAL(accel_z_bias + imu_bebop_filtered_temperature * accel_z_bias_t1 + temp2 * accel_z_bias_t2 + temp3 * accel_z_bias_t3) * IMU_ACCEL_Z_SIGN *
        IMU_ACCEL_Z_SENS_DEN / ((float) IMU_ACCEL_Z_SENS_NUM));

    _imu->accel.x = ((_imu->accel_unscaled.x - _imu->accel_neutral.x - accel_TC_bias_x) * IMU_ACCEL_X_SIGN *
        IMU_ACCEL_X_SENS_NUM) / IMU_ACCEL_X_SENS_DEN / accel_x_sfe;
    _imu->accel.y = ((_imu->accel_unscaled.y - _imu->accel_neutral.y - accel_TC_bias_y) * IMU_ACCEL_Y_SIGN *
        IMU_ACCEL_Y_SENS_NUM) / IMU_ACCEL_Y_SENS_DEN / accel_y_sfe;
    _imu->accel.z = ((_imu->accel_unscaled.z - _imu->accel_neutral.z - accel_TC_bias_z) * IMU_ACCEL_Z_SIGN *
        IMU_ACCEL_Z_SENS_NUM) / IMU_ACCEL_Z_SENS_DEN / accel_z_sfe;
  }else{
    _imu->accel.x = ((_imu->accel_unscaled.x - _imu->accel_neutral.x) * IMU_ACCEL_X_SIGN *
        IMU_ACCEL_X_SENS_NUM) / IMU_ACCEL_X_SENS_DEN;
    _imu->accel.y = ((_imu->accel_unscaled.y - _imu->accel_neutral.y) * IMU_ACCEL_Y_SIGN *
        IMU_ACCEL_Y_SENS_NUM) / IMU_ACCEL_Y_SENS_DEN;
    _imu->accel.z = ((_imu->accel_unscaled.z - _imu->accel_neutral.z) * IMU_ACCEL_Z_SIGN *
        IMU_ACCEL_Z_SENS_NUM) / IMU_ACCEL_Z_SENS_DEN;
  }

}

void imu_bebop_event(void)
{
  static uint8_t avg_temp_cnt = 1;
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
    if(avg_temp_cnt <= BEBOP_MPU_TEMP_AVG_CNT){
      imu_bebop_filtered_temperature = (imu_bebop.mpu.temp + imu_bebop_filtered_temperature * (avg_temp_cnt - 1)) / ((float) avg_temp_cnt);
      avg_temp_cnt++;
    }else{
      imu_bebop_filtered_temperature = (imu_bebop_filtered_temperature * (BEBOP_MPU_TEMP_FILT_COEF - 1) + imu_bebop.mpu.temp) / ((float) BEBOP_MPU_TEMP_FILT_COEF);
    }
    imu_scale_gyro(&imu);
    imu_scale_accel(&imu);
    AbiSendMsgIMU_GYRO_INT32(IMU_BOARD_ID, now_ts, &imu.gyro);
    AbiSendMsgIMU_ACCEL_INT32(IMU_BOARD_ID, now_ts, &imu.accel);
  }

  /* AKM8963 event task */
  ak8963_event(&imu_bebop.ak);

  if (imu_bebop.ak.data_available) {
#if BEBOP_VERSION2
    struct Int32Vect3 mag_temp;
    // In the second bebop version the magneto is turned 90 degrees
    VECT3_ASSIGN(mag_temp, (-imu_bebop.ak.data.vect.x), (-imu_bebop.ak.data.vect.y), (imu_bebop.ak.data.vect.z));
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
