/*
 * Copyright (C) w.vlenterie
 *
 * This file is part of paparazzi
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
 * along with paparazzi; see the file COPYING.  If not, see
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "modules/bebop_accel_calib_flat/bebop_accel_calib_flat.c"
 * @author w.vlenterie
 * Calibrate the bebop accelerometer from a flat surface
 */

#include "modules/calibration/bebop_accel_calib_flat.h"
#include "subsystems/imu.h"
#include "subsystems/imu/imu_bebop.h"
#include "state.h"
#include <autopilot.h>
#include <stdbool.h>
#include <stdio.h>
#include "subsystems/datalink/telemetry.h"
#include "subsystems/abi.h"

#ifndef BEBOP_ACCEL_CALIB_AUTOSTART
#define BEBOP_ACCEL_CALIB_AUTOSTART 0
#endif

#ifndef BEBOP_ACCEL_CALIB_MEASURE_TIME
#define BEBOP_ACCEL_CALIB_MEASURE_TIME 30.0f
#endif

void bebop_accel_calib_run( uint8_t __attribute__ ((unused)) sender_id, uint32_t __attribute__ ((unused)) stamp, struct Int32Vect3* __attribute__ ((unused)) accel );

int32_t accel_x_tot = 0;
int32_t accel_y_tot = 0;
int32_t accel_z_tot = 0;

int32_t  accel_x_avg = 0;
int32_t  accel_y_avg = 0;
int32_t  accel_z_avg = 0;

int32_t z_ideal;

bool  settings_calibration_running  = BEBOP_ACCEL_CALIB_AUTOSTART;
bool settings_send_accel_neutral    = false;
float settings_calibration_time     = BEBOP_ACCEL_CALIB_MEASURE_TIME;
static abi_event bebop_accel_ev;

void bebop_accel_calib_init( void ) {
    z_ideal = (int32_t) round( ACCEL_BFP_OF_REAL( -9.81 ) * IMU_ACCEL_Z_SENS_DEN * accel_z_sfe / (IMU_ACCEL_Z_SIGN * IMU_ACCEL_Z_SENS_NUM));
    AbiBindMsgIMU_ACCEL_INT32(ABI_BROADCAST, &bebop_accel_ev, bebop_accel_calib_run);
}

#warning Do not fly with this module activated!!

void bebop_accel_calib_run( uint8_t __attribute__ ((unused)) sender_id, uint32_t stamp, struct Int32Vect3* __attribute__ ((unused)) accel ) {
  static uint32_t runCount = 0;
  static uint32_t prevStamp = 0;
  if(settings_send_accel_neutral){
    settings_send_accel_neutral = false;
    bebop_send_accel_neutral();
  }
  if(!autopilot.motors_on && settings_calibration_running){
      if( imu.accel_neutral.x || imu.accel_neutral.y || imu.accel_neutral.z){
          imu.accel_neutral.x = 0;
          imu.accel_neutral.y = 0;
          imu.accel_neutral.z = 0;
      }
      if(runCount != 0){
        settings_calibration_time -= (stamp - prevStamp) / 1000000.0;
      }
      prevStamp = stamp;
      if(settings_calibration_time < 0.0){
          bebop_set_accel_neutral();
          accel_x_tot = 0;
          accel_y_tot = 0;
          accel_z_tot = 0;
          settings_calibration_running = false;
          settings_calibration_time = BEBOP_ACCEL_CALIB_MEASURE_TIME;
          runCount = 0;
          bebop_send_accel_neutral();
      } else {
        accel_x_tot += imu.accel.x;
        accel_y_tot += imu.accel.y;
        accel_z_tot += imu.accel.z;
        runCount++;
        accel_x_avg = (int32_t) round(accel_x_tot * accel_x_sfe / ((float) runCount) * IMU_ACCEL_X_SIGN * IMU_ACCEL_X_SENS_DEN / ((float) IMU_ACCEL_X_SENS_NUM));
        accel_y_avg = (int32_t) round(accel_y_tot * accel_y_sfe / ((float) runCount) * IMU_ACCEL_Y_SIGN * IMU_ACCEL_Y_SENS_DEN / ((float) IMU_ACCEL_Y_SENS_NUM));
        accel_z_avg = (int32_t) round(accel_z_tot * accel_z_sfe / ((float) runCount) * IMU_ACCEL_Z_SIGN * IMU_ACCEL_Z_SENS_DEN / ((float) IMU_ACCEL_Z_SENS_NUM));
      }
  }
}

void bebop_send_accel_neutral( void ){
  char data[200];
  snprintf(data, 200, "bebob %d ACC (X %d,  Y %d,  Z  %d)", AC_ID, accel_x_avg, accel_y_avg, accel_z_avg - z_ideal);
  printf("%s\n",data);
  DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, strlen(data), data);
}

void bebop_set_accel_neutral( void ){
    imu.accel_neutral.x = accel_x_avg;
    imu.accel_neutral.y = accel_y_avg;
    imu.accel_neutral.z = accel_z_avg - z_ideal;
}
