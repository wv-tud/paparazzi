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

#include "modules/bebop_accel_calib_flat/bebop_accel_calib_flat.h"
#include "subsystems/imu.h"
#include "subsystems/imu/imu_bebop.h"
#include "state.h"
#include <autopilot.h>
#include <stdbool.h>
#include <stdio.h>
#include "subsystems/datalink/telemetry.h"

#ifndef BEBOP_ACCEL_CALIB_MEASURE_TIME
#define BEBOP_ACCEL_CALIB_MEASURE_TIME 30
#endif

uint32_t runCount = 0;
int32_t accel_x_tot = 0;
int32_t accel_y_tot = 0;
int32_t accel_z_tot = 0;

int32_t  accel_x_avg = 0;
int32_t  accel_y_avg = 0;
int32_t  accel_z_avg = 0;

int32_t z_ideal;

bool calibration_running = true;

void bebop_accel_calib_init( void ) {
    imu.accel_neutral.x = 0;
    imu.accel_neutral.y = 0;
    imu.accel_neutral.z = 0;
    z_ideal = (int32_t) round( ACCEL_BFP_OF_REAL( -9.81 ) * MPU60X0_ACCEL_SENS_FRAC[BEBOP_ACCEL_RANGE][1] / (IMU_ACCEL_Z_SIGN * MPU60X0_ACCEL_SENS_FRAC[BEBOP_ACCEL_RANGE][0]));
}

#warning Do not fly with this module activated!!

void bebop_accel_calib_run( void ) {
  if(!autopilot.motors_on && calibration_running){
      if(runCount == BEBOP_ACCEL_CALIB_MEASURE_TIME * PERIODIC_FREQUENCY){
          bebop_set_accel_neutral();
          calibration_running = false;
          char data[200];
          snprintf(data, 200, "bebob %d ACCEL_NEUTRAL (X: %d  Y: %d  Z:  %d)", AC_ID, accel_x_avg, accel_y_avg, accel_z_avg - z_ideal);
          DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, strlen(data), data);
      }
      accel_x_tot           += imu.accel_unscaled.x;
      accel_y_tot           += imu.accel_unscaled.y;
      accel_z_tot           += imu.accel_unscaled.z;
      runCount++;
      accel_x_avg = (int32_t) round( accel_x_tot / ((double) runCount) );
      accel_y_avg = (int32_t) round( accel_y_tot / ((double) runCount) );
      accel_z_avg = (int32_t) round( accel_z_tot / ((double) runCount) );
  }
}

void bebop_set_accel_neutral( void ){
    imu.accel_neutral.x = accel_x_avg;
    imu.accel_neutral.y = accel_y_avg;
    imu.accel_neutral.z = accel_z_avg - z_ideal;
}
