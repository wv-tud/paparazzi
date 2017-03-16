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
 * @file "modules/calibration/mag_calib_ukf.c"
 * @author w.vlenterie
 * Calibrate magnetometer using UKF
 */

#include "modules/calibration/mag_calib_ukf.h"
#include "subsystems/ahrs/ahrs_magnetic_field_model.h"
#include "math/pprz_algebra_double.h"
#include "TRICAL.h"
#include "state.h"
#include "subsystems/imu.h"
#include "subsystems/imu/imu_bebop.h"
#include "modules/geo_mag/geo_mag.h"
#include "stdio.h"
#include "error.h"
#include "generated/airframe.h"
#include <stdbool.h>

#define PRINT(string,...) fprintf(stderr, "[CALIB_UKF->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

#if !defined CALIB_UKF_VERBOSE
#define VERBOSE_PRINT(...)
#elif CALIB_UKF_VERBOSE == TRUE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

TRICAL_instance_t mag_calib;
TRICAL_instance_t acc_calib;

bool mag_field_from_geo_mag = false;

float accel_field[3] = {0.0f, 0.0f, -9.81f};
//mag_field[3] = {1.0f, 0.0f, 0.0f};

void mag_calib_ukf_init() {
    TRICAL_init(&mag_calib);
    TRICAL_norm_set(&mag_calib, 1.0f);
    TRICAL_noise_set(&mag_calib, 1e-4f);

    VERBOSE_PRINT("magnetometer initial calibration(x_n: %d  y_n: %d  z_n: %d)\n", imu.mag_neutral.x, imu.mag_neutral.y, imu.mag_neutral.z);

    imu.mag_neutral.x = 0;
    imu.mag_neutral.y = 0;
    imu.mag_neutral.z = 0;
    /*
    mag_field[0] = 0.3892503;
    mag_field[1] = 0.0017972;
    mag_field[2] = 0.9211303;
    */
}

void accel_calib_ukf_init() {
    TRICAL_init(&acc_calib);
    TRICAL_norm_set(&acc_calib, (float) 9.81);
    TRICAL_noise_set(&acc_calib, (float) 0.5f);

    VERBOSE_PRINT("acelerometer initial calibration(x_n: %d  y_n: %d  z_n: %d)\n", imu.accel_neutral.x, imu.accel_neutral.y, imu.accel_neutral.z);

    imu.accel_neutral.x = 0;
    imu.accel_neutral.y = 0;
    imu.accel_neutral.z = 0;
    accel_field[0] = 0.0;
    accel_field[1] = 0.0;
    accel_field[2] = -9.81;
}

void mag_calib_ukf_run() {
    float bias[3] = {0.0, 0.0, 0.0}, scale[9] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, meas[3] = {0.0, 0.0, 0.0}, calib_meas[3] = {0.0, 0.0, 0.0};
    /*
    if(geo_mag.ready && mag_field_from_geo_mag == false){
        struct FloatVect3 vect = { .x = geo_mag.vect.x ,
                                   .y = geo_mag.vect.y ,
                                   .z = geo_mag.vect.z  };
        float n = float_vect3_norm(&vect);
        mag_field[0] = vect.x / n;
        mag_field[1] = vect.y / n;
        mag_field[2] = vect.z / n;
        mag_field_from_geo_mag = true;
    }
    */
    if(imu.mag.x != 0 || imu.mag.y != 0 || imu.mag.z != 0){
        // Update magnetometer UKF
        meas[0] = MAG_FLOAT_OF_BFP( imu.mag.x );
        meas[1] = MAG_FLOAT_OF_BFP( imu.mag.y );
        meas[2] = MAG_FLOAT_OF_BFP( imu.mag.z );
        //VERBOSE_PRINT("magnetometer field (x: %4.2f  y: %4.2f  z: %4.2f) norm: %4.2f\n", mag_field[0], mag_field[1], mag_field[2], hypot(hypot(mag_field[0],mag_field[1]), mag_field[2]));
        VERBOSE_PRINT("magnetometer measurement (x: %4.2f  y: %4.2f  z: %4.2f) norm: %4.2f\n", meas[0], meas[1], meas[2], hypot(hypot(meas[0],meas[1]), meas[2]));
        TRICAL_estimate_update(&mag_calib, meas, meas);//mag_field);
        // Read magnetometer estimate and update measurement
        TRICAL_estimate_get(&mag_calib, bias, scale);
        VERBOSE_PRINT("magnetometer bias (x_b: %4.2f  y_b: %4.2f  z_b: %4.2f)\n", bias[0], bias[1], bias[2]);
        TRICAL_measurement_calibrate(&mag_calib, meas, calib_meas);
        VERBOSE_PRINT("calibrated   measurement (x: %4.2f  y: %4.2f  z: %4.2f) norm: %4.2f\n", calib_meas[0], calib_meas[1], calib_meas[2], hypot(hypot(calib_meas[0],calib_meas[1]), calib_meas[2]));
        float n = hypot(hypot(calib_meas[0],calib_meas[1]), calib_meas[2]);
        imu.mag.x = (int32_t) MAG_BFP_OF_REAL( calib_meas[0] / n );
        imu.mag.y = (int32_t) MAG_BFP_OF_REAL( calib_meas[1] / n );
        imu.mag.z = (int32_t) MAG_BFP_OF_REAL( calib_meas[2] / n );
    }
}

void accel_calib_ukf_run() {
    float bias[3] = {0.0, 0.0, 0.0}, scale[9] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, meas[3] = {0.0, 0.0, 0.0}, calib_meas[3] = {0.0, 0.0, 0.0};
    if(imu.accel.x != 0 || imu.accel.y != 0 || imu.accel.z != 0){
        // Update accelerometer UKF
        meas[0] = ACCEL_FLOAT_OF_BFP( imu.accel.x );
        meas[1] = ACCEL_FLOAT_OF_BFP( imu.accel.y );
        meas[2] = ACCEL_FLOAT_OF_BFP( imu.accel.z );
        //VERBOSE_PRINT("accelerometer field (x: %4.2f  y: %4.2f  z: %4.2f) norm: %4.2f\n", accel_field[0], accel_field[1], accel_field[2], hypot(hypot(accel_field[0],accel_field[1]), accel_field[2]));
        VERBOSE_PRINT("accelerometer measurement (x: %4.2f  y: %4.2f  z: %4.2f) norm: %4.2f\n", meas[0], meas[1], meas[2], hypot(hypot(meas[0],meas[1]), meas[2]));
        TRICAL_estimate_update(&acc_calib, meas, meas);// accel_field);
        // Read accelerometer estimate and update measurement
        TRICAL_estimate_get(&acc_calib, bias, scale);
        VERBOSE_PRINT("accelerometer bias (x_b: %4.2f  y_b: %4.2f  z_b: %4.2f)\n", bias[0], bias[1], bias[2]);
        TRICAL_measurement_calibrate(&acc_calib, meas,  calib_meas);
        VERBOSE_PRINT("calibrated   measurement (x: %4.2f  y: %4.2f  z: %4.2f) norm: %4.2f\n", calib_meas[0], calib_meas[1], calib_meas[2], hypot(hypot(calib_meas[0],calib_meas[1]), calib_meas[2]));
        imu.accel.x = (int32_t) ACCEL_BFP_OF_REAL( calib_meas[0] );
        imu.accel.y = (int32_t) ACCEL_BFP_OF_REAL( calib_meas[1] );
        imu.accel.z = (int32_t) ACCEL_BFP_OF_REAL( calib_meas[2] );
    }
}

