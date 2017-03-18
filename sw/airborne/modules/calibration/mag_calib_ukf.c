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

#include <stdio.h>
#include <error.h>
#include <stdbool.h>
#include <time.h>

#include "modules/calibration/mag_calib_ukf.h"
#include "subsystems/ahrs/ahrs_magnetic_field_model.h"
#include "math/pprz_algebra_double.h"
#include "TRICAL.h"
#include "state.h"
#include "subsystems/imu/imu_bebop.h"
#include "modules/geo_mag/geo_mag.h"
#include "generated/airframe.h"
#include "subsystems/gps.h"

TRICAL_instance_t mag_calib;

time_t mag_field_from_geo_mag;
bool   update_geo_mag_field = true;
struct FloatVect3 H = { .x = MAG_CALIB_UKF_NORM, .y = 0.0f, .z =  0.0f};

void mag_calib_ukf_init(struct Imu *_imu) {
    TRICAL_init(&mag_calib);
    TRICAL_norm_set(&mag_calib, MAG_CALIB_UKF_NORM);
    TRICAL_noise_set(&mag_calib, MAG_CALIB_UKF_NOISE_RMS);
    VERBOSE_PRINT("magnetometer initial calibration(x_n: %d  y_n: %d  z_n: %d)\n", _imu->mag_neutral.x, _imu->mag_neutral.y, _imu->mag_neutral.z);
    _imu->mag_neutral.x = 0;
    _imu->mag_neutral.y = 0;
    _imu->mag_neutral.z = 0;
    /* Delft */
    H.x = 0.3892503;
    H.y = 0.0017972;
    H.z = 0.9211303;
}

void mag_calib_ukf_run(struct Imu *_imu) {
    float bias[3] = {0.0, 0.0, 0.0}, scale[9] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}, meas[3] = {0.0, 0.0, 0.0}, calib_meas[3] = {0.0, 0.0, 0.0};
    /** Update geo_mag based on MAG_CALIB_UKF_GEO_MAG_TIMEOUT (0 = no periodic updates) **/
    if(update_geo_mag_field && geo_mag.ready){
        double n                = double_vect3_norm(&geo_mag.vect);
        H.x                     = (float) geo_mag.vect.x / n;
        H.y                     = (float) geo_mag.vect.y / n;
        H.z                     = (float) geo_mag.vect.z / n;
        mag_field_from_geo_mag  = time(0);
        update_geo_mag_field    = false;
        VERBOSE_PRINT("Updating the geo_mag field to H(%4.2f, %4.2f, %4.2f)\n", H.x, H.y, H.z);
    }
    if(MAG_CALIB_UKF_GEO_MAG_TIMEOUT && GpsFixValid() && difftime( time(0), mag_field_from_geo_mag ) >= MAG_CALIB_UKF_GEO_MAG_TIMEOUT){
        geo_mag.ready           = false;
        geo_mag.calc_once       = true;   ///< Geo_mag will not re-update the calculation when the throttle is on so this is neccesary
        update_geo_mag_field    = true;
    }
    /** Update magnetometer UKF and calibrate measurement **/
    if(_imu->mag.x != 0 || _imu->mag.y != 0 || _imu->mag.z != 0){
        /** Rotate the local magnetic field by our current attitude **/
        struct FloatQuat* body_quat = stateGetNedToBodyQuat_f();
        struct FloatRMat body_rmat;
        float_rmat_of_quat(&body_rmat, body_quat);
        struct FloatVect3 expected_meas;
        float_rmat_vmult(&expected_meas, &body_rmat, &H);
        float expected_mag_field[3] = { expected_meas.x, expected_meas.y, expected_meas.z };
        /** Update magnetometer UKF **/
        meas[0] = MAG_FLOAT_OF_BFP( _imu->mag.x );
        meas[1] = MAG_FLOAT_OF_BFP( _imu->mag.y );
        meas[2] = MAG_FLOAT_OF_BFP( _imu->mag.z );
        TRICAL_estimate_update(&mag_calib, meas, expected_mag_field);
        TRICAL_estimate_get(&mag_calib, bias, scale);
        TRICAL_measurement_calibrate(&mag_calib, meas, calib_meas);
        /** Save calibrated result **/
        _imu->mag.x = (int32_t) MAG_BFP_OF_REAL( calib_meas[0] );
        _imu->mag.y = (int32_t) MAG_BFP_OF_REAL( calib_meas[1] );
        _imu->mag.z = (int32_t) MAG_BFP_OF_REAL( calib_meas[2] );
        VERBOSE_PRINT("magnetometer measurement (x: %4.2f  y: %4.2f  z: %4.2f) norm: %4.2f\n", meas[0], meas[1], meas[2], hypot(hypot(meas[0],meas[1]), meas[2]));
        VERBOSE_PRINT("magnetometer bias_f (x: %4.2f  y: %4.2f  z: %4.2f)\n", bias[0], bias[1], bias[2]);
        VERBOSE_PRINT("expected measurement (x: %4.2f  y: %4.2f  z: %4.2f) norm: %4.2f\n", expected_mag_field[0], expected_mag_field[1], expected_mag_field[2], hypot(hypot(expected_mag_field[0],expected_mag_field[1]), expected_mag_field[2]));
        VERBOSE_PRINT("calibrated   measurement (x: %4.2f  y: %4.2f  z: %4.2f) norm: %4.2f\n\n", calib_meas[0], calib_meas[1], calib_meas[2], hypot(hypot(calib_meas[0],calib_meas[1]), calib_meas[2]));
    }
}
