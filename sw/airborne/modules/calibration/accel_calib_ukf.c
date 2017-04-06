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
 * @file "modules/calibration/accel_calib_ukf.c"
 * @author w.vlenterie
 * Calibrate the accelerometer using an unscented kalman filter
 * For more information please visit the following links:
 *   - https://github.com/sfwa/trical
 *   - http://au.tono.my/log/20131213-trical-accelerometer-calibration.html
 *   - http://www.acsu.buffalo.edu/~johnc/mag_cal05.pdf
 */
#include "modules/calibration/accel_calib_ukf.h"

#include "autopilot.h"
#include "math/pprz_algebra_double.h"
#include "state.h"
#include "subsystems/imu.h"
#include "subsystems/abi.h"
#include "generated/airframe.h"
#include "subsystems/datalink/telemetry.h"

//
// Try to print warnings to user for bad configuration
//
#if !defined AHRS_FC_ACCEL_ID && !defined AHRS_ICE_ACCEL_ID && !defined AHRS_MLKF_ACCEL_ID && !defined AHRS_FINV_ACCEL_ID && \
  !defined AHRS_DCM_ACCEL_ID && !defined AHRS_ICQ_ACCEL_ID && !defined INS_FINV_ACCEL_ID
#warning "your AHRS/INS configuration might be wrong to use onboard accel calibration, please refer to the documentation"
#endif

#if defined AHRS_FC_ACCEL_ID && (AHRS_FC_ACCEL_ID != ACCEL_CALIB_UKF_ID)
#warning "your AHRS_FC_ACCEL_ID might by wrong please set to ACCEL_CALIB_UKF_ID to use onboard accel calibration"
#endif

#if defined AHRS_ICE_ACCEL_ID && (AHRS_ICE_ACCEL_ID != ACCEL_CALIB_UKF_ID)
#warning "your AHRS_ICE_ACCEL_ID might by wrong please set to ACCEL_CALIB_UKF_ID to use onboard accel calibration"
#endif

#if defined AHRS_MLKF_ACCEL_ID && (AHRS_MLKF_ACCEL_ID != ACCEL_CALIB_UKF_ID)
#warning "your AHRS_MLKF_ACCEL_ID might by wrong please set to ACCEL_CALIB_UKF_ID to use onboard accel calibration"
#endif

#if defined AHRS_FINV_ACCEL_ID && (AHRS_FINV_ACCEL_ID != ACCEL_CALIB_UKF_ID)
#warning "your AHRS_FINV_ACCEL_ID might by wrong please set to ACCEL_CALIB_UKF_ID to use onboard accel calibration"
#endif

#if defined AHRS_DCM_ACCEL_ID && (AHRS_DCM_ACCEL_ID != ACCEL_CALIB_UKF_ID)
#warning "your AHRS_DCM_ACCEL_ID might by wrong please set to ACCEL_CALIB_UKF_ID to use onboard accel calibration"
#endif

#if defined AHRS_ICQ_ACCEL_ID && (AHRS_ICQ_ACCEL_ID != ACCEL_CALIB_UKF_ID)
#warning "your AHRS_ICQ_ACCEL_ID might by wrong please set to ACCEL_CALIB_UKF_ID to use onboard accel calibration"
#endif

#if defined INS_FINV_ACCEL_ID && (INS_FINV_ACCEL_ID != ACCEL_CALIB_UKF_ID)
#warning "your INS_FINV_ACCEL_ID might by wrong please set to ACCEL_CALIB_UKF_ID to use onboard accel calibration"
#endif

// ABI callback declarations
static void accel_calib_ukf_run(uint8_t sender_id, uint32_t stamp, struct Int32Vect3 *accel);

// Verbose mode is only available on Linux-based autopilots
#ifndef ACCEL_CALIB_UKF_VERBOSE
#define ACCEL_CALIB_UKF_VERBOSE FALSE
#endif

#ifndef ACCEL_CALIB_UKF_ABI_BIND_ID
#define ACCEL_CALIB_UKF_ABI_BIND_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(ACCEL_CALIB_UKF_ABI_BIND_ID)

#ifndef ACCEL_CALIB_UKF_NORM
#define ACCEL_CALIB_UKF_NORM 1.0f
#endif
PRINT_CONFIG_VAR(ACCEL_CALIB_UKF_NORM)

#ifndef ACCEL_CALIB_UKF_NOISE_RMS
#define ACCEL_CALIB_UKF_NOISE_RMS 3e-1f
#endif
PRINT_CONFIG_VAR(ACCEL_CALIB_UKF_NOISE_RMS)

// Hotstart is only available on Linux-based autopilots
#ifndef ACCEL_CALIB_UKF_HOTSTART
#define ACCEL_CALIB_UKF_HOTSTART FALSE
#endif
PRINT_CONFIG_VAR(ACCEL_CALIB_UKF_HOTSTART)

#ifndef ACCEL_CALIB_UKF_HOTSTART_SAVE_FILE
#define ACCEL_CALIB_UKF_HOTSTART_SAVE_FILE /data/ftp/internal_000/accel_ukf_calib.txt
#endif
PRINT_CONFIG_VAR(ACCEL_CALIB_UKF_HOTSTART_SAVE_FILE)

#if ACCEL_CALIB_UKF_VERBOSE || ACCEL_CALIB_UKF_HOTSTART
#include <stdio.h>
#endif
#if ACCEL_CALIB_UKF_VERBOSE
#define VERBOSE_PRINT(string,...) fprintf(stderr, "[ACCEL_CALIB_UKF->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#else
#define VERBOSE_PRINT(...)
#endif
PRINT_CONFIG_VAR(ACCEL_CALIB_UKF_VERBOSE)

bool accel_calib_ukf_reset_state = false;
bool accel_calib_ukf_send_state = false;
struct Int32Vect3 calibrated_accel;

TRICAL_instance_t accel_calib;
static abi_event accel_ev;

#if ACCEL_CALIB_UKF_HOTSTART
static FILE *fp;
static char hotstart_file_name[512];
#endif

void accel_calib_ukf_init(void)
{
  TRICAL_init(&accel_calib);
  TRICAL_norm_set(&accel_calib, ACCEL_CALIB_UKF_NORM);
  TRICAL_noise_set(&accel_calib, ACCEL_CALIB_UKF_NOISE_RMS);
  accel_calib_hotstart_read();
#ifdef ACCEL_CALIB_UKF_INITIAL_STATE
  float initial_state[9] = ACCEL_CALIB_UKF_INITIAL_STATE;
  memcpy(&accel_calib.state, &initial_state, 9 * sizeof(float));
#endif
  AbiBindMsgIMU_ACCEL_INT32(ACCEL_CALIB_UKF_ABI_BIND_ID, &accel_ev, accel_calib_ukf_run);
  VERBOSE_PRINT("Initial state:\n"
		  "State {%4.2f, %4.2f, %4.2f}\n"
		  "      {%4.2f, %4.2f, %4.2f}\n"
		  "      {%4.2f, %4.2f, %4.2f}\n",
		  accel_calib.state[0], accel_calib.state[1],  accel_calib.state[2],
		  accel_calib.state[3], accel_calib.state[4],  accel_calib.state[5],
		  accel_calib.state[6], accel_calib.state[7],  accel_calib.state[8]
  );
}

/** Callback function run for every new accel measurement
 */
void accel_calib_ukf_run(uint8_t sender_id, uint32_t stamp, struct Int32Vect3 *accel)
{
  float measurement[3] 				= {0.0f, 0.0f, 0.0f};
  float calibrated_measurement[3] 	= {0.0f, 0.0f, 0.0f};
  if (sender_id != ACCEL_CALIB_UKF_ID && !autopilot.in_flight  && !autopilot.motors_on) {
    /** See if we need to reset the state **/
    if (accel_calib_ukf_send_state) {
      accel_calib_send_state();
      accel_calib_ukf_send_state = false;
    }
    if (accel_calib_ukf_reset_state || isnan(accel_calib.state[0])) {
      TRICAL_reset(&accel_calib);
      accel_calib_ukf_reset_state = false;
    }
    /** Update accelerometer UKF and calibrate measurement **/
    if (accel->x != 0 || accel->y != 0 || accel->z != 0) {
      measurement[0] = ACCEL_FLOAT_OF_BFP(accel->x);
      measurement[1] = ACCEL_FLOAT_OF_BFP(accel->y);
      measurement[2] = ACCEL_FLOAT_OF_BFP(accel->z);
      measurement[0] /= 9.81;
      measurement[1] /= 9.81;
      measurement[2] /= 9.81;
      TRICAL_measurement_calibrate(&accel_calib, measurement, calibrated_measurement);
      if(fabs(sqrt(pow(calibrated_measurement[0], 2.0) + pow(calibrated_measurement[1], 2.0) + pow(calibrated_measurement[2], 2.0)) - 1) < 0.05){
    	  /** Update accelerometer UKF **/
    	  TRICAL_estimate_update(&accel_calib, measurement);
    	  TRICAL_measurement_calibrate(&accel_calib, measurement, calibrated_measurement);
      }
      calibrated_measurement[0] *= 9.81;
      calibrated_measurement[1] *= 9.81;
      calibrated_measurement[2] *= 9.81;
      /** Save calibrated result **/
      calibrated_accel.x = (int32_t) ACCEL_BFP_OF_REAL(calibrated_measurement[0]);
      calibrated_accel.y = (int32_t) ACCEL_BFP_OF_REAL(calibrated_measurement[1]);
      calibrated_accel.z = (int32_t) ACCEL_BFP_OF_REAL(calibrated_measurement[2]);
      imu.accel.x = calibrated_accel.x;
      imu.accel.y = calibrated_accel.y;
      imu.accel.z = calibrated_accel.z;
      /** Debug print */
      VERBOSE_PRINT("accelerometer measurement (x: %4.2f  y: %4.2f  z: %4.2f) norm: %4.2f\n", measurement[0], measurement[1], measurement[2], hypot(hypot(measurement[0], measurement[1]), measurement[2]));
      VERBOSE_PRINT("accelerometer bias_f      (x: %4.2f  y: %4.2f  z: %4.2f)\n", accel_calib.state[0], accel_calib.state[1],  accel_calib.state[2]);
      VERBOSE_PRINT("calibrated    measurement (x: %4.2f  y: %4.2f  z: %4.2f) norm: %4.2f\n\n", calibrated_measurement[0], calibrated_measurement[1], calibrated_measurement[2], hypot(hypot(calibrated_measurement[0], calibrated_measurement[1]), calibrated_measurement[2]));
      /** Forward calibrated data */
      AbiSendMsgIMU_ACCEL_INT32(ACCEL_CALIB_UKF_ID, stamp, &calibrated_accel);
    }
  }
}

void accel_calib_send_state(void)
{
  DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, 9, accel_calib.state);
}

void accel_calib_hotstart_read(void)
{
#if ACCEL_CALIB_UKF_HOTSTART
  snprintf(hotstart_file_name, 512, "%s", STRINGIFY(ACCEL_CALIB_UKF_HOTSTART_SAVE_FILE));
  fp = fopen(hotstart_file_name, "r");
  if (fp != NULL) {
    fread(accel_calib.state, sizeof(float), 9, fp);
    fclose(fp);
    VERBOSE_PRINT("Loaded initial state from disk:\n"
                  "State {%4.2f, %4.2f, %4.2f}\n"
                  "      {%4.2f, %4.2f, %4.2f}\n"
                  "      {%4.2f, %4.2f, %4.2f}\n",
                  accel_calib.state[0], accel_calib.state[1],  accel_calib.state[2],
                  accel_calib.state[3], accel_calib.state[4],  accel_calib.state[5],
                  accel_calib.state[6], accel_calib.state[7],  accel_calib.state[8]
                 );
    if(isnan(accel_calib.state[0]) || isnan(accel_calib.state[1]) || isnan(accel_calib.state[2]) || isnan(accel_calib.state[3]) ||
    		isnan(accel_calib.state[4]) || isnan(accel_calib.state[5]) || isnan(accel_calib.state[6]) || isnan(accel_calib.state[7]) || isnan(accel_calib.state[8])){
    	accel_calib_ukf_reset_state = true;
    }
  }

#endif
}

void accel_calib_hotstart_write(void)
{
#if ACCEL_CALIB_UKF_HOTSTART
  fp = fopen(hotstart_file_name, "w");
  if (fp != NULL) {
    fwrite(accel_calib.state, sizeof(float), 9, fp);
    fclose(fp);
    VERBOSE_PRINT("Wrote current state to disk:\n"
                  "State {%4.2f, %4.2f, %4.2f}\n"
                  "      {%4.2f, %4.2f, %4.2f}\n"
                  "      {%4.2f, %4.2f, %4.2f}\n",
                  accel_calib.state[0], accel_calib.state[1],  accel_calib.state[2],
                  accel_calib.state[3], accel_calib.state[4],  accel_calib.state[5],
                  accel_calib.state[6], accel_calib.state[7],  accel_calib.state[8]
                 );
  }
#endif
}
