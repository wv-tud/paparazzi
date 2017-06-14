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
 * Calibrate the magnetometer using an unscented kalman filter
 * For more information please visit the following links:
 *   - https://github.com/sfwa/trical
 *   - http://au.tono.my/log/20131213-trical-magnetometer-calibration.html
 *   - http://www.acsu.buffalo.edu/~johnc/mag_cal05.pdf
 */
#include "modules/calibration/mag_calib_ukf.h"

#include "math/pprz_algebra_double.h"
#include "state.h"
#include "subsystems/imu.h"
#include "subsystems/abi.h"
#include "generated/airframe.h"
#include "subsystems/ahrs/ahrs_magnetic_field_model.h"
#include "subsystems/datalink/telemetry.h"
#include "subsystems/ahrs/ahrs_int_utils.h"
#include "subsystems/ahrs/ahrs_aligner.h"

//
// Try to print warnings to user for bad configuration
//
#if !defined AHRS_FC_MAG_ID && !defined AHRS_ICE_MAG_ID && !defined AHRS_MLKF_MAG_ID && !defined AHRS_FINV_MAG_ID && \
  !defined AHRS_DCM_MAG_ID && !defined AHRS_ICQ_MAG_ID && !defined INS_FINV_MAG_ID
#warning "your AHRS/INS configuration might be wrong to use onboard mag calibration, please refer to the documentation"
#endif

#if defined AHRS_FC_MAG_ID && (AHRS_FC_MAG_ID != MAG_CALIB_UKF_ID)
#warning "your AHRS_FC_MAG_ID might by wrong please set to MAG_CALIB_UKF_ID to use onboard mag calibration"
#endif

#if defined AHRS_ICE_MAG_ID && (AHRS_ICE_MAG_ID != MAG_CALIB_UKF_ID)
#warning "your AHRS_ICE_MAG_ID might by wrong please set to MAG_CALIB_UKF_ID to use onboard mag calibration"
#endif

#if defined AHRS_MLKF_MAG_ID && (AHRS_MLKF_MAG_ID != MAG_CALIB_UKF_ID)
#warning "your AHRS_MLKF_MAG_ID might by wrong please set to MAG_CALIB_UKF_ID to use onboard mag calibration"
#endif

#if defined AHRS_FINV_MAG_ID && (AHRS_FINV_MAG_ID != MAG_CALIB_UKF_ID)
#warning "your AHRS_FINV_MAG_ID might by wrong please set to MAG_CALIB_UKF_ID to use onboard mag calibration"
#endif

#if defined AHRS_DCM_MAG_ID && (AHRS_DCM_MAG_ID != MAG_CALIB_UKF_ID)
#warning "your AHRS_DCM_MAG_ID might by wrong please set to MAG_CALIB_UKF_ID to use onboard mag calibration"
#endif

#if defined AHRS_ICQ_MAG_ID && (AHRS_ICQ_MAG_ID != MAG_CALIB_UKF_ID)
#warning "your AHRS_ICQ_MAG_ID might by wrong please set to MAG_CALIB_UKF_ID to use onboard mag calibration"
#endif

#if defined INS_FINV_MAG_ID && (INS_FINV_MAG_ID != MAG_CALIB_UKF_ID)
#warning "your INS_FINV_MAG_ID might by wrong please set to MAG_CALIB_UKF_ID to use onboard mag calibration"
#endif

// ABI callback declarations
static void mag_calib_ukf_run(uint8_t sender_id, uint32_t stamp, struct Int32Vect3 *mag);
static void mag_calib_update_field(uint8_t __attribute__((unused)) sender_id, struct FloatVect3 *h);

// Verbose mode is only available on Linux-based autopilots
#ifndef MAG_CALIB_UKF_VERBOSE
#define MAG_CALIB_UKF_VERBOSE FALSE
#endif

#ifndef MAG_CALIB_UKF_ABI_BIND_ID
#define MAG_CALIB_UKF_ABI_BIND_ID ABI_BROADCAST
#endif
PRINT_CONFIG_VAR(MAG_CALIB_UKF_ABI_BIND_ID)

#ifndef MAG_CALIB_UKF_NORM
#define MAG_CALIB_UKF_NORM 1.0f
#endif
PRINT_CONFIG_VAR(MAG_CALIB_UKF_NORM)

#ifndef MAG_CALIB_UKF_NOISE_RMS
#define MAG_CALIB_UKF_NOISE_RMS 1.5e-1f
#endif
PRINT_CONFIG_VAR(MAG_CALIB_UKF_NOISE_RMS)

// Hotstart is only available on Linux-based autopilots
#ifndef MAG_CALIB_UKF_HOTSTART
#define MAG_CALIB_UKF_HOTSTART FALSE
#endif
PRINT_CONFIG_VAR(MAG_CALIB_UKF_HOTSTART)

#ifndef MAG_CALIB_UKF_HOTSTART_SAVE_FILE
#define MAG_CALIB_UKF_HOTSTART_SAVE_FILE /data/ftp/internal_000/mag_ukf_calib.txt
#endif
PRINT_CONFIG_VAR(MAG_CALIB_UKF_HOTSTART_SAVE_FILE)

#ifndef MAG_CALIB_UKF_CLEAN_START
#define MAG_CALIB_UKF_CLEAN_START FALSE
#endif
PRINT_CONFIG_VAR(MAG_CALIB_UKF_CLEAN_START)

#if MAG_CALIB_UKF_VERBOSE || MAG_CALIB_UKF_HOTSTART
#include <stdio.h>
#define PRINT(string,...) fprintf(stderr, "[MAG_CALIB_UKF->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#endif
#if MAG_CALIB_UKF_VERBOSE
#define VERBOSE_PRINT(string,...) fprintf(stderr, "[MAG_CALIB_UKF->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#else
#define VERBOSE_PRINT(...)
#endif
PRINT_CONFIG_VAR(MAG_CALIB_UKF_VERBOSE)

bool mag_calib_ukf_reset_state = false;
bool mag_calib_ukf_send_state = false;
struct Int32Vect3 calibrated_mag;

float angle_diff_f;
float magneto_psi_f;



TRICAL_instance_t mag_calib;
static abi_event mag_ev;
static abi_event h_ev;
static struct FloatVect3 H = { .x = AHRS_H_X, .y = AHRS_H_Y, .z =  AHRS_H_Z};

#if MAG_CALIB_UKF_HOTSTART
static FILE *fp;
static char mag_hotstart_file_name[512];
#endif

void mag_calib_ukf_init(void)
{
  TRICAL_init(&mag_calib);
  TRICAL_norm_set(&mag_calib, MAG_CALIB_UKF_NORM);
  TRICAL_noise_set(&mag_calib, MAG_CALIB_UKF_NOISE_RMS);
  mag_calib_hotstart_read();
#if MAG_CALIV_UKF_CLEAN_START
  TRICAL_reset(&mag_calib);
#endif
#ifdef MAG_CALIB_UKF_INITIAL_STATE
  float initial_state[TRICAL_STATE_DIM] = MAG_CALIB_UKF_INITIAL_STATE;
  memcpy(&mag_calib.state, &initial_state, TRICAL_STATE_DIM * sizeof(float));
#endif
  AbiBindMsgIMU_MAG_INT32(MAG_CALIB_UKF_ABI_BIND_ID, &mag_ev, mag_calib_ukf_run);
  AbiBindMsgGEO_MAG(ABI_BROADCAST, &h_ev, mag_calib_update_field);
}

/** Callback function run for every new mag measurement
 */
void mag_calib_ukf_run(uint8_t sender_id, uint32_t stamp, struct Int32Vect3 *mag)
{
  static uint32_t runCount = 0;
  static int32_t avg_heading;
  float measurement[3] = {0.0f, 0.0f, 0.0f};
  float calibrated_measurement[3] = {0.0f, 0.0f, 0.0f};
  if (sender_id != MAG_CALIB_UKF_ID) {
    /** See if we need to reset the state **/
    if (mag_calib_ukf_send_state) {
      mag_calib_send_state();
      mag_calib_ukf_send_state = false;
    }
    if (mag_calib_ukf_reset_state || isnan(mag_calib.state[0])) {
      TRICAL_reset(&mag_calib);
      mag_calib_ukf_reset_state = false;
    }
    /** Update magnetometer UKF and calibrate measurement **/
    if (mag->x != 0 || mag->y != 0 || mag->z != 0) {
      runCount++;
      measurement[0] = MAG_FLOAT_OF_BFP(mag->x);
      measurement[1] = MAG_FLOAT_OF_BFP(mag->y);
      measurement[2] = MAG_FLOAT_OF_BFP(mag->z);
      /** Update magnetometer UKF **/
#if 1
      /*
       * Norm only:
       * Please checkout branch norm_only of wv-tud/TRICAL
       */
      TRICAL_estimate_update(&mag_calib, measurement);
#else
      /* Full 3x3 support:
       * Please checkout branch master of wv-tud/TRICAL
       * */
      float expected_mag_field[3] = {0.0f, 0.0f, 0.0f};
      struct FloatQuat *body_quat = stateGetNedToBodyQuat_f();
      struct FloatVect3 expected_measurement;
      float_quat_vmult(&expected_measurement, body_quat, &H);
      expected_mag_field[0] = expected_measurement.x;
      expected_mag_field[1] = expected_measurement.y;
      expected_mag_field[2] = expected_measurement.z;
      TRICAL_estimate_update(&mag_calib, measurement, expected_mag_field);
#endif
      TRICAL_measurement_calibrate(&mag_calib, measurement, calibrated_measurement);
      float measurement_norm = sqrtf(powf(calibrated_measurement[0], 2.0) + powf(calibrated_measurement[1], 2.0) + powf(calibrated_measurement[2], 2.0));
      /** Normalize measurement data */
      if(measurement_norm > 0.1){
        calibrated_mag.x = (int32_t) MAG_BFP_OF_REAL(calibrated_measurement[0] / measurement_norm);
        calibrated_mag.y = (int32_t) MAG_BFP_OF_REAL(calibrated_measurement[1] / measurement_norm);
        calibrated_mag.z = (int32_t) MAG_BFP_OF_REAL(calibrated_measurement[2] / measurement_norm);
      }else{
        calibrated_mag.x = (int32_t) MAG_BFP_OF_REAL(calibrated_measurement[0]);
        calibrated_mag.y = (int32_t) MAG_BFP_OF_REAL(calibrated_measurement[1]);
        calibrated_mag.z = (int32_t) MAG_BFP_OF_REAL(calibrated_measurement[2]);
      }
      /** Save calibrated result **/
      imu.mag.x = calibrated_mag.x;
      imu.mag.y = calibrated_mag.y;
      imu.mag.z = calibrated_mag.z;

      /** Debug print */
      VERBOSE_PRINT("magnetometer measurement (x: %4.2f  y: %4.2f  z: %4.2f) norm: %4.2f\n", measurement[0], measurement[1],
                    measurement[2], hypot(hypot(measurement[0], measurement[1]), measurement[2]));
      VERBOSE_PRINT("magnetometer bias_f      (x: %4.2f  y: %4.2f  z: %4.2f)\n", mag_calib.state[0], mag_calib.state[1],
                    mag_calib.state[2]);
      VERBOSE_PRINT("calibrated   measurement (x: %4.2f  y: %4.2f  z: %4.2f) norm: %4.2f\n\n", calibrated_measurement[0],
                    calibrated_measurement[1], calibrated_measurement[2], hypot(hypot(calibrated_measurement[0], calibrated_measurement[1]),
                        calibrated_measurement[2]));
      struct Int32Eulers e;
      ahrs_int_get_euler_from_accel_mag(&e, &imu.accel, &imu.mag);
      magneto_psi_f = ANGLE_FLOAT_OF_BFP(e.psi);
      if(runCount < 50){
        avg_heading += e.psi;
      }else if(runCount == 50){
        ahrs_icq_realign_heading(avg_heading / ((float) runCount));
      }
      /*else {
        ahrs_icq_update_heading(e.psi);
      }*/
      printf("e.psi: %0.2f  (%d  %d  %d)\n", magneto_psi_f / M_PI * 180.0, calibrated_mag.x, calibrated_mag.y, calibrated_mag.z);

      /** Forward calibrated data */
      AbiSendMsgIMU_MAG_INT32(MAG_CALIB_UKF_ID, stamp, &calibrated_mag);
    }
  }
}

void mag_calib_update_field(uint8_t __attribute__((unused)) sender_id, struct FloatVect3 *h)
{
  double n = float_vect3_norm(h);
  if (n > 0.01) {
    H.x = (float)(h->x / n);
    H.y = (float)(h->y / n);
    H.z = (float)(h->z / n);
    ahrs_int_set_H(&H);
    PRINT("Updating local magnetic field from geo_mag module (Hx: %4.4f, Hy: %4.4f, Hz: %4.4f)\n", H.x, H.y, H.z);
  }
}

void mag_calib_send_state(void)
{
  DOWNLINK_SEND_PAYLOAD_FLOAT(DefaultChannel, DefaultDevice, TRICAL_STATE_DIM, mag_calib.state);
}

void mag_calib_hotstart_read(void)
{
#if MAG_CALIB_UKF_HOTSTART
  snprintf(mag_hotstart_file_name, 512, "%s", STRINGIFY(MAG_CALIB_UKF_HOTSTART_SAVE_FILE));
  fp = fopen(mag_hotstart_file_name, "r");
  if (fp != NULL) {
    fread(mag_calib.state, sizeof(float), TRICAL_STATE_DIM, fp);
    fread(mag_calib.state_covariance, sizeof(float), TRICAL_STATE_DIM * TRICAL_STATE_DIM, fp);
    fclose(fp);
    PRINT("Loaded initial state from disk:\n"
          "State {%4.2f, %4.2f, %4.2f}\n"
          "      {%4.2f, %4.2f, %4.2f}\n"
          "      {%4.2f, %4.2f, %4.2f}\n"
          "Cov   {%4.2f, %4.2f, ...., %4.2f}\n",
          mag_calib.state[0], mag_calib.state[1],  mag_calib.state[2],
          mag_calib.state[3], mag_calib.state[4],  mag_calib.state[5],
          mag_calib.state[6], mag_calib.state[7],  mag_calib.state[8],
          mag_calib.state_covariance[0], mag_calib.state_covariance[1],
          mag_calib.state_covariance[TRICAL_STATE_DIM * TRICAL_STATE_DIM - 1]
         );
    if (isnan(mag_calib.state[0]) || isnan(mag_calib.state[1]) || isnan(mag_calib.state[2]) || isnan(mag_calib.state[3]) ||
        isnan(mag_calib.state[4]) || isnan(mag_calib.state[5]) || isnan(mag_calib.state[6]) || isnan(mag_calib.state[7])
        || isnan(mag_calib.state[8])) {
      mag_calib_ukf_reset_state = true;
    }
  }

#endif
}

void mag_calib_hotstart_write(void)
{
#if USE_MAGNETOMETER && MAG_CALIB_UKF_HOTSTART
  fp = fopen(mag_hotstart_file_name, "w");
  if (fp != NULL) {
    fwrite(mag_calib.state, sizeof(float), TRICAL_STATE_DIM, fp);
    fwrite(mag_calib.state_covariance, sizeof(float), TRICAL_STATE_DIM * TRICAL_STATE_DIM, fp);
    fclose(fp);
    PRINT("Wrote current state to disk:\n"
          "State {%4.2f, %4.2f, %4.2f}\n"
          "      {%4.2f, %4.2f, %4.2f}\n"
          "      {%4.2f, %4.2f, %4.2f}\n"
          "Cov   {%4.2f, %4.2f, ...., %4.2f}\n",
          mag_calib.state[0], mag_calib.state[1],  mag_calib.state[2],
          mag_calib.state[3], mag_calib.state[4],  mag_calib.state[5],
          mag_calib.state[6], mag_calib.state[7],  mag_calib.state[8],
          mag_calib.state_covariance[0], mag_calib.state_covariance[1],
          mag_calib.state_covariance[TRICAL_STATE_DIM * TRICAL_STATE_DIM - 1]
         );
  }
#endif
}
