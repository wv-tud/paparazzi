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
//#include "subsystems/ahrs/ahrs_int_cmpl_quat.h"
//#include "subsystems/ahrs/ahrs_float_cmpl.h"
#include "subsystems/ins/ins_float_invariant.h"
//#include "subsystems/ahrs/ahrs_float_invariant.h"
#include "filters/low_pass_filter.h"

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

void mag_calib_state_sanity_check(void);
// ABI callback declarations
static void mag_calib_ukf_run(uint8_t sender_id, uint32_t stamp, struct Int32Vect3 *mag);
static void mag_calib_update_field(uint8_t __attribute__((unused)) sender_id, struct FloatVect3 *h);
static void body_to_imu_cb(uint8_t sender_id __attribute__((unused)), struct FloatQuat *q_b2i_f);

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
#define MAG_CALIB_UKF_NOISE_RMS 5.0e-1f
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

#ifndef MAG_CALIB_UKF_AUTOSTART
#define MAG_CALIB_UKF_AUTOSTART FALSE
#endif
PRINT_CONFIG_VAR(MAG_CALIB_UKF_AUTOSTART)

#ifndef MAG_CALIB_UKF_UPDATE_EVERY
#define MAG_CALIB_UKF_UPDATE_EVERY 1
#endif
PRINT_CONFIG_VAR(MAG_CALIB_UKF_UPDATE_EVERY)

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

#ifndef MAG_CALIB_UKF_FULL_3X3
#define MAG_CALIB_UKF_FULL_3X3 FALSE;
#endif
PRINT_CONFIG_VAR(MAG_CALIB_UKF_FULL_3X3)

#ifndef MAG_CALIB_UKF_FILTER_MAG
#define MAG_CALIB_UKF_FILTER_MAG FALSE
#endif
PRINT_CONFIG_VAR(MAG_CALIB_UKF_FILTER_MAG)

#ifndef MAG_CALIB_UKF_FILTER_MAG_CUTOFF
#define MAG_CALIB_UKF_FILTER_MAG_CUTOFF 6.0
#endif
PRINT_CONFIG_VAR(MAG_CALIB_UKF_FILTER_MAG_CUTOFF)

#ifndef MAG_CALIB_UKF_ROTATION_SPEED
#define MAG_CALIB_UKF_ROTATION_SPEED 12.0
#endif
PRINT_CONFIG_VAR(MAG_CALIB_UKF_ROTATION_SPEED)

#ifndef MAG_CALIB_UKF_AUTOSAVE
#define MAG_CALIB_UKF_AUTOSAVE TRUE
#endif
PRINT_CONFIG_VAR(MAG_CALIB_UKF_AUTOSAVE)

bool mag_calib_ukf_full_3x3 = MAG_CALIB_UKF_FULL_3X3;
bool mag_calib_ukf_update_filter = MAG_CALIB_UKF_AUTOSTART;
uint8_t mag_calib_ukf_update_every = MAG_CALIB_UKF_UPDATE_EVERY;
bool mag_calib_ukf_reset_state = false;
bool mag_calib_ukf_send_state = false;
bool mag_calib_ukf_filter_mag = MAG_CALIB_UKF_FILTER_MAG;
bool mag_calib_ukf_rotating = 0;
bool mag_calib_ukf_autosave = MAG_CALIB_UKF_AUTOSAVE;

float mag_calib_ukf_calibration_rotation_speed = MAG_CALIB_UKF_ROTATION_SPEED; // Degrees

struct Int32Vect3 calibrated_mag;

float mag_calib_ukf_noise_rms = MAG_CALIB_UKF_NOISE_RMS;
float mag_calib_ukf_filter_cutoff = MAG_CALIB_UKF_FILTER_MAG_CUTOFF;
float angle_diff_f;
float magneto_psi_f;

Butterworth2LowPass mag_lowpass_filters[3];

float mag_calib_calibrate_threshold_scale = 1.5;
float mag_calib_reset_threshold_scale = 0.4;
float mag_calib_reset_threshold_bias = 0.9;
TRICAL_instance_t mag_calib;
static abi_event mag_ev;
static abi_event h_ev;
static abi_event body_to_imu_ev;
/** body_to_imu rotation */
struct OrientationReps mag_calib_body_to_imu;
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
  AbiBindMsgBODY_TO_IMU_QUAT(MAG_CALIB_UKF_ABI_BIND_ID, &body_to_imu_ev, body_to_imu_cb);
}

static void body_to_imu_cb(uint8_t sender_id __attribute__((unused)),
                           struct FloatQuat *q_b2i_f)
{
  orientationSetQuat_f(&mag_calib_body_to_imu, q_b2i_f);
}

/** Callback function run for every new mag measurement
 */
void mag_calib_ukf_run(uint8_t sender_id, uint32_t stamp, struct Int32Vect3 *mag)
{
  static uint32_t runCount = 0;
  static uint32_t filtCount = 0;
  static float mag_calib_ukf_prev_filter_cutoff = MAG_CALIB_UKF_FILTER_MAG_CUTOFF;
  static float warning_mag_calib_calibrate_threshold = 5.0;
  float measurement[3] = {0.0f, 0.0f, 0.0f};
  float calibrated_measurement[3] = {0.0f, 0.0f, 0.0f};
  if(mag_calib.measurement_noise != mag_calib_ukf_noise_rms){
    TRICAL_noise_set(&mag_calib, mag_calib_ukf_noise_rms);
  }
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
      if(mag_calib_ukf_filter_mag){
        if(filtCount == 0 || mag_calib_ukf_prev_filter_cutoff != mag_calib_ukf_filter_cutoff){
          float tau = 1.0/(2.0 * M_PI * mag_calib_ukf_filter_cutoff);
          float sample_time = 1.0 / 100.0;
          init_butterworth_2_low_pass(&mag_lowpass_filters[0], tau, sample_time, measurement[0]);
          init_butterworth_2_low_pass(&mag_lowpass_filters[1], tau, sample_time, measurement[1]);
          init_butterworth_2_low_pass(&mag_lowpass_filters[2], tau, sample_time, measurement[2]);
          mag_calib_ukf_prev_filter_cutoff = mag_calib_ukf_filter_cutoff;
        }else{
          update_butterworth_2_low_pass(&mag_lowpass_filters[0], measurement[0]);
          update_butterworth_2_low_pass(&mag_lowpass_filters[1], measurement[1]);
          update_butterworth_2_low_pass(&mag_lowpass_filters[2], measurement[2]);
          measurement[0] = mag_lowpass_filters[0].o[0];
          measurement[1] = mag_lowpass_filters[1].o[0];
          measurement[2] = mag_lowpass_filters[2].o[0];
        }
        filtCount++;
      }else{
        if(filtCount != 0){
          filtCount = 0;
        }
      }
      /** Update magnetometer UKF **/
      if(mag_calib_ukf_update_filter && !(runCount % mag_calib_ukf_update_every)){
        if(!mag_calib_ukf_full_3x3){
          /*
           * Norm only:
           */
          TRICAL_estimate_update(&mag_calib, measurement);//, calibrated_measurement); // Third argument not used for norm only
          if(warning_mag_calib_calibrate_threshold != 5.0){
            warning_mag_calib_calibrate_threshold = 5.0;
          }
        }else{
          /*
           * Full 3x3 support:
           */
          float expected_mag_field[3] = {0.0f, 0.0f, 0.0f};
          struct FloatQuat *body_quat = stateGetNedToBodyQuat_f();
          struct FloatVect3 expected_measurement;
          float_quat_vmult(&expected_measurement, body_quat, &H);
          expected_mag_field[0] = expected_measurement.x;
          expected_mag_field[1] = expected_measurement.y;
          expected_mag_field[2] = expected_measurement.z;
          TRICAL_estimate_update(&mag_calib, measurement);//, expected_mag_field);
          if(warning_mag_calib_calibrate_threshold != 5.0){
            warning_mag_calib_calibrate_threshold = 5.0;
          }
        }
      }
      /* Sanity check on the state */
      mag_calib_state_sanity_check();
      /** Calibrate measurement */
      TRICAL_measurement_calibrate(&mag_calib, measurement, calibrated_measurement);
      float measurement_norm = sqrtf(powf(calibrated_measurement[0], 2.0) + powf(calibrated_measurement[1], 2.0) + powf(calibrated_measurement[2], 2.0));
      /** Check the norm of the calibrated measurement data */
      if(!mag_calib_ukf_update_filter && (measurement_norm > mag_calib_calibrate_threshold_scale || measurement_norm < 1.0/mag_calib_calibrate_threshold_scale)){
        if(mag_calib_calibrate_threshold_scale != warning_mag_calib_calibrate_threshold){
          warning_mag_calib_calibrate_threshold = mag_calib_calibrate_threshold_scale;
          char data[200];
          snprintf(data, 200, "%s: Please recalibrate magnetometer (norm %0.2f)", AIRFRAME_NAME, measurement_norm);
          printf("%s\n",data);
          DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, strlen(data), data);
        }
      }
      /* Convert to int32_t */
      calibrated_mag.x = (int32_t) MAG_BFP_OF_REAL(calibrated_measurement[0]);
      calibrated_mag.y = (int32_t) MAG_BFP_OF_REAL(calibrated_measurement[1]);
      calibrated_mag.z = (int32_t) MAG_BFP_OF_REAL(calibrated_measurement[2]);
      /** Save calibrated result in global imu struct **/
      imu.mag.x = calibrated_mag.x;
      imu.mag.y = calibrated_mag.y;
      imu.mag.z = calibrated_mag.z;
      /** Forward calibrated data */
      AbiSendMsgIMU_MAG_INT32(MAG_CALIB_UKF_ID, stamp, &calibrated_mag);
      /** Calculate for logging magneto psi */
      // Convert to body frame
      struct Int32Vect3 calibrated_body_mag;
      struct Int32RMat *mag_calib_body_to_imu_rmat = orientationGetRMat_i(&mag_calib_body_to_imu);
      int32_rmat_transp_vmult(&calibrated_body_mag, mag_calib_body_to_imu_rmat, &calibrated_mag);
      struct FloatEulers *e = stateGetNedToBodyEulers_f();
      float cphi   = cosf(e->phi);
      float sphi   = sinf(e->phi);
      float ctheta = cosf(e->theta);
      float stheta = sinf(e->theta);
      float mn = ctheta * MAG_FLOAT_OF_BFP(calibrated_body_mag.x) + sphi * stheta * MAG_FLOAT_OF_BFP(calibrated_body_mag.y) + cphi * stheta * MAG_FLOAT_OF_BFP(calibrated_body_mag.z);
      float me =     0. * MAG_FLOAT_OF_BFP(calibrated_body_mag.x) + cphi          * MAG_FLOAT_OF_BFP(calibrated_body_mag.y) - sphi          * MAG_FLOAT_OF_BFP(calibrated_body_mag.z);
      magneto_psi_f = -atan2f(me, mn) + atan2(AHRS_H_Y, AHRS_H_X);
      if (magneto_psi_f > M_PI) { magneto_psi_f -= 2.*M_PI; } if (magneto_psi_f < -M_PI) { magneto_psi_f += 2.*M_PI; }
      //printf("mag_psi_f: %0.2f\n", magneto_psi_f / M_PI * 180);
      /** Debug print */
      VERBOSE_PRINT("magnetometer measurement (x: %4.2f  y: %4.2f  z: %4.2f) norm: %4.2f\n", measurement[0], measurement[1],
                    measurement[2], hypot(hypot(measurement[0], measurement[1]), measurement[2]));
      VERBOSE_PRINT("calibrated   measurement (x: %4.2f  y: %4.2f  z: %4.2f) norm: %4.2f\n\n", calibrated_measurement[0],
                    calibrated_measurement[1], calibrated_measurement[2], hypot(hypot(calibrated_measurement[0], calibrated_measurement[1]),
                        calibrated_measurement[2]));
    }
  }
}

void mag_calib_update_field(uint8_t __attribute__((unused)) sender_id, struct FloatVect3 *h)
{
  H = *h;
  PRINT("Updating local magnetic field from geo_mag module (Hx: %4.4f, Hy: %4.4f, Hz: %4.4f)\n", H.x, H.y, H.z);
}

void mag_calib_rotate_toggle( bool rotate ){
  if(rotate){
    mag_calib_ukf_rotating = 1;
  }else{
    mag_calib_ukf_rotating = 0;
    mag_calib_ukf_calibration_rotation_speed *= -1;
  }
}

bool mag_calib_get_rotate_status( void ){
  return mag_calib_ukf_rotating;
}

void mag_calib_rotate( void ){
  static float start_heading = 0.0;
  static bool started_rotating = false;
  struct FloatEulers *e = stateGetNedToBodyEulers_f();
  if(mag_calib_ukf_rotating){
    float angle_diff = fabs(e->psi - start_heading);
    angle_diff = fmin(angle_diff, fabs(angle_diff - 2 * M_PI));
    if(started_rotating && angle_diff < fabs(mag_calib_ukf_calibration_rotation_speed)){
      mag_calib_ukf_rotating = false;
      started_rotating = false;
      nav_set_heading_rad(start_heading);
    }else{
      nav_set_heading_rad(e->psi + RadOfDeg(mag_calib_ukf_calibration_rotation_speed));
      if(angle_diff > fmin(100.0, 1.5 * fabs(mag_calib_ukf_calibration_rotation_speed))){
        started_rotating = true;
      }
    }
  }else{
    start_heading = e->psi;
    started_rotating = false;
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
#if TRICAL_STATE_DIM == 9
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
#elif TRICAL_STATE_DIM == 12
    PRINT("Loaded initial state from disk:\n"
        "State {%4.2f, %4.2f, %4.2f}\n"
        "      {%4.2f, %4.2f, %4.2f}\n"
        "      {%4.2f, %4.2f, %4.2f}\n"
        "      {%4.2f, %4.2f, %4.2f}\n"
        "Cov   {%4.2f, %4.2f, ...., %4.2f}\n",
        mag_calib.state[0], mag_calib.state[1],  mag_calib.state[2],
        mag_calib.state[3], mag_calib.state[4],  mag_calib.state[5],
        mag_calib.state[6], mag_calib.state[7],  mag_calib.state[8],
        mag_calib.state[9], mag_calib.state[10],  mag_calib.state[11],
        mag_calib.state_covariance[0], mag_calib.state_covariance[1],
        mag_calib.state_covariance[TRICAL_STATE_DIM * TRICAL_STATE_DIM - 1]
    );
#endif
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
  if(mag_calib_ukf_autosave){
    fp = fopen(mag_hotstart_file_name, "w");
    if (fp != NULL) {
      fwrite(mag_calib.state, sizeof(float), TRICAL_STATE_DIM, fp);
      fwrite(mag_calib.state_covariance, sizeof(float), TRICAL_STATE_DIM * TRICAL_STATE_DIM, fp);
      fclose(fp);
#if TRICAL_STATE_DIM == 9
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
#elif TRICAL_STATE_DIM == 12
    PRINT("Wrote current state to disk:\n"
        "State {%4.2f, %4.2f, %4.2f}\n"
        "      {%4.2f, %4.2f, %4.2f}\n"
        "      {%4.2f, %4.2f, %4.2f}\n"
        "      {%4.2f, %4.2f, %4.2f}\n"
        "Cov   {%4.2f, %4.2f, ...., %4.2f}\n",
        mag_calib.state[0], mag_calib.state[1],  mag_calib.state[2],
        mag_calib.state[3], mag_calib.state[4],  mag_calib.state[5],
        mag_calib.state[6], mag_calib.state[7],  mag_calib.state[8],
        mag_calib.state[9], mag_calib.state[10],  mag_calib.state[11],
        mag_calib.state_covariance[0], mag_calib.state_covariance[1],
        mag_calib.state_covariance[TRICAL_STATE_DIM * TRICAL_STATE_DIM - 1]
    );
#endif
    }
  }
#endif
}

void mag_calib_state_sanity_check(void){
  static uint8_t warning_mag_calib_calibrate_scale_x = 0;
  static uint8_t warning_mag_calib_calibrate_scale_y = 0;
  static uint8_t warning_mag_calib_calibrate_scale_z = 0;
  static uint8_t warning_mag_calib_calibrate_bias_x = 0;
  static uint8_t warning_mag_calib_calibrate_bias_y = 0;
  static uint8_t warning_mag_calib_calibrate_bias_z = 0;
  /* Sanity check on the scale
  if(fabs(mag_calib.state[3]) > mag_calib_reset_threshold_scale){
    if(warning_mag_calib_calibrate_scale_x == 0){
      warning_mag_calib_calibrate_scale_x = 1;
      char data[200];
      snprintf(data, 200, "%s: Please reset magnetometer filter (x scale: %0.2f)", AIRFRAME_NAME, mag_calib.state[3]);
      printf("%s\n",data);
      DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, strlen(data), data);
    }
  }
  if(fabs(mag_calib.state[7]) > mag_calib_reset_threshold_scale){
    if(warning_mag_calib_calibrate_scale_y == 0){
      warning_mag_calib_calibrate_scale_y = 1;
      char data[200];
      snprintf(data, 200, "%s: Please reset magnetometer filter (y scale: %0.2f)", AIRFRAME_NAME, mag_calib.state[7]);
      printf("%s\n",data);
      DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, strlen(data), data);
    }
  }
  if(fabs(mag_calib.state[11]) > mag_calib_reset_threshold_scale){
    if(warning_mag_calib_calibrate_scale_z == 0){
      warning_mag_calib_calibrate_scale_z = 1;
      char data[200];
      snprintf(data, 200, "%s: Please reset magnetometer filter (z scale: %0.2f)", AIRFRAME_NAME, mag_calib.state[11]);
      printf("%s\n",data);
      DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, strlen(data), data);
    }
  }*/
  if(fabs(mag_calib.state[3]) > mag_calib_reset_threshold_scale){
      if(warning_mag_calib_calibrate_scale_x == 0){
        warning_mag_calib_calibrate_scale_x = 1;
        char data[200];
        snprintf(data, 200, "%s: Please reset magnetometer filter (x scale: %0.2f)", AIRFRAME_NAME, mag_calib.state[3]);
        printf("%s\n",data);
        DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, strlen(data), data);
      }
    }
    if(fabs(mag_calib.state[6]) > mag_calib_reset_threshold_scale){
      if(warning_mag_calib_calibrate_scale_y == 0){
        warning_mag_calib_calibrate_scale_y = 1;
        char data[200];
        snprintf(data, 200, "%s: Please reset magnetometer filter (y scale: %0.2f)", AIRFRAME_NAME, mag_calib.state[6]);
        printf("%s\n",data);
        DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, strlen(data), data);
      }
    }
    if(fabs(mag_calib.state[8]) > mag_calib_reset_threshold_scale){
      if(warning_mag_calib_calibrate_scale_z == 0){
        warning_mag_calib_calibrate_scale_z = 1;
        char data[200];
        snprintf(data, 200, "%s: Please reset magnetometer filter (z scale: %0.2f)", AIRFRAME_NAME, mag_calib.state[8]);
        printf("%s\n",data);
        DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, strlen(data), data);
      }
    }
  /* Sanity check on the bias */
  if(fabs(mag_calib.state[0]) > mag_calib_reset_threshold_bias){
    if(warning_mag_calib_calibrate_bias_x == 0){
      warning_mag_calib_calibrate_bias_x = 1;
      char data[200];
      snprintf(data, 200, "%s: Please reset magnetometer filter (x bias: %0.2f)", AIRFRAME_NAME, mag_calib.state[0]);
      printf("%s\n",data);
      DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, strlen(data), data);
    }
  }
  if(fabs(mag_calib.state[1]) > mag_calib_reset_threshold_bias){
    if(warning_mag_calib_calibrate_bias_y == 0){
      warning_mag_calib_calibrate_bias_y = 1;
      char data[200];
      snprintf(data, 200, "%s: Please reset magnetometer filter (y bias: %0.2f)", AIRFRAME_NAME, mag_calib.state[1]);
      printf("%s\n",data);
      DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, strlen(data), data);
    }
  }
  if(fabs(mag_calib.state[2]) > mag_calib_reset_threshold_bias){
    if(warning_mag_calib_calibrate_bias_z == 0){
      warning_mag_calib_calibrate_bias_z = 1;
      char data[200];
      snprintf(data, 200, "%s: Please reset magnetometer filter (z bias: %0.2f)", AIRFRAME_NAME, mag_calib.state[2]);
      printf("%s\n",data);
      DOWNLINK_SEND_INFO_MSG(DefaultChannel, DefaultDevice, strlen(data), data);
    }
  }
}
