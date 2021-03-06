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
 * @file "modules/calibration/mag_calib_ukf.h"
 * @author w.vlenterie
 * Calibrate the magnetometer using an unscented kalman filter
 * For more information please visit the following links:
 *   - https://github.com/sfwa/trical
 *   - http://au.tono.my/log/20131213-trical-magnetometer-calibration.html
 *   - http://www.acsu.buffalo.edu/~johnc/mag_cal05.pdf
 */

#ifndef MAG_CALIB_UKF_H
#define MAG_CALIB_UKF_H

#include "std.h"
#include "TRICAL.h"

// Used for magneto calibration block
#include "firmwares/rotorcraft/stabilization.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude_rc_setpoint.h"
extern float mag_calib_ukf_calibration_rotation_speed;

// setting to request state reset
extern bool mag_calib_ukf_update_filter;
extern uint8_t mag_calib_ukf_update_every;
extern bool mag_calib_ukf_reset_state;
extern bool mag_calib_ukf_send_state;
extern bool mag_calib_ukf_filter_mag;
extern float mag_calib_ukf_filter_cutoff;
extern bool mag_calib_ukf_full_3x3;
extern bool mag_calib_ukf_autosave;
extern float mag_calib_ukf_noise_rms;

extern float angle_diff_f;
extern float magneto_psi_f;

extern float mag_calib_calibrate_threshold_scale;
extern float mag_calib_reset_threshold_scale;
extern float mag_calib_reset_threshold_bias;
extern TRICAL_instance_t mag_calib;

void mag_calib_ukf_init(void);
void mag_calib_hotstart_write(void);
void mag_calib_hotstart_read(void);
void mag_calib_send_state(void);
void mag_calib_rotate_toggle( bool rotate );
void mag_calib_rotate( void );
bool mag_calib_get_rotate_status( void );

#endif
