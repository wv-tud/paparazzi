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
 * Calibrate magnetometer using UKF
 */

#ifndef MAG_CALIB_UKF_H
#define MAG_CALIB_UKF_H

#include "subsystems/imu.h"

#define PRINT(string,...) fprintf(stderr, "[CALIB_UKF->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

#if !defined MAG_CALIB_UKF_GEO_MAG_TIMEOUT
#define MAG_CALIB_UKF_GEO_MAG_TIMEOUT 0
#endif

#if !defined MAG_CALIB_UKF_NORM
#define MAG_CALIB_UKF_NORM 1.0f
#endif

#if !defined MAG_CALIB_UKF_NOISE_RMS
#define MAG_CALIB_UKF_NOISE_RMS 2e-1f
#endif

#if !defined MAG_CALIB_UKF_HOTSTART
#define MAG_CALIB_UKF_HOTSTART TRUE
#endif

#if !defined MAG_CALIB_UKF_HOTSTART_SAVE_FILE
#define MAG_CALIB_UKF_HOTSTART_SAVE_FILE /data/ftp/internal_000/mag_ukf_calib.txt
#endif

#if !defined MAG_CALIB_UKF_VERBOSE
#define VERBOSE_PRINT(...)
#elif MAG_CALIB_UKF_VERBOSE == TRUE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

extern void mag_calib_ukf_init( struct Imu *_imu );
extern void mag_calib_ukf_run( struct Imu *_imu );
void mag_calib_hotstart_read( void );
void mag_calib_hotstart_write( void );

#endif

