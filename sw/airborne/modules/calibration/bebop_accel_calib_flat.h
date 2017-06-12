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
 * @file "modules/bebop_accel_calib_flat/bebop_accel_calib_flat.h"
 * @author w.vlenterie
 * Calibrate the bebop accelerometer from a flat surface
 */

#ifndef BEBOP_ACCEL_CALIB_FLAT_H
#define BEBOP_ACCEL_CALIB_FLAT_H

#include <stdbool.h>

void bebop_accel_calib_init( void );
void bebop_set_accel_neutral( void );
void bebop_send_accel_neutral( void );

extern bool settings_send_accel_neutral;
extern bool  settings_calibration_running;
extern float settings_calibration_time;

#endif

