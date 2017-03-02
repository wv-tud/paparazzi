/*
 * Copyright (C) Wilco Vlenterie
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
 * @file "modules/computer_vision/autoswarm//autoswarm.h"
 * @author Wilco Vlenterie
 * Autonomous bebop swarming module based on vision
 */

#ifndef AS_H
#define AS_H

#include "std.h"
#include "stdbool.h"
#include <math.h>                                           ///< Used for sin/cos/tan/M_PI/abs
#include <time.h>                                           ///< Used to write time to results.txt
#include <stdio.h>                                          ///< Used for printing
#include <errno.h>                                          ///< Used for error handling

// Global options definitions
#define AS_POINT             0
#define AS_BUCKET            1
#define AS_CIRCLE_CW         2
#define AS_CIRCLE_CC         3
/// Camera options definitions
#define AS_CAM_FORWARD       1
#define AS_CAM_GLOBAL        2

extern double   AS_CIRCLE_R;
extern double   AS_SEPARATION;
extern int      AS_ATTRACTOR;
extern double   AS_AMAX;
extern double   AS_VMAX;
extern double   AS_YAWRATEMAX;
extern double   AS_GLOBAL;
extern int      AS_MODE;
extern double   AS_E;
extern double   AS_EPS;

// Initialize global attractor
struct originPoint { double cx; double cy; double cz;};
struct originPoint globalOrigin;
static inline void setGlobalOrigin  (double x, double y, double z){ globalOrigin.cx = x; globalOrigin.cy = y; globalOrigin.cz = z;};
static inline bool setGlobalMode    (int mode){ AS_ATTRACTOR = mode; return false; };
static inline bool setSwarmMode     (int mode){ AS_MODE = mode; return false; };

void autoswarm_init                 ( void );
struct image_t* autoswarm_run       ( struct image_t* img );
bool amIhome                        ( void );

#endif
