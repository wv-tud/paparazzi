/*
 * Copyright (C) Wilco Vlenterie (wv-tud)
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
 * @file "modules/computer_vision//cv_bebop_camera_stabilization.c"
 * @author Wilco Vlenterie (wv-tud)
 * Active random sampling colour filter
 */
#include "std.h"

#define CAM_STAB_MAX_OBJECTS   30  // Maximum nr of objects


extern double       CAM_STAB_VIEW_R;
extern double       default_k;
extern double       default_6th_o;
extern double       default_2nd_o;
extern float        angleOfView;

#ifdef __cplusplus
extern "C" {
#endif
#include <state.h>                              // C header used for state functions and data
void bebop_camera_stabilization_init(void);
void bebop_camera_stabilization(char* buff, uint16_t width, uint16_t height, struct FloatEulers* curEulerAngles);
/** Fisheye + Perspective correction **/
void             point2pixel        (double x_out, double y_out, double *x_in, double *y_in);
void             pixel2point        (double x_in, double y_in, double *x_out, double *y_out);
/** Conversion from angles to output frame point **/
void             angles2point        (double xAngle, double yAngle, double *x_out, double * y_out);
void             point2angles        (double x_out, double y_out, double *xAngle, double *yAngle);

#ifdef __cplusplus
}
#endif
