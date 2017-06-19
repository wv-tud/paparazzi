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
 * @file "modules/computer_vision//cv_active_random_filter.c"
 * @author Wilco Vlenterie (wv-tud)
 * Active random sampling colour filter
 */

#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/cv_frame_plotter_wrapper.h"
#include "modules/computer_vision/cv_frame_plotter.h"

#ifndef FRAME_PLOTTER_CAMERA
#define FRAME_PLOTTER_CAMERA front_camera
#endif
PRINT_CONFIG_VAR(FRAME_PLOTTER_CAMERA)

struct image_t* cv_frame_plotter_wrapper_func(struct image_t* img);
struct image_t* cv_frame_plotter_wrapper_func(struct image_t* img)
{
  cv_frame_plotter_func((char*) img->buf, (uint16_t) img->w, (uint16_t) img->h);
	return NULL;
}

void cv_frame_plotter_wrapper_init() {
  cv_frame_plotter_init();
  cv_add_to_device(&FRAME_PLOTTER_CAMERA, cv_frame_plotter_wrapper_func);
}
