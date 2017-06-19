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
 * @file "modules/computer_vision//cv_active_random_filter.h"
 * @author Wilco Vlenterie (wv-tud)
 * Active random sampling colour filter
 */

#ifndef CV_FRAME_PLOTTER_WRAPPER_H
#define CV_FRAME_PLOTTER_WRAPPER_H

extern bool cv_frame_plotter_show_as_totv;
extern bool cv_frame_plotter_show_arf_ball_circles;
extern bool cv_frame_plotter_show_arf_gate_corners;
extern bool cv_frame_plotter_show_arf_obj_coords;
extern bool cv_frame_plotter_show_arf_obj_distance;
extern bool cv_frame_plotter_show_arf_stats;
extern bool cv_frame_plotter_show_ae_awb_info;
extern bool cv_frame_plotter_show_ae_awb_histogram;
extern bool cv_frame_plotter_show_fps;

void cv_frame_plotter_wrapper_init(void);

#endif

