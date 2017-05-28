/*
 * Copyright (C) Freek van Tienen
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
 * @file "modules/computer_vision/cv_ae_awb.h"
 * @author Freek van Tienen
 * Auto exposure and Auto white balancing for the Bebop 1 and 2
 */

#ifndef CV_AE_AWB_H
#define CV_AE_AWB_H

extern float awb_avgU;
extern float awb_avgV;
extern uint32_t awb_nb_pixels;

extern float ae_exposure_gain;
extern uint8_t ae_dark_bins;
extern uint8_t ae_bright_bins;
extern uint8_t ae_middle_index;
extern float ae_bright_ignore;
extern float ae_dark_ignore;
extern float ae_current_level;
extern int8_t awb_offset_u;
extern int8_t awb_offset_v;
extern float awb_gain;

extern uint32_t histogram_plot[255];

extern void cv_ae_awb_init(void);

#ifndef MAX_HIST_Y
#define MAX_HIST_Y 256 - 20
#endif
#ifndef MIN_HIST_Y
#define MIN_HIST_Y 16
#endif

#endif

