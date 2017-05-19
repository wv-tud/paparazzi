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
 * @file "modules/computer_vision/cv_ae_awb.c"
 * @author Freek van Tienen
 * Auto exposure and Auto white balancing for the Bebop 1 and 2
 */

#include "modules/computer_vision/cv.h"
#include "modules/computer_vision/cv_ae_awb.h"
#include "boards/bebop/mt9f002.h"
#include "lib/isp/libisp.h"
#include <stdio.h>

#ifndef CV_AE_AWB_CAMERA
#define CV_AE_AWB_CAMERA front_camera
#endif
PRINT_CONFIG_VAR(CV_AE_AWB_CAMERA)

#ifndef CV_AE_AWB_VERBOSE
#define CV_AE_AWB_VERBOSE 0
#endif
PRINT_CONFIG_VAR(CV_AE_AWB_VERBOSE)

#define PRINT(string,...) fprintf(stderr, "[cv_ae_awb->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

#if CV_AE_AWB_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

#ifndef CV_AE_AWB_MIN_GAINS
#define CV_AE_AWB_MIN_GAINS 0.5
#endif
PRINT_CONFIG_VAR(CV_AE_AWB_MIN_GAINS)

#ifndef CV_AE_AWB_MAX_GAINS
#define CV_AE_AWB_MAX_GAINS 64.0
#endif
PRINT_CONFIG_VAR(CV_AE_AWB_MAX_GAINS)

#ifndef CV_AE_MIDDLE_INDEX
#define CV_AE_MIDDLE_INDEX 95
#endif
PRINT_CONFIG_VAR(CV_AE_MIDDLE_INDEX)

#ifndef CV_AE_DARK_IGNORE
#define CV_AE_DARK_IGNORE 0.5
#endif
PRINT_CONFIG_VAR(CV_AE_DARK_IGNORE)

#ifndef CV_AE_BRIGHT_IGNORE
#define CV_AE_BRIGHT_IGNORE 0.25
#endif
PRINT_CONFIG_VAR(CV_AE_BRIGHT_IGNORE)

#ifndef CV_AE_DARK_BINS
#define CV_AE_DARK_BINS 65
#endif
PRINT_CONFIG_VAR(CV_AE_DARK_BINS)

#ifndef CV_AE_BRIGHT_BINS
#define CV_AE_BRIGHT_BINS 65
#endif
PRINT_CONFIG_VAR(CV_AE_BRIGHT_BINS)

#ifndef CV_AE_AWB_GAIN_SCHEDULING
#define CV_AE_AWB_GAIN_SCHEDULING 1
#endif
PRINT_CONFIG_VAR(CV_AE_AWB_GAIN_SCHEDULING)

#if CV_AE_AWB_GAIN_SCHEDULING
#ifndef CV_AE_AWB_GAIN_SCHEDULING_TARGET
#define CV_AE_AWB_GAIN_SCHEDULING_TARGET 15.0
#endif
PRINT_CONFIG_VAR(CV_AE_AWB_GAIN_SCHEDULING_TARGET)

#ifndef CV_AE_AWB_GAIN_SCHEDULING_TOLERANCE
#define CV_AE_AWB_GAIN_SCHEDULING_TOLERANCE 2.5
#endif
PRINT_CONFIG_VAR(CV_AE_AWB_GAIN_SCHEDULING_TOLERANCE)

#ifndef CV_AE_AWB_GAIN_SCHEDULING_STEP
#define CV_AE_AWB_GAIN_SCHEDULING_STEP 0.0025
#endif
PRINT_CONFIG_VAR(CV_AE_AWB_GAIN_SCHEDULING_STEP)

#ifndef CV_AE_AWB_GAIN_SCHEDULING_MARGIN
#define CV_AE_AWB_GAIN_SCHEDULING_MARGIN 1.5
#endif
PRINT_CONFIG_VAR(CV_AE_AWB_GAIN_SCHEDULING_MARGIN)
#endif

#define MAX_HIST_Y 256-20
#define MIN_HIST_Y 16

float   ae_bright_ignore = CV_AE_BRIGHT_IGNORE;
float   ae_dark_ignore   = CV_AE_DARK_IGNORE;
uint8_t ae_middle_index  = CV_AE_MIDDLE_INDEX;

uint8_t ae_dark_bins     = CV_AE_DARK_BINS;
uint8_t ae_bright_bins   = CV_AE_BRIGHT_BINS;

float ae_current_level   = 0.0;
float awb_avgU           = 0.0;
float awb_avgV           = 0.0;
uint32_t awb_nb_pixels   = 0;

int8_t gs_adjustDir      = 0;
bool gs_gains_maxed      = FALSE;
bool gs_gains_minned     = FALSE;

struct image_t *cv_ae_awb_periodic(struct image_t *img);
struct image_t *cv_ae_awb_periodic(struct image_t *img)
{
  struct isp_yuv_stats_t yuv_stats;
  if (isp_get_statistics_yuv(&yuv_stats) == 0 && yuv_stats.nb_valid_Y != 0) {
    /*
     *      Auto-exposure (Histogram median centering)
     */
    // Calculate the cummulative histogram based on the histogram
    uint32_t cdf[MAX_HIST_Y];
    cdf[MIN_HIST_Y - 1] = 0;
    for (int i = MIN_HIST_Y; i < MAX_HIST_Y; i++) {
      cdf[i] = cdf[i - 1] + yuv_stats.ae_histogram_Y[i];
    }
    // Calculate the indices of the dark and bright bins
    uint8_t dark_index   = MIN_HIST_Y + ae_dark_bins;
    uint8_t bright_index = MAX_HIST_Y - ae_bright_bins;
    // Calculate the median number of pixels ignoring the dark_ignore % and bright-ignore %
    uint32_t median_pixels = (uint32_t) round(((1 - ae_dark_ignore) * cdf[dark_index] +
                             (cdf[bright_index - 1] - cdf[dark_index]) + (1 - ae_bright_ignore) * (cdf[MAX_HIST_Y - 1] - cdf[bright_index - 1])) /
                             2.0f);
    // Find the level that contains the median
    uint32_t current_pixels = 0;
    ae_current_level = MIN_HIST_Y - 1.0;
    while (current_pixels < median_pixels && ae_current_level < MAX_HIST_Y) {
      ae_current_level++;
      if (ae_current_level <= dark_index) {
        // In dark bin - ignore dark_ignore %
        current_pixels += (1 - ae_dark_ignore) * yuv_stats.ae_histogram_Y[(uint8_t) ae_current_level];
      } else if (ae_current_level >= bright_index) {
        // In bright bin - ignore bright_ignore %
        current_pixels += (1 - ae_bright_ignore) * yuv_stats.ae_histogram_Y[(uint8_t) ae_current_level];
      } else {
        // In centre bin - 100%
        current_pixels += yuv_stats.ae_histogram_Y[(uint8_t) ae_current_level];
      }
    }
    if (yuv_stats.ae_histogram_Y[(uint8_t) ae_current_level] > 0) {
      // Calculate decimal level
      ae_current_level -= (current_pixels - median_pixels) / ((float) yuv_stats.ae_histogram_Y[(uint8_t) ae_current_level]);
    }
    // that level is supposed to be 'middle_index'
    float adjustment = pow(ae_middle_index / ae_current_level, 0.125);
    Bound(adjustment, 1 / 16.0f, 16.0f);
    // Calculate exposure based on adjustment
    mt9f002.target_exposure = mt9f002.target_exposure * adjustment;
    Bound(mt9f002.target_exposure, 1 / 128.0f, 45.0f);
    mt9f002_set_exposure(&mt9f002);
    // Verbose prints
    VERBOSE_PRINT("AE lvl: target %d, actual %3.2f (%d pixels)\n", ae_middle_index, ae_current_level, yuv_stats.nb_valid_Y);
    VERBOSE_PRINT("AE exp: target %5.2f ms, real %5.2f ms (%f)\n", mt9f002.target_exposure, mt9f002.real_exposure,
                  adjustment);
    /*
     *      Auto white-balance (Robust Automatic White Balance Algorithm using Gray Color Points in Images - Huo et al.)
     */
    if (yuv_stats.awb_nb_grey_pixels > 0) {
      awb_nb_pixels     = yuv_stats.awb_nb_grey_pixels;
      awb_avgU          = (yuv_stats.awb_sum_U) / ((float) yuv_stats.awb_nb_grey_pixels) - 128.0;
      awb_avgV          = (yuv_stats.awb_sum_V) / ((float) yuv_stats.awb_nb_grey_pixels) - 128.0;
      VERBOSE_PRINT("avgU = %d / %d - 128.0 = %f\n", yuv_stats.awb_sum_U, yuv_stats.awb_nb_grey_pixels, awb_avgU);
      VERBOSE_PRINT("avgV = %d / %d - 128.0 = %f\n", yuv_stats.awb_sum_V, yuv_stats.awb_nb_grey_pixels, awb_avgV);
      // |B| = |Y| + |U|  &&   |R| = |Y| + |V|
      // ideal:
      // |U| = |V| = 0  &&  |B| = |R| = |Y|
      // so:
      // gain_blue *= |Y| / (|Y| + |U|)  --> ^0.25 in order to make less agressive updates
      // gain_red  *= |Y| / (|Y| + |V|)  --> ^0.25 in order to make less agressive updates
      mt9f002.gain_blue *= pow(yuv_stats.awb_sum_Y / ((float)(yuv_stats.awb_sum_Y + yuv_stats.awb_sum_U - 128 *
                               yuv_stats.awb_nb_grey_pixels)), 0.25);
      mt9f002.gain_red  *= pow(yuv_stats.awb_sum_Y / ((float)(yuv_stats.awb_sum_Y + yuv_stats.awb_sum_V - 128 *
                               yuv_stats.awb_nb_grey_pixels)), 0.25);
      /*
       *          Gain scheduling
       */
#if CV_AE_AWB_GAIN_SCHEDULING
      // Let's try to center the exposure, that way we'll be able to account for fast changes in lightness
      if (!gs_gains_maxed
          && (mt9f002.target_exposure > (CV_AE_AWB_GAIN_SCHEDULING_TARGET + CV_AE_AWB_GAIN_SCHEDULING_TOLERANCE)
              || (mt9f002.target_exposure > CV_AE_AWB_GAIN_SCHEDULING_TARGET && gs_adjustDir == 1))) {
        gs_gains_minned = FALSE;
        gs_adjustDir = +1;
      } else if (!gs_gains_minned
                 && (mt9f002.target_exposure < (CV_AE_AWB_GAIN_SCHEDULING_TARGET - CV_AE_AWB_GAIN_SCHEDULING_TOLERANCE)
                     || (mt9f002.target_exposure < CV_AE_AWB_GAIN_SCHEDULING_TARGET && gs_adjustDir == -1))) {
        gs_gains_maxed = FALSE;
        gs_adjustDir = -1;
      } else {
        gs_adjustDir = 0;
      }
      mt9f002.gain_blue     *= 1 + gs_adjustDir * CV_AE_AWB_GAIN_SCHEDULING_STEP;
      mt9f002.gain_red      *= 1 + gs_adjustDir * CV_AE_AWB_GAIN_SCHEDULING_STEP;
      mt9f002.gain_green1   *= 1 + gs_adjustDir * CV_AE_AWB_GAIN_SCHEDULING_STEP;
      mt9f002.gain_green2   *= 1 + gs_adjustDir * CV_AE_AWB_GAIN_SCHEDULING_STEP;
#endif
      Bound(mt9f002.gain_blue,    CV_AE_AWB_MIN_GAINS, CV_AE_AWB_MAX_GAINS);
      Bound(mt9f002.gain_red,     CV_AE_AWB_MIN_GAINS, CV_AE_AWB_MAX_GAINS);
      Bound(mt9f002.gain_green1,  CV_AE_AWB_MIN_GAINS, CV_AE_AWB_MAX_GAINS);
      Bound(mt9f002.gain_green2,  CV_AE_AWB_MIN_GAINS, CV_AE_AWB_MAX_GAINS);
      // Check to see if gains have reached max level
      if (mt9f002.gain_blue >= CV_AE_AWB_MAX_GAINS / CV_AE_AWB_GAIN_SCHEDULING_MARGIN
          || mt9f002.gain_red >= CV_AE_AWB_MAX_GAINS / CV_AE_AWB_GAIN_SCHEDULING_MARGIN
          || mt9f002.gain_green1 >= CV_AE_AWB_MAX_GAINS / CV_AE_AWB_GAIN_SCHEDULING_MARGIN) {
        gs_gains_maxed = TRUE;
      }
      // Check to see if gains have reached min level
      if (mt9f002.gain_blue <= CV_AE_AWB_MIN_GAINS * CV_AE_AWB_GAIN_SCHEDULING_MARGIN
          || mt9f002.gain_red <= CV_AE_AWB_MIN_GAINS * CV_AE_AWB_GAIN_SCHEDULING_MARGIN
          || mt9f002.gain_green1 <= CV_AE_AWB_MIN_GAINS * CV_AE_AWB_GAIN_SCHEDULING_MARGIN) {
        gs_gains_minned = TRUE;
      }
      // Set gains
      mt9f002_set_gains(&mt9f002);
    } else {
      VERBOSE_PRINT("Error: 0 nb_grey_pixels\n");
    }
  } else {
    VERBOSE_PRINT("Error: 0 valid Y pixels\n");
  }
  return NULL;
}

void cv_ae_awb_init(void)
{
  cv_add_to_device(&CV_AE_AWB_CAMERA, cv_ae_awb_periodic);
}
