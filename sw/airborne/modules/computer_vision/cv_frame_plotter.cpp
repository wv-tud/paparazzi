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

#define BOARD_CONFIG "boards/bebop.h"               ///< Define which board

#include "cv_frame_plotter.h"
#include <ctime>
#include <vector>
#include "cv_frame_plotter_wrapper.h"
#include "active_random_filter.h"

extern "C" {
#include "boards/bebop.h"                       ///< C header used for bebop specific settings
#include <state.h>                              ///< C header used for state functions and data
#include <sys/time.h>                           ///< C header used for system time functions and data
#include "mcu_periph/sys_time.h"                ///< C header used for PPRZ time functions and data
#include <stdio.h>

#include "cv_bebop_camera_stabilization.h"
#include "modules/computer_vision/cv_image_pose.h"
#include "modules/computer_vision/cv_ae_awb.h"
#include "modules/autoswarm/autoswarm.h"
}

using namespace std;
#include <opencv2/core/core.hpp>                    ///< Load openCV
#include <opencv2/imgproc/imgproc.hpp>              ///< Load openCV image processing library
using namespace cv;


#define PRINT(string,...) fprintf(stderr, "[FRAME_PLOTTER->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

#define ARF_VERBOSE FALSE
#if ARF_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

#define xSign(x) ( ( x ) >= ( 0 ) ? ( 1 ) : ( -1 ) )

#ifndef FP_SHOW_TOTV
#define FRAME_PLOTTER_SHOW_TOTV 1 ///< Show the desired velocity vector from the autoswarm module
#endif
PRINT_CONFIG_VAR(FRAME_PLOTTER_SHOW_TOTV)

#ifndef FRAME_PLOTTER_BALL_CIRCLES
#define FRAME_PLOTTER_BALL_CIRCLES 1 ///< Draw circles around balls from the cv_active_random_filter module
#endif
PRINT_CONFIG_VAR(FRAME_PLOTTER_BALL_CIRCLES)

#ifndef FRAME_PLOTTER_GATE_CORNERS
#define FRAME_PLOTTER_GATE_CORNERS 0 ///< Plot corner points of Gates from the cv_active_random_filter module
#endif
PRINT_CONFIG_VAR(FRAME_PLOTTER_GATE_CORNERS)

#ifndef FRAME_PLOTTER_PLOT_COORDS
#define FRAME_PLOTTER_PLOT_COORDS 1 ///< Plot the coordinates of objects on frame from the cv_active_random_filter module
#endif
PRINT_CONFIG_VAR(FRAME_PLOTTER_PLOT_COORDS)

#ifndef FRAME_PLOTTER_DISTANCE_PLOT
#define FRAME_PLOTTER_DISTANCE_PLOT 1 ///< Plot lines with distance on frame to objects from the cv_active_random_filter module
#endif
PRINT_CONFIG_VAR(FRAME_PLOTTER_DISTANCE_PLOT)

#ifndef FRAME_PLOTTER_SHOW_STATS
#define FRAME_PLOTTER_SHOW_STATS 1 ///< Show statistics on the performance of the contour detection from the cv_active_random_filter module
#endif
PRINT_CONFIG_VAR(FRAME_PLOTTER_SHOW_STATS)

#ifndef FRAME_PLOTTER_SHOW_CAM_INFO
#define FRAME_PLOTTER_SHOW_CAM_INFO 1 ///< Show colour gains and exposure on frame from cv_ae_awb module
#endif
PRINT_CONFIG_VAR(FRAME_PLOTTER_SHOW_CAM_INFO)

#ifndef FRAME_PLOTTER_SHOW_HISTOGRAM
#define FRAME_PLOTTER_SHOW_HISTOGRAM 1 ///< Show histogram from cv_ae_awb module
#endif
PRINT_CONFIG_VAR(FRAME_PLOTTER_SHOW_HISTOGRAM)

#ifndef FRAME_PLOTTER_SHOW_FPS
#define FRAME_PLOTTER_SHOW_FPS 1 ///< Measure and show the (low-passed) FPS
#endif
PRINT_CONFIG_VAR(FRAME_PLOTTER_SHOW_FPS)

bool cv_frame_plotter_show_as_totv = FRAME_PLOTTER_SHOW_TOTV;
bool cv_frame_plotter_show_arf_ball_circles = FRAME_PLOTTER_BALL_CIRCLES;
bool cv_frame_plotter_show_arf_gate_corners = FRAME_PLOTTER_GATE_CORNERS;
bool cv_frame_plotter_show_arf_obj_coords = FRAME_PLOTTER_PLOT_COORDS;
bool cv_frame_plotter_show_arf_obj_distance = FRAME_PLOTTER_DISTANCE_PLOT;
bool cv_frame_plotter_show_arf_stats = FRAME_PLOTTER_SHOW_STATS;
bool cv_frame_plotter_show_ae_awb_info = FRAME_PLOTTER_SHOW_CAM_INFO;
bool cv_frame_plotter_show_fps = FRAME_PLOTTER_SHOW_FPS;
bool cv_frame_plotter_show_ae_awb_histogram = FRAME_PLOTTER_SHOW_HISTOGRAM;

static Rect             setISPvars( uint16_t width, uint16_t height);

static uint16_t 	      runCount            = 0;                    ///< Current frame number

extern uint16_t             ispWidth;                                   ///< Maximum width of ISP after applied scaling
extern uint16_t             ispHeight;                                  ///< Maximum height of ISP after applied scaling
extern uint16_t             initialWidth;                               ///< Initial width of ISP after applied scaling
extern uint16_t             initialHeight;                              ///< Initial height of ISP after applied scaling
extern double               ispScalar;                                  ///< Applied scalar by the ISP

#if FRAME_PLOTTER_SHOW_FPS
double      AVG_FPS                         = 20.0;                 ///< Estimated FPS to estimate lost neighbour decay
static struct timespec      time_now;                               ///< The current time
static struct timespec      time_prev;                              ///< The time of the previous frame
static struct timespec      time_init;                              ///< The time the processing began (after timeout)
static uint32_t curT;                                               ///< The time in us between time_init and time_now
#endif

void cv_frame_plotter_init(void){
#if FP_SHOW_FPS
  clock_gettime(CLOCK_MONOTONIC, &time_prev);
  time_init = time_prev;
#endif
}

void cv_frame_plotter_func(char* buff, uint16_t width, uint16_t height){
  Mat sourceFrame (height, width, CV_8UC2, buff);                 // Initialize current frame in openCV (UYVY) 2 channel
#if FRAME_PLOTTER_SHOW_FPS
  clock_gettime(CLOCK_MONOTONIC, &time_now);
  curT            = sys_time_elapsed_us(&time_init, &time_now);
  uint32_t dt_us  = sys_time_elapsed_us(&time_prev, &time_now);
  AVG_FPS   = 0.975 * AVG_FPS + 0.025 * 1000000.f / dt_us;
  time_prev       = time_now;
#endif
  Rect crop 	        = setISPvars( width, height); 	            // Calculate ISP related parameters
  sourceFrame = sourceFrame(crop); 				                        // Crop the frame
  uint16_t plotline_left = 20;
  uint16_t plotline_right = sourceFrame.rows-20;
  char text[200];
#if FRAME_PLOTTER_SHOW_FPS
  if(cv_frame_plotter_show_fps){
    sprintf(text,"%5.2f %5.d %8.2fs", AVG_FPS,(runCount), curT / 1000000.0);
    putText(sourceFrame, text, Point(10,plotline_right), FONT_HERSHEY_PLAIN, 1, Scalar(0,255,255), 1);
    plotline_right-= 20;
  }
#endif
  if(cv_frame_plotter_show_arf_stats){
    sprintf(text,"t:%4.1f%% o:%4.1f%%", pixCount/((float) ispHeight * ispWidth) * 100, pixSucCount/((float) pixCount) * 100);
    putText(sourceFrame, text, Point(10,plotline_right), FONT_HERSHEY_PLAIN, 1, Scalar(0,255,255), 1);
    plotline_right-= 20;
    sprintf(text,"d:%4.1f%% n:%4.1f%% s:%4.1f%%", pixDupCount/((float) pixCount) * 100, pixNofCount/((float) pixCount) * 100, pixSrcCount/((float) pixCount) * 100);
    putText(sourceFrame, text, Point(10,plotline_right), FONT_HERSHEY_PLAIN, 1, Scalar(0,255,255), 1);
    plotline_right-= 20;
  }
  if(cv_frame_plotter_show_ae_awb_info){
    sprintf(text,"Exp: %2.3f / %2.3f  (%3.1f / %d)", mt9f002.real_exposure, mt9f002.target_exposure, ae_current_level, ae_middle_index);
    putText(sourceFrame, text, Point(10 , plotline_left), FONT_HERSHEY_PLAIN, 1, Scalar(0,255,255), 1);
    plotline_left+= 20;
    sprintf(text,"R:%4.3f B:%4.3f G:%4.3f", mt9f002.gain_red, mt9f002.gain_blue, mt9f002.gain_green1);
    putText(sourceFrame, text, Point(10 , plotline_left), FONT_HERSHEY_PLAIN, 1, Scalar(0,255,255), 1);
    plotline_left+= 20;
    sprintf(text,"avgU:%5.2f avgV:%5.2f (%d)", awb_avgU, awb_avgV, awb_nb_pixels);
    putText(sourceFrame, text, Point(10 , plotline_left), FONT_HERSHEY_PLAIN, 1, Scalar(0,255,255), 1);
    plotline_left+= 20;
  }
  if(settings_as_extended && cv_frame_plotter_show_as_totv){
    sprintf(text,"GC: %5.3f", AS_GC);
    putText(sourceFrame, text, Point(10 , plotline_left), FONT_HERSHEY_PLAIN, 1, Scalar(0,255,255), 1);
    plotline_left+= 20;
  }
  if(cv_frame_plotter_show_ae_awb_histogram){
    if(histogram_plot[0] > 0){
      for(uint16_t i = MIN_HIST_Y; i < MAX_HIST_Y; i++){
        if(i < MIN_HIST_Y + ae_dark_bins){
          line(sourceFrame, Point(0, plotline_left + i - MIN_HIST_Y), Point(round((1 - ae_dark_ignore) * histogram_plot[i] / ((float) histogram_plot[0]) * 2000), plotline_left + i - MIN_HIST_Y), Scalar(0,255), 1, 4);
        }else if(i == MIN_HIST_Y + ae_dark_bins){
          line(sourceFrame, Point(0, plotline_left + i - MIN_HIST_Y), Point(75, plotline_left + i - MIN_HIST_Y), Scalar(127,127), 1, 4);
        }else if(i == MAX_HIST_Y - ae_bright_bins){
          line(sourceFrame, Point(0, plotline_left + i - MIN_HIST_Y), Point(75, plotline_left + i - MIN_HIST_Y), Scalar(127,127), 1, 4);
        }else if(i > MAX_HIST_Y - ae_bright_bins){
          line(sourceFrame, Point(0, plotline_left + i - MIN_HIST_Y), Point(round((1 - ae_bright_ignore) * histogram_plot[i] / ((float) histogram_plot[0]) * 2000), plotline_left + i - MIN_HIST_Y), Scalar(0,255), 1, 4);
        }else if(i == ae_middle_index){
          line(sourceFrame, Point(0, plotline_left + i - MIN_HIST_Y), Point(75, plotline_left + i - MIN_HIST_Y), Scalar(255,255), 1, 4);
        }else{
          line(sourceFrame, Point(0, plotline_left + i - MIN_HIST_Y), Point(round(histogram_plot[i] / ((float) histogram_plot[0]) * 2000), plotline_left + i - MIN_HIST_Y), Scalar(0,255), 1, 4);
        }
      }
      plotline_left += MAX_HIST_Y - MIN_HIST_Y + 20;
    }
  }
  if(cv_frame_plotter_show_arf_obj_distance){
      for(unsigned int r=0; r < neighbourMem_size; r++)         // Convert angles & Write/Print output
      {
        line(sourceFrame, Point(0,sourceFrame.rows / 2.0), Point(neighbourMem[r].x_p - cropCol, neighbourMem[r].y_p), Scalar(0,255), 1, 4);
        sprintf(text,"%4.2f", neighbourMem[r].r_b);
        putText(sourceFrame, text, Point((neighbourMem[r].x_p - cropCol + 0) / 2.0, (neighbourMem[r].y_p + sourceFrame.rows / 2.0) / 2.0), FONT_HERSHEY_PLAIN, 1, Scalar(0,255), 1);
      }
    }
    if(cv_frame_plotter_show_arf_obj_coords){
      for(unsigned int r=0; r < neighbourMem_size; r++)         // Convert angles & Write/Print output
      {
        uint8_t tColor = (uint8_t) round(255 * (ARF_MEMORY - (runCount - neighbourMem[r].lastSeen)) / ((float) ARF_MEMORY));
        sprintf(text,"x%5.2f", neighbourMem[r].x_w);
        putText(sourceFrame, text, Point(neighbourMem[r].x_p - cropCol + sqrt(neighbourMem[r].area_p / M_PI) + 10, neighbourMem[r].y_p - 15), FONT_HERSHEY_PLAIN, 1, Scalar(0,tColor), 1);
        sprintf(text,"y%5.2f", neighbourMem[r].y_w);
        putText(sourceFrame, text, Point(neighbourMem[r].x_p - cropCol + sqrt(neighbourMem[r].area_p / M_PI) + 10, neighbourMem[r].y_p), FONT_HERSHEY_PLAIN, 1, Scalar(0,tColor), 1);
        sprintf(text,"z%5.2f", neighbourMem[r].z_w);
        putText(sourceFrame, text, Point(neighbourMem[r].x_p - cropCol + sqrt(neighbourMem[r].area_p / M_PI) + 10, neighbourMem[r].y_p + 15), FONT_HERSHEY_PLAIN, 1, Scalar(0,tColor), 1);
      }
    }
#if FRAME_PLOTTER_BALL_CIRCLES
  if(cv_frame_plotter_show_arf_ball_circles){
    for(unsigned int r=0; r < neighbourMem_size; r++)         // Convert angles & Write/Print output
    {
      circle(sourceFrame,cvPoint(neighbourMem[r].x_p - cropCol, neighbourMem[r].y_p), sqrt(neighbourMem[r].area_p / M_PI), cvScalar(100,255), 1, 4);
    }
  }
#endif
#if FRAME_PLOTTER_GATE_CORNERS
  if(cv_frame_plotter_show_arf_gate_corners){
    for(unsigned int r=0; r < neighbourMem_size; r++)         // Convert angles & Write/Print output
    {
      Point p1, p2;
      p1.x = neighbourMem[r].corners[3].x;
      p1.y = neighbourMem[r].corners[3].y;
      for(uint8_t i = 0; i < 4; i++){
        p2.x = neighbourMem[r].corners[i].x;
        p2.y = neighbourMem[r].corners[i].y;
        circle(sourceFrame, p2, 5, cvScalar(100,255), 1);
        line(sourceFrame, p1, p2, Scalar(0,255), 1);
        p1 = p2;
      }
    }
  }
#endif
  if(cv_frame_plotter_show_as_totv){
    circle(sourceFrame, Point(sourceFrame.cols / 2.0, sourceFrame.rows / 2.0), 3, cvScalar(100,255), 1, 4);
    arrowedLine(sourceFrame, Point(sourceFrame.cols / 2.0, sourceFrame.rows / 2.0), Point(sourceFrame.cols / 2.0 + 240.0 / settings_as_vmax * lastTotV[0], sourceFrame.rows / 2.0  + 240.0 / settings_as_vmax * lastTotV[1]), Scalar(100,255), 1, 4);
  }
  sourceFrame.release();                                          // Release Mat
  runCount++; // Increase counter
  return;
}

Rect setISPvars( uint16_t width, uint16_t height){
  // This function computes the cropping according to the desires FOV Y and the current euler angles
  ARF_MIN_CIRCLE_SIZE   = pow(sqrt((double) default_calArea  * pow(ispScalar,2.0)) / ARF_CAM_RANGE, 2.0);
  ARF_MIN_LAYERS        = (uint16_t) round(ARF_MIN_CIRCLE_PERC * 2 * M_PI * sqrt((double) default_calArea  * pow(ispScalar,2.0)/ M_PI) / ARF_CAM_RANGE);
  ARF_LARGE_LAYERS      = (uint16_t) round(ARF_MIN_CIRCLE_PERC * 2 * M_PI * sqrt((double) default_calArea  * pow(ispScalar,2.0)/ M_PI) / 1.0);
  ARF_MIN_POINTS        = (uint16_t) round(0.25 * ARF_MIN_LAYERS);

  Rect crop;
  if(fillHeight > 0){
    crop                   = cvRect(0, 0, width, height);
  }else{
    if(mt9f002.output_width > fillHeight){
      crop                   = cvRect((int) fillHeight,(int) 0,(int) (mt9f002.output_width - fillHeight), (int) height);
    }else{
      crop                   = cvRect((int) fillHeight,(int) 0,(int) width,(int) height);
    }
  }
  if(crop.width > width || crop.width <= 0) crop.width = width;
  if(crop.height > height || crop.height <= 0) crop.height = height;
  if(crop.x > width || crop.x <= 0) crop.x = 0;
  if(crop.y > height || crop.y <= 0) crop.y = 0;
  return crop;
}
