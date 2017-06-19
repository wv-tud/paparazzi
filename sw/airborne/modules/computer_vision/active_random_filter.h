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
#include "std.h"

#define ARF_MAX_OBJECTS   30  // Maximum nr of objects

/** For ball detection **/

typedef struct _trackResults {
    uint16_t     x_p;
    uint16_t     y_p;
    uint32_t  area_p;
    double 	x_c;
    double 	y_c;
    double 	r_c;
    double  x_b;
    double  y_b;
    double  z_b;
    double  r_b;
    double  x_w;
    double  y_w;
    double  z_w;
} trackResults;

typedef struct _memBlock {
	uint32_t lastSeen;
	uint16_t id;
	uint16_t x_p;
	uint16_t y_p;
	uint32_t area_p;
	double r_c;
	double r_b;
	double x_w;
	double y_w;
	double z_w;
} memoryBlock;

/** For gate detection **/
typedef struct _gatePoint {
    uint16_t x;
    uint16_t y;
} gatePoint;

typedef struct _gateResults {
    uint16_t     x_p;
    uint16_t     y_p;
    uint32_t     area_p;
    gatePoint    corners[4];
} gateResults;

typedef struct _memGateBlock {
    uint32_t    lastSeen;
    uint16_t    id;
    uint16_t    x_p;
    uint16_t    y_p;
    uint32_t    area_p;
    gatePoint   corners[4];
} memoryGateBlock;

extern uint8_t      ARF_FLOOD_STYLE;
extern uint8_t      ARF_SAMPLE_STYLE;
extern uint16_t     ARF_RND_PIX_SAMPLE;

extern uint8_t 		ARF_Y_MIN;
extern uint8_t 		ARF_Y_MAX;
extern uint8_t 		ARF_U_MIN;
extern uint8_t 		ARF_U_MAX;
extern uint8_t 		ARF_V_MIN;
extern uint8_t 		ARF_V_MAX;
extern uint8_t      ARF_CDIST_YTHRES;
extern uint8_t      ARF_CDIST_UTHRES;
extern uint8_t      ARF_CDIST_VTHRES;
extern int8_t      ARF_GREY_THRES;

extern double       ARF_CAM_RANGE;
extern double       ARF_MIN_CIRCLE_PERC;
extern double       ARF_MAX_CIRCLE_DEF;
extern uint16_t     default_calArea;

extern uint16_t           pixCount;                    ///< Total pixels processed (resets to 0 before each frame)
extern uint16_t           pixSucCount;                    ///< Total pixels passing pixTest whilst following contours (resets to 0 before each frame)
extern uint16_t             pixDupCount;                    ///< Total pixels processed whilst being a duplicate contour (resets to 0 before each frame)
extern uint16_t             pixSrcCount;                    ///< Total pixels processed during search for contours (resets to 0 before each frame)
extern uint16_t             pixNofCount;

// Filter sample styles
#define ARF_STYLE_FULL   0
#define ARF_STYLE_GRID   1
#define ARF_STYLE_RANDOM 2
// Filter flood styles
#define ARF_FLOOD_OMNI   0
#define ARF_FLOOD_CW     1
// Filter flood directions
#define ARF_NONE      -1
#define ARF_SEARCH     0
#define ARF_UP         1
#define ARF_UP_RIGHT   2
#define ARF_RIGHT      3
#define ARF_RIGHT_DOWN 4
#define ARF_DOWN       5
#define ARF_DOWN_LEFT  6
#define ARF_LEFT       7
#define ARF_LEFT_UP    8
// Filter return status
#define ARF_FINISHED   1
#define ARF_NO_FOUND   0
#define ARF_DUPLICATE -1
#define ARF_ERROR     -2


/** Set up Remaining parameters **/
extern double      ARF_CROP_X;                  ///< Crop margin for blobs when using omni detection
extern uint8_t     ARF_MEMORY;                   ///< Frames to keep neighbours in memory
extern double      ARF_VMAX;                  ///< Maximum estimated velocity of a neighbour (account for some noise)
extern double      ARF_MIN_CIRCLE_SIZE;                                        ///< Minimum contour area
extern uint16_t    ARF_MIN_LAYERS;                                             ///< Miminum recursive depth of CW flood
extern uint16_t    ARF_LARGE_LAYERS;                                           ///< Miminum recursive depth of CW flood before classified as a large contour
extern uint16_t    ARF_MIN_POINTS;                                             ///< Mimimum contour length

#ifdef __cplusplus
extern "C" {
#endif
#include <state.h>                              // C header used for state functions and data
void active_random_filter_init(void);
void active_random_filter(char* buff, uint16_t width, uint16_t height);

extern memoryBlock          neighbourMem[ARF_MAX_OBJECTS];      ///< The array of neighbours from active_random_filter
extern uint8_t              neighbourMem_size;                  ///< The size of the neighbour array
#ifdef __linux__
//extern pthread_mutex_t      neighbourMem_mutex;
#endif

#ifdef __cplusplus
}
#endif
