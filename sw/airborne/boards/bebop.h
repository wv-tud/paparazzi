/*
 * Copyright (C) 2014 Freek van Tienen <freek.v.tienen@gmail.com>
 *
 * This file is part of paparazzi.
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
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */

#ifndef CONFIG_BEBOP
#define CONFIG_BEBOP

#define BOARD_BEBOP

#include "std.h"
#include "peripherals/video_device.h"
#include "boards/bebop/mt9f002.h"

#ifndef MT9F002_THETA_OFFSET
#define MT9F002_THETA_OFFSET -30 * M_PI / 180
#endif

#ifndef MT9F002_X_OFFSET
#define MT9F002_X_OFFSET 0.1
#endif

#ifndef MT9F002_Y_OFFSET
#define MT9F002_Y_OFFSET 0.0
#endif

#ifndef MT9F002_Z_OFFSET
#define MT9F002_Z_OFFSET 0.0
#endif

#ifndef MT9F002_OUTPUT_WIDTH
#define MT9F002_OUTPUT_WIDTH 960 // 960
#endif

#ifndef MT9F002_OUTPUT_HEIGHT
#define MT9F002_OUTPUT_HEIGHT 840 // 834
#endif

#ifndef MT9F002_SENSOR_HEIGHT
#define MT9F002_SENSOR_HEIGHT (CFG_MT9F002_Y_ADDR_MAX - CFG_MT9F002_Y_ADDR_MIN)
#endif

#ifndef MT9F002_SENSOR_WIDTH
#define MT9F002_SENSOR_WIDTH  (CFG_MT9F002_X_ADDR_MAX - CFG_MT9F002_X_ADDR_MIN)
#endif

#ifndef MT9F002_INITIAL_OFFSET_X
#define MT9F002_INITIAL_OFFSET_X (CFG_MT9F002_X_ADDR_MIN) // pixels in the raw sensor(please make steps of 2 for ISP)!!
#endif

#ifndef MT9F002_INITIAL_OFFSET_Y
#define MT9F002_INITIAL_OFFSET_Y (CFG_MT9F002_Y_ADDR_MIN) // pixels in the raw sensor(please make steps of 2 for ISP)!!
#endif

/** Our output is only OUTPUT_SCALER of the pixels we take of the sensor
 * It is programmable in 1/16 steps determined by ScaleFactor = 16/scale_m.
 * Legal values for scale_m are 16 through 128, giving you the ability to scale from
 * 1:1 to 1:8 (with m=128).
 *  Example:
 *  output_width = 512
 *  output_height = 830
 *  output_scaler = 0.25
 *  We now get an image of 512 by 830 which contains a "compressed version"
 *  of what would normally be an image of 2048 by 3320.
 *  Be warned: set your offset x appropriately.
 *  Example of what could go wrong:
 *  output_width = 512
 *  output_height = 830
 *  output_scaler = 0.25
 *  offset_x = 1500
 *  We now ask for pixels outside the 4608H x 2592V sensor or the 3320H x 2048W of the ISP.
 */
#ifndef MT9F002_OUTPUT_SCALER
#define MT9F002_OUTPUT_SCALER 1.0
#endif

/** Exposure of the front camera of the bebop. Experimental values:
 * Outside: 15
 * Inside well lit: 30
 * Inside poorly lit: 60
 */
#ifndef MT9F002_TARGET_EXPOSURE
#define MT9F002_TARGET_EXPOSURE 30
#endif

#ifndef MT9F002_TARGET_FPS
#define MT9F002_TARGET_FPS 30
#endif

/* Set the colour balance gains */
#ifndef MT9F002_GAIN_GREEN1
#define MT9F002_GAIN_GREEN1 2.0
#endif

#ifndef MT9F002_GAIN_GREEN2
#define MT9F002_GAIN_GREEN2 2.0
#endif

#ifndef MT9F002_GAIN_RED
#define MT9F002_GAIN_RED 2.0
#endif

#ifndef MT9F002_GAIN_BLUE
#define MT9F002_GAIN_BLUE 2.0
#endif

/* Set subsampling */
#ifndef MT9F002_X_ODD_INC_VAL
#define MT9F002_X_ODD_INC_VAL 7
#endif

#ifndef MT9F002_Y_ODD_INC_VAL
#define MT9F002_Y_ODD_INC_VAL 7
#endif

/** uart connected to GPS internally */
#define UART1_DEV /dev/ttyPA1

/** Parrot Freeflight app uses NMEA */
#if !defined GPS_UBX_ENABLE_NMEA || GPS_UBX_ENABLE_NMEA
#define GPS_UBX_ENABLE_NMEA_DATA_MASK 0xff
#else
#define GPS_UBX_ENABLE_NMEA_DATA_MASK 0x00
#endif

/** FTDI cable for stereoboard or external GPS */
#define UART2_DEV /dev/ttyUSB0

/* Default actuators driver */
#define DEFAULT_ACTUATORS "boards/bebop/actuators.h"
#define ActuatorDefaultSet(_x,_y) ActuatorsBebopSet(_x,_y)
#define ActuatorsDefaultInit() ActuatorsBebopInit()
#define ActuatorsDefaultCommit() ActuatorsBebopCommit()

/* Cameras */
extern struct video_config_t bottom_camera;
extern struct video_config_t front_camera;

/* ISP */
extern struct mt9f002_t mt9f002;

/* by default activate onboard baro */
#ifndef USE_BARO_BOARD
#define USE_BARO_BOARD 1
#endif

/* The ADC from the sonar */
#if USE_ADC0
#define ADC0_ID             0
#define ADC0_CHANNELS       2
#define ADC0_CHANNELS_CNT   1
#define ADC0_BUF_LENGTH     8192
#endif

/* The SPI from the sonar */
#if USE_SPI0
#define SPI0_MODE           0
#define SPI0_BITS_PER_WORD  8
#define SPI0_MAX_SPEED_HZ   320000
#endif

#endif /* CONFIG_BEBOP */
