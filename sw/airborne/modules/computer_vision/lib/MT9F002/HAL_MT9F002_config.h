/*
 * MT9F002_MIPI.h
 *
 *  Created on: 20 févr. 2013
 *      Author: peline
 */

#ifndef _HAL_MT9F002_CONFIG_H_
#define _HAL_MT9F002_CONFIG_H_

/*
 * MT9F002 i2c settings
 */
#define HAL_MT9F002_I2C_ADDR          0x10
#define HAL_MT9F002_I2C_REGADDR_SIZE     2
#define HAL_MT9F002_I2C_REG_SIZE         2

/*
 * PLL constraint (see figure - MTF9002_DS_D.pdf p38), in MHZ
 */
// frequencies range
#define EXT_CLK_MIN 2.0     // camera input clock min value (symbol ext_clk_freq_mhz)
#define EXT_CLK_MAX 64.0    // camera input clock max value (symbol ext_clk_freq_mhz)

#define PLL_INPUT_CLK_MIN 2.0 // pll input clock min value (symbol pll_ip_clk_freq)
#define PLL_INPUT_CLK_MAX 24.0  // pll input clock max value (symbol pll_ip_clk_freq)

#define VCO_CLK_MIN 384.0   // vco clock min value (symbol frequency)
#define VCO_CLK_MAX 768.0   // vco clock max value (symbol frequency)

#define PIXEL_CLOCK_MAX 96.0
#define SERIAL_CLOCK_MAX 700.0

#define PIXEL_CLK_MAX 120.0

// pll registers range
#define PRE_PLL_CLK_DIV_MIN 1
#define PRE_PLL_CLK_DIV_MAX 64

#define PLL_MUL_EVEN_MIN 32
//#define PLL_MUL_EVEN_MAX 384 // TODO according to datasheet max value is 384 but in fact it seems to be 254
#define PLL_MUL_EVEN_MAX 254
#define PLL_MUL_ODD_MIN 17
#define PLL_MUL_ODD_MAX 191
#define PLL_MUL_MIN PLL_MUL_ODD_MIN
#define PLL_MUL_MAX PLL_MUL_EVEN_MAX

// other constraints
#define SERIAL_INTERFACE_ROW_SPEED_10_8_VALUE 1 // only one value is authorized for serial interface

#define LINE_LENGTH_MAX  0xFFFF
#define FRAME_LENGTH_MAX 0xFFFF
/*
 * Scaler
 */
#define SCALER_N 16
#define SCALER_M_MIN 16
#define SCALER_M_MAX 128

/*
 * Miscellaneous
 */
#define MIN_GAIN 1.5

#define SET_POSITION_WRITE_DURATION_US  3000     /* a rough estimation of the delay (in µs) to send position to camera */
#define SET_POSITION_DEADLINE_WINDOW_US 4000     /* minimum time (in µs) between a position write and new frame start to be in secure window 2*/

#endif /* MT9F002_MIPI_H_ */
