/*
 * HAL_MT9F002_MIPI_camera.c
 *
 *  Created on: 20 févr. 2013
 *      Author: peline
 */

#include <stdio.h>
#include <HAL_i2ctool.h>
#include "HAL_MT9F002_config.h"
#include "HAL_MT9F002_register_definition.h"
#include <HAL_MT9F002.h>
#include <unistd.h>
#include <math.h>
#include <string.h>

/*
 * Macro
 */

#define HAL_MT9F002_PRINT(string,...) fprintf(stderr, string, ##__VA_ARGS__)

#define CHECK_NULL_CTX(ctx)                       \
  if (ctx == NULL)                                \
  {                                               \
    HAL_MT9F002_PRINT("error, context is null\n");    \
    return -1;                                    \
  }

#define CHECK_STATE(ctx, assert_state)           \
  if (ctx->state != assert_state)                \
  {                                              \
    switch(assert_state)                         \
    {                                            \
       case HAL_MT9F002_INIT_STATE:                  \
         HAL_MT9F002_PRINT("context state error, either context has not been initialized or camera is already running [current state %d expected state %d]\n",ctx->state,assert_state); \
         break;                                  \
       case HAL_MT9F002_OPEN_STATE:                  \
         HAL_MT9F002_PRINT("context state error, either context has not been initialized or camera is not already running [current state %d expected state %d]\n",ctx->state,assert_state); \
         break;                                  \
       default:                                  \
         HAL_MT9F002_PRINT("context state error, (default message) [current state %d expected state %d]\n",ctx->state,assert_state); \
         break;                                  \
    }                                            \
    return -1;                                   \
  }

#define MAX(a,b) ((a) > (b) ? (a) : (b))
#define MIN(a,b) ((a) > (b) ? (b) : (a))

/*
 *  Default config
 */
static HAL_MT9F002_pixelDepth_t   init_MT9F002_pixelDepth  = HAL_MT9F002_8bits;
static HAL_MT9F002_Serial_lanes_t   init_MT9F002_serial_lanes  = HAL_MT9F002_2lane;
static HAL_MT9F002_Colorbar_t     init_MT9F002_Colorabar   = HAL_MT9F002_ColorBar_off;
static HAL_MT9F002_StreamOn_t     init_MT9F002_StreamOn    = HAL_MT9F002_Stream_off;
static HAL_MT9F002_colorGain_t    init_MT9F002_colorGain   = {2.0, 2.0, 2.0, 2.0};
static HAL_MT9F002_res_t          init_MT9F002_res         = {640,480};
static HAL_MT9F002_pos_t          init_MT9F002_pos         = {0,0};
static float                      init_MT9F002_fps         = 30;
static float                      init_MT9F002_exposure_ms = 30;
static float                      init_MT9F002_scaler      = 1.0;

/*
 * Time function
 * return a-b in µs
 *
 * Note: a & b have to be close enough so that the res doesn't overflow
 */
static int diffTime(struct timespec *a, struct timespec *b)
{
  int res = a->tv_sec - b->tv_sec;
  res *= 1000000;
  res = res + ((a->tv_nsec - b->tv_nsec)/1000);
  return res;
}


/*
 * I2C helper functions
 */

/*
 * Function to read easily from a register
 */
static int readReg(HAL_i2ctoolContext_t *i2cCtx, regDef_t reg, uint32_t *value)
{
  uint64_t val;
  int res = HAL_i2ctool_read_regSize(i2cCtx,reg.addr,&val,reg.regSize);
  *value = val;
  return res;
}

/*
 * Function to write easily to a register
 */
static int writeReg(HAL_i2ctoolContext_t *i2cCtx, regDef_t reg, uint32_t value)
{
  return HAL_i2ctool_write_regSize(i2cCtx, reg.addr, value, reg.regSize);
}

/*
 * Camera helper functions
 */
static int softReset(HAL_MT9F002_context_t* ctx)
{
  int res=0;
  res = writeReg(&ctx->i2c,SOFTWARE_RESET,1);
  usleep(1000000);
  return res;
}

static int streamOn(HAL_MT9F002_context_t* ctx, HAL_MT9F002_StreamOn_t on)
{
  int res=0;
  switch (on)
  {
  case HAL_MT9F002_Stream_off_with_PxlClock :
    // output 0 lines
    res |= writeReg(&ctx->i2c ,Y_OUTPUT_SIZE,0);
    // trun on stream, to ouput pixel clock
    res |= writeReg(&ctx->i2c,MODE_SELECT,1);
    break;
  case HAL_MT9F002_Stream_off :
    res |= writeReg(&ctx->i2c,MODE_SELECT,0);
    break;
  case HAL_MT9F002_Stream_on :
    // restore Y_OUTPUT_SIZE that could have been reset by a previous HAL_MT9F002_Stream_off_with_PxlClock
    res |= writeReg(&ctx->i2c ,Y_OUTPUT_SIZE,ctx->res.height);
    // turn on stream
    res |= writeReg(&ctx->i2c,MODE_SELECT,1);
    break;
  default :
    HAL_MT9F002_PRINT("error, unknown stream option %d\n",on);
    res = -1;
  }
  return res;
}

static int parallel_stage1_conf(HAL_MT9F002_context_t *ctx)
{
  int res=0;
  res |= writeReg(&ctx->i2c , RESET_REGISTER            , 0x0010 );
  res |= writeReg(&ctx->i2c , GLOBAL_GAIN               , 0x1430 );
  res |= writeReg(&ctx->i2c , RESET_REGISTER            , 0x0010 );
  res |= writeReg(&ctx->i2c , RESET_REGISTER            , 0x0010 );
  res |= writeReg(&ctx->i2c , RESET_REGISTER            , 0x0010 );
  res |= writeReg(&ctx->i2c , DAC_LD_14_15              , 0xE525 );
  res |= writeReg(&ctx->i2c , CTX_CONTROL_REG           , 0x0000 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0xF873 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0x08AA );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0x3219 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0x3219 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0x3219 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0x3200 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0x3200 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0x3200 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0x3200 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0x3200 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0x1769 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0xA6F3 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0xA6F3 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0xA6F3 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0xA6F3 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0xA6F3 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0xA6F3 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0xA6F3 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0xAFF3 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0xFA64 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0xFA64 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0xFA64 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0xF164 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0xFA64 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0xFA64 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0xFA64 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0xF164 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0x276E );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0x18CF );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0x18CF );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0x18CF );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0x28CF );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0x18CF );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0x18CF );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0x18CF );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0x18CF );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0x2363 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0x2363 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0x2352 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0x2363 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0x2363 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0x2363 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0x2352 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0x2352 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0xA394 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0xA394 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0x8F8F );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0xA3D4 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0xA394 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0xA394 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0x8F8F );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0x8FCF );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0xDC23 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0xDC63 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0xDC63 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0xDC23 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0xDC23 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0xDC63 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0xDC63 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0xDC23 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0x0F73 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0x85C0 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0x85C0 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0x85C0 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0x85C0 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0x85C0 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0x85C0 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0x85C0 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0x85C4 );
  res |= writeReg(&ctx->i2c , CTX_WR_DATA_REG           , 0x0000 );
  res |= writeReg(&ctx->i2c , ANALOG_CONTROL4           , 0x8000 );
  res |= writeReg(&ctx->i2c , DAC_LD_14_15              , 0xE525 );
  res |= writeReg(&ctx->i2c , DATA_PEDESTAL_            , 0x00A8 );
  res |= writeReg(&ctx->i2c , RESET_REGISTER            , 0x0090 );
  res |= writeReg(&ctx->i2c , SERIAL_FORMAT             , 0x0301 );
  res |= writeReg(&ctx->i2c , RESET_REGISTER            , 0x1090 );
  res |= writeReg(&ctx->i2c , SMIA_TEST                 , 0x0845 );
  res |= writeReg(&ctx->i2c , RESET_REGISTER            , 0x1080 );
  res |= writeReg(&ctx->i2c , DATAPATH_SELECT           , 0xD880 );
  res |= writeReg(&ctx->i2c , RESET_REGISTER            , 0x9080 );
  res |= writeReg(&ctx->i2c , DATAPATH_SELECT           , 0xD880 );
  res |= writeReg(&ctx->i2c , RESET_REGISTER            , 0x10C8 );
  res |= writeReg(&ctx->i2c , DATAPATH_SELECT           , 0xD880 );

  return res;
}

static int parallel_stage2_conf(HAL_MT9F002_context_t *ctx)
{
  int res=0;

  res |= writeReg(&ctx->i2c , ANALOG_CONTROL4, 0x8000);
  res |= writeReg(&ctx->i2c , READ_MODE, 0x0041);

  res |= writeReg(&ctx->i2c , READ_MODE                 , 0x04C3);
  res |= writeReg(&ctx->i2c , READ_MODE                 , 0x04C3);
  res |= writeReg(&ctx->i2c , ANALOG_CONTROL5           , 0x0000);
  res |= writeReg(&ctx->i2c , ANALOG_CONTROL5           , 0x0000);
  res |= writeReg(&ctx->i2c , ANALOG_CONTROL5           , 0x0000);
  res |= writeReg(&ctx->i2c , ANALOG_CONTROL5           , 0x0000);
  res |= writeReg(&ctx->i2c , DAC_LD_28_29              , 0x0047);
  res |= writeReg(&ctx->i2c , COLUMN_CORRECTION         , 0xB080);
  res |= writeReg(&ctx->i2c , COLUMN_CORRECTION         , 0xB100);
  res |= writeReg(&ctx->i2c , DARK_CONTROL3             , 0x0020);
  res |= writeReg(&ctx->i2c , DAC_LD_24_25              , 0x6349);
  res |= writeReg(&ctx->i2c , ANALOG_CONTROL7           , 0x800A);
  res |= writeReg(&ctx->i2c , RESET_REGISTER            , 0x90C8);
  res |= writeReg(&ctx->i2c , CTX_CONTROL_REG           , 0x8005);
  res |= writeReg(&ctx->i2c , ANALOG_CONTROL7           , 0x800A);
  res |= writeReg(&ctx->i2c , DAC_LD_28_29              , 0x0047);
  res |= writeReg(&ctx->i2c , DAC_LD_30_31              , 0x15F0);
  res |= writeReg(&ctx->i2c , DAC_LD_30_31              , 0x15F0);
  res |= writeReg(&ctx->i2c , DAC_LD_30_31              , 0x15F0);
  res |= writeReg(&ctx->i2c , DAC_LD_28_29              , 0x0047);
  res |= writeReg(&ctx->i2c , DAC_LD_28_29              , 0x0047);
  res |= writeReg(&ctx->i2c , RESET_REGISTER            , 0x10C8);
  //res |= writeReg(&ctx->i2c , RESET_REGISTER            , 0x14C8); // reset bad frame
  res |= writeReg(&ctx->i2c , COARSE_INTEGRATION_TIME   , 0x08C3);
  res |= writeReg(&ctx->i2c , DIGITAL_TEST              , 0x0000);
  //res |= writeReg(&ctx->i2c , DATAPATH_SELECT           , 0xd881); // permanent line valid
  res |= writeReg(&ctx->i2c , DATAPATH_SELECT           , 0xd880);
  res |= writeReg(&ctx->i2c , READ_MODE                 , 0x0041);
  res |= writeReg(&ctx->i2c , X_ODD_INC                 , 0x0001);
  res |= writeReg(&ctx->i2c , Y_ODD_INC                 , 0x0001);
  res |= writeReg(&ctx->i2c , MASK_CORRUPTED_FRAME      , 0x0001); // 0 output corrupted frame, 1 mask them

  return res;
}

static int mipi_stage1_conf(HAL_MT9F002_context_t *ctx)
{
  int res=0;
  res |= writeReg(&ctx->i2c , RESET_REGISTER   , 0x0118);
  res |= writeReg(&ctx->i2c , MODE_SELECT      , 0x00  );
  uint32_t serialFormat;
  if (ctx->interface == HAL_MT9F002_HiSPi)
  {
    serialFormat = (3<<8)|ctx->numberOfLanes; // parrot flex
  }
  else
  {
    serialFormat = (2<<8)|ctx->numberOfLanes;// mipi demo board
  }

  res |= writeReg(&ctx->i2c , SERIAL_FORMAT    , serialFormat);
  uint32_t dataFormat = (ctx->pixelDepth<<8)|ctx->pixelDepth;
  res |= writeReg(&ctx->i2c , CCP_DATA_FORMAT  , dataFormat);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D00, 0x0435);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D02, 0x435D);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D04, 0x6698);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D06, 0xFFFF);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D08, 0x7783);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D0A, 0x101B);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D0C, 0x732C);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D0E, 0x4230);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D10, 0x5881);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D12, 0x5C3A);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D14, 0x0140);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D16, 0x2300);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D18, 0x815F);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D1A, 0x6789);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D1C, 0x5920);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D1E, 0x0C20);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D20, 0x21C0);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D22, 0x4684);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D24, 0x4892);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D26, 0x1A00);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D28, 0xBA4C);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D2A, 0x8D48);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D2C, 0x4641);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D2E, 0x408C);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D30, 0x4784);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D32, 0x4A87);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D34, 0x561A);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D36, 0x00A5);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D38, 0x1A00);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D3A, 0x5693);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D3C, 0x4D8D);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D3E, 0x4A47);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D40, 0x4041);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D42, 0x8200);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D44, 0x24B7);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D46, 0x0024);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D48, 0x8D4F);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D4A, 0x831A);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D4C, 0x00B4);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D4E, 0x4684);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D50, 0x49CE);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D52, 0x4946);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D54, 0x4140);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D56, 0x9247);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D58, 0x844B);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D5A, 0xCE4B);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D5C, 0x4741);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D5E, 0x502F);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D60, 0xBD3A);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D62, 0x5181);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D64, 0x5E73);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D66, 0x7C0A);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D68, 0x7770);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D6A, 0x8085);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D6C, 0x6A82);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D6E, 0x6742);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D70, 0x8244);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D72, 0x831A);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D74, 0x0099);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D76, 0x44DF);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D78, 0x1A00);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D7A, 0x8542);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D7C, 0x8567);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D7E, 0x826A);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D80, 0x857C);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D82, 0x6B80);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D84, 0x7000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D86, 0xB831);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D88, 0x40BE);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D8A, 0x6700);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D8C, 0x0CBD);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D8E, 0x4482);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D90, 0x7898);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D92, 0x7480);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D94, 0x5680);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D96, 0x9755);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D98, 0x8057);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D9A, 0x8056);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D9C, 0x9256);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3D9E, 0x8057);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3DA0, 0x8055);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3DA2, 0x817C);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3DA4, 0x969B);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3DA6, 0x56A6);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3DA8, 0x44BE);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3DAA, 0x000C);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3DAC, 0x867A);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3DAE, 0x9474);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3DB0, 0x8A79);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3DB2, 0x9367);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3DB4, 0xBF6A);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3DB6, 0x816C);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3DB8, 0x8570);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3DBA, 0x836C);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3DBC, 0x826A);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3DBE, 0x8245);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3DC0, 0xFFFF);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3DC2, 0xFFD6);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3DC4, 0x4582);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3DC6, 0x6A82);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3DC8, 0x6C83);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3DCA, 0x7000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3DCC, 0x8024);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3DCE, 0xB181);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3DD0, 0x6859);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3DD2, 0x732B);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3DD4, 0x4030);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3DD6, 0x4982);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3DD8, 0x101B);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3DDA, 0x4083);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3DDC, 0x6785);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3DDE, 0x3A00);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3DE0, 0x8820);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3DE2, 0x0C59);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3DE4, 0x8546);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3DE6, 0x8348);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3DE8, 0xD04C);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3DEA, 0x8B48);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3DEC, 0x4641);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3DEE, 0x4083);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3DF0, 0x1A00);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3DF2, 0x8347);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3DF4, 0x824A);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3DF6, 0x9A56);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3DF8, 0x1A00);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3DFA, 0x951A);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3DFC, 0x0056);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3DFE, 0x914D);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E00, 0x8B4A);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E02, 0x4700);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E04, 0x0300);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E06, 0x2492);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E08, 0x0024);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E0A, 0x8A1A);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E0C, 0x004F);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E0E, 0xB446);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E10, 0x8349);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E12, 0xB249);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E14, 0x4641);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E16, 0x408B);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E18, 0x4783);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E1A, 0x4BDB);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E1C, 0x4B47);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E1E, 0x4180);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E20, 0x502B);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E22, 0x4C3A);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E24, 0x4180);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E26, 0x737C);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E28, 0xD124);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E2A, 0x9068);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E2C, 0x8A20);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E2E, 0x2170);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E30, 0x8081);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E32, 0x6A67);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E34, 0x4257);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E36, 0x5544);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E38, 0x8644);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E3A, 0x9755);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E3C, 0x5742);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E3E, 0x676A);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E40, 0x807D);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E42, 0x3180);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E44, 0x7000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E46, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E48, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E4A, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E4C, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E4E, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E50, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E52, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E54, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E56, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E58, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E5A, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E5C, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E5E, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E60, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E62, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E64, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E66, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E68, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E6A, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E6C, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E6E, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E70, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E72, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E74, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E76, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E78, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E7A, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E7C, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E7E, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E80, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E82, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E84, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E86, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E88, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E8A, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E8C, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E8E, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E90, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E92, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E94, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E96, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E98, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E9A, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E9C, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3E9E, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3EA0, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3EA2, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3EA4, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3EA6, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3EA8, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3EAA, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3EAC, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3EAE, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3EB0, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3EB2, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3EB4, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3EB6, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3EB8, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3EBA, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3EBC, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3EBE, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3EC0, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3EC2, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3EC4, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3EC6, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3EC8, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3ECA, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3176, 0x4000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_317C, 0xA00A);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3EE6, 0x0000);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3ED8, 0xE0E0);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3EE8, 0x0001);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3064, 0x0005);

  return res;
}

static int mipi_stage2_conf(HAL_MT9F002_context_t *ctx)
{
  int res=0;
#if 0
  { 0x3064, 0x0045   }, // RESERVED_MFR_3064
  { 0x0100, 0x01   }, // MODE_SELECT
  { 0x0100, 0x00   }, // MODE_SELECT
#endif
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3064, 0x0045);
  // TODO: why turning on/off stream ? try without
  //res |= writeReg(&ctx->i2c , MODE_SELECT      , 0x01  );
  //res |= writeReg(&ctx->i2c , MODE_SELECT      , 0x00  );
  return res;
}

static int mipi_stage3_conf(HAL_MT9F002_context_t *ctx)
{
  int res=0;
  res |= writeReg(&ctx->i2c , EXTRA_DELAY      , 0x0000);
  res |= writeReg(&ctx->i2c , RESET_REGISTER   , 0x0118);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3EDC, 0x68CF);
  res |= writeReg(&ctx->i2c , RESERVED_MFR_3EE2, 0xE363);

  return res;
}

static int pllConf(HAL_MT9F002_context_t* ctx)
{
  // fill constant value
  ctx->pll.shift_vt_pix_clk_div=1; // no documentation, set to 1 in the excel sheet
  ctx->pll.rowSpeed_2_0 = 1;       // don't understand what it is, fix it to 1

  /*
   * Fix [VCO -> pixel clock] pll
   */

#define SIZEOF_TAB(a) (sizeof(a)/sizeof(a[0]))

  uint32_t vt_sys_clk_div_tab[] = {1,2,4,6,8};
  uint32_t vt_pix_clk_div_tab[] = {2,3,4,5,6,7,8};
  uint32_t op_sys_clk_div_tab[] = {1,2,4,6,8};
  uint32_t op_pix_clk_div_tab[] = {8,10,12};
  uint32_t row_speed_10_8_tab[] = {1,2,4};

  if (ctx->interface == HAL_MT9F002_MIPI ||
      ctx->interface == HAL_MT9F002_HiSPi )
  {
    // MIPI/HiSPi has some forbiden values
    row_speed_10_8_tab[0] = row_speed_10_8_tab[1] = row_speed_10_8_tab[2] = 1;
    op_pix_clk_div_tab[0] = op_pix_clk_div_tab[1] = op_pix_clk_div_tab[2] = ctx->pixelDepth;
  }

  uint32_t pre_pll_clk_div,pll_multiplier;
  uint32_t best_pre_pll_clk_div=0,best_pll_multiplier=0;
  uint32_t best_vt_sys_clk_div=0, best_vt_pix_clk_div=0, best_op_sys_clk_div=0, best_op_pix_clk_div=0, best_row_speed_10_8=0;
  uint32_t vt_sys_clk_div_index, vt_pix_clk_div_index, op_sys_clk_div_index, op_pix_clk_div_index, row_speed_10_8_index;
  float minError=-1;

  // explore all possibilities
  for (pre_pll_clk_div=PRE_PLL_CLK_DIV_MIN;pre_pll_clk_div<=PRE_PLL_CLK_DIV_MAX;pre_pll_clk_div++)
  {
    for (pll_multiplier=PLL_MUL_MIN; pll_multiplier<=PLL_MUL_MAX;pll_multiplier++)
    {
      if (pll_multiplier%2==0)
      {
        // even mul, check range
        if (pll_multiplier < PLL_MUL_EVEN_MIN || pll_multiplier > PLL_MUL_EVEN_MAX)
        {
          // out of range
          continue;
        }
      }
      else
      {
        // odd mul, check range
        if (pll_multiplier < PLL_MUL_ODD_MIN || pll_multiplier > PLL_MUL_ODD_MAX)
        {
          // out of range
          continue;
        }
      }
      for (vt_sys_clk_div_index=0; vt_sys_clk_div_index<SIZEOF_TAB(vt_sys_clk_div_tab); vt_sys_clk_div_index++)
      {
        for (vt_pix_clk_div_index=0; vt_pix_clk_div_index<SIZEOF_TAB(vt_pix_clk_div_tab); vt_pix_clk_div_index++)
        {
          for (op_sys_clk_div_index=0; op_sys_clk_div_index<SIZEOF_TAB(op_sys_clk_div_tab); op_sys_clk_div_index++)
          {
            for (op_pix_clk_div_index=0; op_pix_clk_div_index<SIZEOF_TAB(op_pix_clk_div_tab); op_pix_clk_div_index++)
            {
              for (row_speed_10_8_index=0; row_speed_10_8_index<SIZEOF_TAB(row_speed_10_8_tab); row_speed_10_8_index++)
              {
                // if pll input clock is out of range, cancel this option
                float virtual_pll_ip_clk_freq = ctx->inputClkFreq/pre_pll_clk_div;
                if ( virtual_pll_ip_clk_freq > PLL_INPUT_CLK_MAX || virtual_pll_ip_clk_freq < PLL_INPUT_CLK_MIN)
                {
                  continue;
                }

                // check that we remain in the VCO range
                float virtualVCOfreq = virtual_pll_ip_clk_freq*pll_multiplier;
                if (virtualVCOfreq < VCO_CLK_MIN || virtualVCOfreq > VCO_CLK_MAX)
                {
                  // VCO out of range, cancel this option
                  continue;
                }

                // check that clk_op <= clk_pixel (from EQ 5 and EQ 4)
                if ((vt_sys_clk_div_tab[vt_sys_clk_div_index] * vt_pix_clk_div_tab[vt_pix_clk_div_index] * ctx->pll.rowSpeed_2_0 ) > (op_sys_clk_div_tab[op_sys_clk_div_index] * op_pix_clk_div_tab[op_pix_clk_div_index] * row_speed_10_8_tab[row_speed_10_8_index])  )
                {
                  // with this settings clk_op would be greater than clk_pixel, forget it !
                  continue;
                }

                // check that we remain in clk_pixel range
                // try to run clk_pixel as close as possible to 120MHz (max value)
                float virtual_clk_pixel = virtualVCOfreq / (vt_sys_clk_div_tab[vt_sys_clk_div_index] * vt_pix_clk_div_tab[vt_pix_clk_div_index] * ctx->pll.rowSpeed_2_0); // (EQ 4)
                if (virtual_clk_pixel > PIXEL_CLK_MAX)
                {
                  continue;
                }

                float virtual_clk;
                if (ctx->interface == HAL_MT9F002_Parallel)
                {
                // compute pixel clock (named clk_op in the datasheet)
                  virtual_clk = virtualVCOfreq/(op_sys_clk_div_tab[op_sys_clk_div_index] * op_pix_clk_div_tab[op_pix_clk_div_index] * row_speed_10_8_tab[row_speed_10_8_index]);
                }
                else
                {
                  // compute serial clock (named op_sys_clk in the datasheet)
                  virtual_clk = virtualVCOfreq/op_sys_clk_div_tab[op_sys_clk_div_index];
                }

                // if this solution is best we've seen so far, save it
                float error = fabs(virtual_clk-ctx->targetOutputClkFreq);
                if (error < minError || minError<0)
                {
                  minError = error;
                  best_vt_sys_clk_div =  vt_sys_clk_div_index;
                  best_vt_pix_clk_div =  vt_pix_clk_div_index;
                  best_op_sys_clk_div =  op_sys_clk_div_index;
                  best_op_pix_clk_div =  op_pix_clk_div_index;
                  best_row_speed_10_8 = row_speed_10_8_index;
                  best_pre_pll_clk_div = pre_pll_clk_div;
                  best_pll_multiplier = pll_multiplier;
                }
              }
            }
          }
        }
      }
    }
  }

  // check that an option has been found
  if (minError<0)
  {
    return -1;
  }

  // save values
  ctx->pll.op_sys_clk_div = op_sys_clk_div_tab[best_op_sys_clk_div];
  ctx->pll.vt_sys_clk_div = vt_sys_clk_div_tab[best_vt_sys_clk_div];
  ctx->pll.vt_pix_clk_div = vt_pix_clk_div_tab[best_vt_pix_clk_div];
  ctx->pll.op_pix_clk_div = op_pix_clk_div_tab[best_op_pix_clk_div];
  ctx->pll.row_speed_10_8 = row_speed_10_8_tab[best_row_speed_10_8];
  ctx->pll.pll_multiplier = best_pll_multiplier;
  ctx->pll.pre_pll_clk_div = best_pre_pll_clk_div;

  // compute&save some relevant clock values
  // compute vt_pix_clk (EQ 3)
  ctx->clock.vt_pix_clk = ctx->inputClkFreq*(float)ctx->pll.pll_multiplier*(float)(1+ctx->pll.shift_vt_pix_clk_div)
            /((float)ctx->pll.pre_pll_clk_div*(float)ctx->pll.vt_sys_clk_div*(float)ctx->pll.vt_pix_clk_div);
  // compute op_pix_clk
  ctx->clock.op_pix_clk=ctx->inputClkFreq*(float)ctx->pll.pll_multiplier
       /((float)ctx->pll.pre_pll_clk_div*(float)ctx->pll.op_sys_clk_div*(float)ctx->pll.op_pix_clk_div);

  return 0;
}

int pllCheck(HAL_MT9F002_context_t* ctx)
{
  int res = 0;

  /*
   *  check ext_clk_freq_mhz (inputClk) is in the range
   */
  if (ctx->inputClkFreq < EXT_CLK_MIN || ctx->inputClkFreq > EXT_CLK_MAX)
  {
    HAL_MT9F002_PRINT("error,input clock (%f MHz) not in the allowed range\n",ctx->inputClkFreq);
    res = -1;
  }

  /*
   * check pll_ip_clk_freq is in the range
   */
  if (ctx->pll.pre_pll_clk_div < PRE_PLL_CLK_DIV_MIN || ctx->pll.pre_pll_clk_div > PRE_PLL_CLK_DIV_MAX)
  {
    HAL_MT9F002_PRINT("error, pre_pll_clk_div (%d) not in the allowed range\n",ctx->pll.pre_pll_clk_div);
    res = -1;
  }

  float pll_ip_clk_freq = ctx->inputClkFreq/(float)(ctx->pll.pre_pll_clk_div); // (EQ 1)
  if ((float)pll_ip_clk_freq < PLL_INPUT_CLK_MIN || (float)pll_ip_clk_freq > PLL_INPUT_CLK_MAX)
  {
    HAL_MT9F002_PRINT("error, pll_ip_clk_freq (%f) not in the allowed range\n",pll_ip_clk_freq);
    res = -1;
  }

  /*
   *  check VCO is in the range
   */
  if (ctx->pll.pll_multiplier%2)
  {
    // odd case
    if (ctx->pll.pll_multiplier < PLL_MUL_ODD_MIN || ctx->pll.pll_multiplier > PLL_MUL_ODD_MAX)
    {
      HAL_MT9F002_PRINT("error, odd pre_pll_clk_div (%d) not in the allowed range\n",ctx->pll.pll_multiplier);
      res = -1;
    }
  }
  else
  {
    // even case
    if (ctx->pll.pll_multiplier < PLL_MUL_EVEN_MIN || ctx->pll.pll_multiplier > PLL_MUL_EVEN_MAX)
    {
      HAL_MT9F002_PRINT("error, even pre_pll_clk_div (%d) not in the allowed range\n",ctx->pll.pll_multiplier);
      res = -1;
    }
  }

  float vco_freq = pll_ip_clk_freq*(float)ctx->pll.pll_multiplier; // (EQ 2)
  if (vco_freq < VCO_CLK_MIN || vco_freq > VCO_CLK_MAX)
  {
    HAL_MT9F002_PRINT("error, vco_freq (%f) not in the allowed range\n",vco_freq);
    res = -1;
  }

  /*
   *  check clk_op < clk_pixel
   */
  float clk_pixel = ctx->inputClkFreq*(float)ctx->pll.pll_multiplier*(float)(1+ctx->pll.shift_vt_pix_clk_div)
          /((float)ctx->pll.pre_pll_clk_div*(float)ctx->pll.vt_sys_clk_div*(float)ctx->pll.vt_pix_clk_div*2.0*(float)ctx->pll.rowSpeed_2_0); // (EQ 4)

  float clk_op = ctx->inputClkFreq*(float)ctx->pll.pll_multiplier
       /((float)ctx->pll.pre_pll_clk_div*(float)ctx->pll.op_sys_clk_div*(float)ctx->pll.op_pix_clk_div*(float)ctx->pll.row_speed_10_8); // (EQ 5)

  if (clk_op > clk_pixel)
  {
    HAL_MT9F002_PRINT("error, clk_op (%f) runs faster than clk_pixel (%f)\n",clk_op,clk_pixel);
    res = -1;
  }

  /*
   *  When the serial interface is used the clk_op divider cannot be used; row_speed[10:8] must equal 1
   */
  if (ctx->interface == HAL_MT9F002_MIPI ||
      ctx->interface == HAL_MT9F002_HiSPi)
  {
    if(ctx->pll.row_speed_10_8 != 1)
    {
      HAL_MT9F002_PRINT("error, wrong row_speed_10_8 value (%d) intead of 1\n",ctx->pll.row_speed_10_8);
      res = -1;
    }
  }

  /*
    The value of op_sys_clk_div must match the bit-depth of the image when using serial
    interface. R0x0112-3 controls whether the pixel data interface will generate 12, 10, or 8
    bits per pixel. When the pixel data interface is generating 8 bits per-pixel,
    op_pix_clk_div must be programmed with the value 8. When the pixel data interface
    is generating 10 bits per pixel, op_pix_clk_div must be programmed with the value 10.
    And when the pixel data interface is generating 12 bits per pixel, op_pix_clk_div must
    be programmed with the value 12. This is not required when using the parallel inter-face.
   */
  if (ctx->interface == HAL_MT9F002_MIPI ||
      ctx->interface == HAL_MT9F002_HiSPi)
  {
    if(ctx->pll.op_pix_clk_div != ctx->pixelDepth)
    {
      HAL_MT9F002_PRINT("error, wrong op_pix_clk_div expected 0x%x get 0x%x\n",ctx->pixelDepth,ctx->pll.op_pix_clk_div);
      res = -1;
    }
  }

#if 0
  /*
    Although the PLL VCO input frequency range is advertised as 2-24 MHz, superior
    performance (better PLL stability) is obtained by keeping the VCO input frequency as
    high as possible.
   */

  // compute vt_pix_clk (EQ 3)
  float vt_pix_clk = ctx->inputClkFreq*(float)ctx->pll.pll_multiplier*(float)(1+ctx->pll.shift_vt_pix_clk_div)
            /((float)ctx->pll.pre_pll_clk_div*(float)ctx->pll.vt_sys_clk_div*(float)ctx->pll.vt_pix_clk_div);
  HAL_MT9F002_PRINT("info, vt_pix_clk %f MHz\n",vt_pix_clk);
  // compute op_pix_clk
  float op_pix_clk=ctx->inputClkFreq*(float)ctx->pll.pll_multiplier
       /((float)ctx->pll.pre_pll_clk_div*(float)ctx->pll.op_sys_clk_div*(float)ctx->pll.op_pix_clk_div);
  HAL_MT9F002_PRINT("info, op_pix_clk %f MHz\n",op_pix_clk);


  HAL_MT9F002_PRINT("info, clock stability check : try to keep pll_ip_clk_freq (%f) as close as possible to PLL_INPUT_CLK_MAX (%f)\n",pll_ip_clk_freq,PLL_INPUT_CLK_MAX);

  uint32_t pllDivider = ctx->pll.pre_pll_clk_div*ctx->pll.op_sys_clk_div*ctx->pll.op_pix_clk_div*ctx->pll.row_speed_10_8;
  HAL_MT9F002_PRINT("info, pll conf: %d/%d\n",ctx->pll.pll_multiplier,pllDivider);

  // compute serial clock
  float op_sys_clk=ctx->inputClkFreq*(float)ctx->pll.pll_multiplier
      /((float)ctx->pll.pre_pll_clk_div*(float)ctx->pll.op_sys_clk_div);

  if (ctx->interface == HAL_MT9F002_Parallel)
  {
  HAL_MT9F002_PRINT("info, final PIXEL_CLK : %f MHz\n",clk_op);
  }
  else
  {
    HAL_MT9F002_PRINT("info, final SERIAL_CLK : %f MHz\n",op_sys_clk);
  }
#endif
  return res;
}

int pllWrite(HAL_MT9F002_context_t* ctx)
{
  int res = 0;
  HAL_private_MT9F002_pll_t* pll = &ctx->pll;

  res |= writeReg(&ctx->i2c, VT_PIX_CLK_DIV , pll->vt_pix_clk_div);
  res |= writeReg(&ctx->i2c, VT_SYS_CLK_DIV , pll->vt_sys_clk_div);
  res |= writeReg(&ctx->i2c, PRE_PLL_CLK_DIV, pll->pre_pll_clk_div);
  res |= writeReg(&ctx->i2c, PLL_MULTIPLIER , pll->pll_multiplier);
  res |= writeReg(&ctx->i2c, OP_PIX_CLK_DIV , pll->op_pix_clk_div);
  res |= writeReg(&ctx->i2c, OP_SYS_CLK_DIV , pll->op_sys_clk_div);

  uint32_t smia;
  res |= readReg(&ctx->i2c,SMIA_TEST,&smia);
  smia = (smia&0xFFFFFFBF) | ((pll->shift_vt_pix_clk_div & 0x01)<<6);
  res |= writeReg(&ctx->i2c ,  SMIA_TEST  , smia);

  uint32_t row_speed;
  res |= readReg(&ctx->i2c,ROW_SPEED,&row_speed);
  row_speed = (row_speed&0xFFFFFFF8) | (pll->rowSpeed_2_0 & 0x07);
  row_speed = (row_speed&0xFFFFF8FF) | ((pll->row_speed_10_8 & 0x07)<<8);
  // from ASIC team : change opclk_delay
  row_speed = (row_speed&(~0x70)) | (0x2<<4);
  res |= writeReg(&ctx->i2c ,  ROW_SPEED  ,row_speed );

  return res;
}

int posWrite(HAL_MT9F002_context_t* ctx)
{
  int res=0;
  res |= writeReg(&ctx->i2c ,X_ADDR_START ,ctx->pos.offsetX);
  res |= writeReg(&ctx->i2c ,X_ADDR_END   ,ctx->pos.offsetX + ctx->sensorRes.width - 1);
  res |= writeReg(&ctx->i2c ,Y_ADDR_START ,ctx->pos.offsetY);
  res |= writeReg(&ctx->i2c ,Y_ADDR_END   ,ctx->pos.offsetY + ctx->sensorRes.height - 1);

  return res;
}

int resWrite(HAL_MT9F002_context_t* ctx)
{
  int res = 0;
  /* output resolution */
  res |= writeReg(&ctx->i2c ,X_OUTPUT_SIZE,ctx->res.width);
  res |= writeReg(&ctx->i2c ,Y_OUTPUT_SIZE,ctx->res.height);
  /* scaler */
  if (ctx->scaler != 1.0)
  {
    // enable scaling mode
    res |= writeReg(&ctx->i2c ,SCALING_MODE,2); // vertical and horizontal
    uint32_t scaleFactor = ceil(SCALER_N/ctx->scaler);
    res |= writeReg(&ctx->i2c ,SCALE_M, scaleFactor);
  }

  return res;
}

static int gainWrite(HAL_i2ctoolContext_t* ctx, regDef_t reg, float gainValue)
{
  int res=0;

  pixelGain_t pixelGain;

  if (gainValue < 1.0)
  {
    HAL_MT9F002_PRINT("error, gain (%f) too low (<1.0)\n",gainValue);
    res = -1;
  }

  if (!res)
  {
    // table 19 p56
    if (gainValue < 1.50)
    {
      // warning, gain < 1.5 are not recommended
      HAL_MT9F002_PRINT("warning, gain (%f) <1.5 is not recommended\n",gainValue);
      pixelGain.colamp_gain = 0;
      pixelGain.analog_gain3 = 0;
      pixelGain.digital_gain = 1;
    }
    else if (gainValue < 3.0)
    {
      pixelGain.colamp_gain = 1;
      pixelGain.analog_gain3 = 0;
      pixelGain.digital_gain = 1;
    }
    else if (gainValue < 6.0)
    {
      pixelGain.colamp_gain = 2;
      pixelGain.analog_gain3 = 0;
      pixelGain.digital_gain = 1;
    }
    else if (gainValue < 16.0)
    {
      pixelGain.colamp_gain = 3;
      pixelGain.analog_gain3 = 0;
      pixelGain.digital_gain = 1;
    }
    else if (gainValue < 32.0)
    {
      pixelGain.colamp_gain = 3;
      pixelGain.analog_gain3 = 0;
      pixelGain.digital_gain = 2;
    }
    else
    {
      pixelGain.colamp_gain = 3;
      pixelGain.analog_gain3 = 0;
      pixelGain.digital_gain = 4;
    }

    // compute pixelGain.analog_gain2
    uint32_t analogGain = gainValue/(float)pixelGain.digital_gain
                      /((float)(1<<pixelGain.colamp_gain))
                      /((float)(1<<pixelGain.analog_gain3))
                      * 64.0;

    if (analogGain == 0)
    {
      HAL_MT9F002_PRINT("error, analogGain (%d) too low (<1) \n",analogGain);
      analogGain = 1;
      res = -1;
    }
    else if (analogGain > 127)
    {
      HAL_MT9F002_PRINT("error, analogGain (%d) too high (>127) \n",analogGain);
      analogGain = 127;
      res = -1;
    }

    if (analogGain < 48)
    {
      HAL_MT9F002_PRINT("warning, analogGain (%d) <48 is not recommended\n",analogGain);
    }

    pixelGain.analog_gain2 = analogGain;

    // write value
    res |= writeReg(ctx, reg, pixelGain.val);
  }

  return res;
}

static int gainsWrite(HAL_MT9F002_context_t* ctx)
{
  int res=0;
  res |= gainWrite(&ctx->i2c,GREEN1_GAIN, ctx->colorGain.greenR);
  res |= gainWrite(&ctx->i2c,BLUE_GAIN,   ctx->colorGain.blue);
  res |= gainWrite(&ctx->i2c,RED_GAIN,    ctx->colorGain.red);
  res |= gainWrite(&ctx->i2c,GREEN2_GAIN, ctx->colorGain.greenB);
  return res;
}

static int exposureInit(HAL_MT9F002_context_t* ctx)
{
  int res=0;
  HAL_private_MT9F002_exposure_t* exposure = &ctx->exposure;
  res |= readReg(&ctx->i2c, FINE_INTEGRATION_TIME_MIN, &exposure->fine_integration_time_min );
  res |= readReg(&ctx->i2c, FINE_INTEGRATION_TIME_MAX_MARGIN, &exposure->fine_integration_time_max_margin );
  res |= readReg(&ctx->i2c, COARSE_INTEGRATION_TIME_MIN, &exposure->coarse_integration_time_min );
  res |= readReg(&ctx->i2c, COARSE_INTEGRATION_TIME_MAX_MARGIN, &exposure->coarse_integration_time_max_margin );

  return res;
}

static void exposureMax(HAL_MT9F002_context_t* ctx)
{
  // compute maximum exposure
  HAL_private_MT9F002_exposure_t* exposure = &ctx->exposure;
  HAL_private_MT9F002_blanking_t* blanking = &ctx->blanking;
  HAL_private_MT9F002_clock_t* clock = &ctx->clock;
  uint32_t max_fine_integration_time = (blanking->line_length - exposure->fine_integration_time_max_margin);
  uint32_t max_coarse_integration_time = (blanking->frame_length - exposure->coarse_integration_time_max_margin);
  ctx->maxExposure_ms = (float)(max_coarse_integration_time*blanking->line_length + max_fine_integration_time)/(clock->vt_pix_clk*1000);
  HAL_MT9F002_PRINT("info, maximum exposure time %f ms\n",ctx->maxExposure_ms);
}

static int exposureConf(HAL_MT9F002_context_t* ctx)
{
  int res=0;

  HAL_private_MT9F002_clock_t* clock = &ctx->clock;
  HAL_private_MT9F002_exposure_t* exposure = &ctx->exposure;
  HAL_private_MT9F002_blanking_t* blanking = &ctx->blanking;

  // compute fine and coarse integration time
  uint32_t integration = ctx->targetExposure_ms*clock->vt_pix_clk*1000;
  exposure->coarse_integration_time = integration/blanking->line_length;
  exposure->fine_integration_time = integration%blanking->line_length;

  // fine integration must be in a specific range, round exposure if it's not the case
  if ( (exposure->fine_integration_time_min > exposure->fine_integration_time) ||
       (exposure->fine_integration_time > (blanking->line_length - exposure->fine_integration_time_max_margin)))
  {
    // test upper and lower value, take the closest
    int32_t upper_coarse_integration_time = exposure->coarse_integration_time+1;
    int32_t upper_fine_integration_time = exposure->fine_integration_time_min;

    int32_t lower_coarse_integration_time = exposure->coarse_integration_time-1;
    int32_t lower_fine_integration_time = blanking->line_length - exposure->fine_integration_time_max_margin;

    // test saturation
    if (lower_coarse_integration_time < 0)
    {
      // lower case is negative, reject it
      exposure->coarse_integration_time = upper_coarse_integration_time;
      exposure->fine_integration_time = upper_fine_integration_time;
    }
    else if (upper_coarse_integration_time > (blanking->frame_length - exposure->coarse_integration_time_max_margin))
    {
      // upper case is to high, reject it
      exposure->coarse_integration_time = lower_coarse_integration_time;
      exposure->fine_integration_time = lower_fine_integration_time;
    }
    else
    {
      // both case are correct values

      // which one is better, upper or lower case ?
      int32_t upper_integration = blanking->line_length*upper_coarse_integration_time + upper_fine_integration_time;
      int32_t lower_integration = blanking->line_length*lower_coarse_integration_time + lower_fine_integration_time;

      // compute error
      int32_t upper_error = upper_integration - integration;
      int32_t lower_error = integration - lower_integration;

      // choose the best
      if (upper_error < lower_error)
      {
        // take upper case
        exposure->coarse_integration_time = upper_coarse_integration_time;
        exposure->fine_integration_time = upper_fine_integration_time;
      }
      else
      {
        // take lower case
        exposure->coarse_integration_time = lower_coarse_integration_time;
        exposure->fine_integration_time = lower_fine_integration_time;
      }
    }
  }

  //PLOGI(__MODULE_NAME__,"ctx->targetExposure_ms %f\n",ctx->targetExposure_ms);
  //PLOGI(__MODULE_NAME__,"coarse_integration_time %d\n",exposure->coarse_integration_time);
  //PLOGI(__MODULE_NAME__,"fine_integration_time %d\n",exposure->fine_integration_time);

  //PLOGI(__MODULE_NAME__,"\nfine_integration_time_min %d\n",exposure.fine_integration_time_min);
  //PLOGI(__MODULE_NAME__,"line_length_pck %d\n",exposure.line_length);
  //PLOGI(__MODULE_NAME__,"fine_integration_time_max_margin %d\n",exposure.fine_integration_time_max_margin);

  // check limit
  if (exposure->fine_integration_time_min > exposure->fine_integration_time)
  {
    HAL_MT9F002_PRINT("warning, fine_integration_time too low (bottom limit %d)\n",exposure->fine_integration_time_min);
    exposure->fine_integration_time = exposure->fine_integration_time_min;
  }
  if (exposure->fine_integration_time > (blanking->line_length - exposure->fine_integration_time_max_margin))
  {
    HAL_MT9F002_PRINT("warning, fine_integration_time too high (upper limit %d)\n",(blanking->line_length - exposure->fine_integration_time_max_margin));
    exposure->fine_integration_time = (blanking->line_length - exposure->fine_integration_time_max_margin);
  }

  if (exposure->coarse_integration_time_min > exposure->coarse_integration_time)
  {
    HAL_MT9F002_PRINT("warning, coarse_integration_time too low (bottom limit %d)\n",exposure->coarse_integration_time_min);
    exposure->coarse_integration_time = exposure->coarse_integration_time_min;
    res = -1;
  }
  if (exposure->coarse_integration_time > (blanking->frame_length - exposure->coarse_integration_time_max_margin))
  {
    HAL_MT9F002_PRINT("warning, coarse_integration_time too high (upper limit %d)\n",(blanking->frame_length - exposure->coarse_integration_time_max_margin));
    exposure->coarse_integration_time = (blanking->frame_length - exposure->coarse_integration_time_max_margin);
    res = -1;
  }

  // compute final time exposure
  ctx->realExposure_ms = (float)(exposure->coarse_integration_time*blanking->line_length + exposure->fine_integration_time)/(clock->vt_pix_clk*1000);
  if (res == -1)
  {
    HAL_MT9F002_PRINT("warning, exposure saturation, ask for %f ms, final time exposure %f ms\n",ctx->targetExposure_ms,ctx->realExposure_ms);
  }

  return res;
}

static int exposureWrite(HAL_MT9F002_context_t* ctx)
{
  int res = 0;

  HAL_private_MT9F002_exposure_t* exposure = &ctx->exposure;

  // write values
  res |= writeReg(&ctx->i2c, COARSE_INTEGRATION_TIME, exposure->coarse_integration_time);
  res |= writeReg(&ctx->i2c, FINE_INTEGRATION_TIME, exposure->fine_integration_time);
  return res;
}

//TODO: separate configuration and register write, so that we can surround write with sensor commands group option.
//TODO: do that for other functions such as color gain, framerate ...
int blankingInit(HAL_MT9F002_context_t* ctx)
{
  int res = 0;
  HAL_private_MT9F002_blanking_t* blanking = &ctx->blanking;
  res |= readReg(&ctx->i2c,MIN_LINE_BLANKING_PCK,&blanking->min_line_blanking_pck);
  res |= readReg(&ctx->i2c,X_ODD_INC,&blanking->x_odd_inc);
  res |= readReg(&ctx->i2c,MIN_FRAME_BLANKING_LINES,&blanking->min_frame_blanking_lines);
  res |= readReg(&ctx->i2c,MIN_LINE_LENGTH_PCK,&blanking->min_line_length_pck);
  return res;
}

int blankingConf(HAL_MT9F002_context_t* ctx)
{
  int res=0;
  HAL_private_MT9F002_blanking_t* blanking = &ctx->blanking;
  HAL_private_MT9F002_clock_t* clock = &ctx->clock;
  //uint32_t subsampling_factor=1; // TODO: assume no subsampling, don't know where to read it
  float subsampling_factor = (float)(1+blanking->x_odd_inc)/(float)2; // not really sure (see p52)
  uint32_t minimum_line_length;

  // compute minimum line length
  minimum_line_length = blanking->min_line_length_pck;
  minimum_line_length = MAX(minimum_line_length,(ctx->sensorRes.width)/subsampling_factor + blanking->min_line_blanking_pck); // (EQ 9)
  minimum_line_length = MAX(minimum_line_length,(ctx->sensorRes.width -1 + blanking->x_odd_inc)/subsampling_factor/2 + blanking->min_line_blanking_pck);
  if (ctx->interface == HAL_MT9F002_MIPI ||
      ctx->interface == HAL_MT9F002_HiSPi )
  {
    minimum_line_length = MAX(minimum_line_length,((uint32_t)((float)ctx->sensorRes.width * clock->vt_pix_clk / clock->op_pix_clk)/ctx->numberOfLanes) + 0x005E);
  }
  else
  {
    minimum_line_length = MAX(minimum_line_length,((uint32_t)((float)ctx->sensorRes.width * clock->vt_pix_clk / clock->op_pix_clk)) + 0x005E);
  }

  // at this point we have the minimum line length according to the data sheet.
  // Life is not that easy. Sometime minimum_line_length is not large enough.
  // Therefore we increase a bit minimum_line_length
  //minimum_line_length = ((uint32_t)(1.2*minimum_line_length));

  // It's not over... P7 ISP fpga request that length_line = 2*n :
  // sensor blanking is base on vt_pix_clk, but fpga blanking is based on op_pix_clk
  // we have to found an integer length_line corresponding to an integer even fpga lenth_line

  // first we compute clkRatio fraction
  uint32_t clkRatio_num = ctx->pll.op_sys_clk_div*ctx->pll.op_pix_clk_div*ctx->pll.row_speed_10_8*(1+ctx->pll.shift_vt_pix_clk_div);
  uint32_t clkRatio_den = ctx->pll.vt_sys_clk_div*ctx->pll.vt_pix_clk_div;

  // reduce fraction (lazy way)
  int i=clkRatio_den;
  while (i>1)
  {
    if (clkRatio_den%i==0 && clkRatio_num%i==0)
    {
      clkRatio_den = clkRatio_den/i;
      clkRatio_num = clkRatio_num/i;
    }
    i--;
  }

  // then we adjust length_line to meet all the requirement
  // fpga length_line must me divisible by 2 (fpga constraint) and sensor lenght_line by clkRatio_den (so that length_line is an integer in vt based and op based)
  uint32_t minHBlkStep = clkRatio_num;
  if (clkRatio_den%2 != 0)
  {
    minHBlkStep*=2;
  }

  uint32_t fpgaCorrection = (minimum_line_length%minHBlkStep);
  if (fpgaCorrection)
  {
    minimum_line_length += minHBlkStep-fpgaCorrection;
  }

  // save  minimum line length
  blanking->minimum_line_length = minimum_line_length;

  // compute minimum frame length
  blanking->minimum_frame_length_lines = (ctx->sensorRes.height)/subsampling_factor + blanking->min_frame_blanking_lines; // (EQ 10)

  // compute max framerate
  ctx->maxFps = clock->vt_pix_clk*1000000/(float)(blanking->minimum_line_length*blanking->minimum_frame_length_lines);
  HAL_MT9F002_PRINT("info, max framerate %f fps\n",ctx->maxFps);

  if (ctx->targetFps > ctx->maxFps)
  {
    res = -1;
    HAL_MT9F002_PRINT("warning, unreachable framerate (%f fps) \n",ctx->targetFps);

    // set highest possible framerate, that is to say minimum line_length and minimum frame_length_lines
    blanking->frame_length = blanking->minimum_frame_length_lines;
    blanking->line_length = blanking->minimum_line_length;
  }
  else
  {
    // look for the best line_length/frame_length_lines couple to reach desired framerate
    uint32_t best_line_length=0,best_frame_length_lines=0;
    uint32_t line_length,frame_length_lines;
    float minError=-1;

    for (line_length=blanking->minimum_line_length; line_length<=LINE_LENGTH_MAX; line_length+=minHBlkStep)
    {
      // compute framerate
      float fps = clock->vt_pix_clk*1000000/(line_length*blanking->minimum_frame_length_lines);
      if(fps<ctx->targetFps)
      {
        // don't go further
        break;
      }

      for (frame_length_lines=blanking->minimum_frame_length_lines; frame_length_lines<=LINE_LENGTH_MAX; frame_length_lines++)
      {
        // compute framerate
        float fps = clock->vt_pix_clk*1000000/(line_length*frame_length_lines);

        // if this solution is the best we've seen so far, save it
        float error = fabs(fps-ctx->targetFps);
        if (error < minError || minError<0)
        {
          minError=error;
          best_line_length=line_length;
          best_frame_length_lines=frame_length_lines;
        }
        if(fps<ctx->targetFps)
        {
          // don't go further
          break;
        }
      }
    }

    // check that an option has been found, should not happen
    if (minError<0)
    {
      HAL_MT9F002_PRINT("error, failed to configure fps\n");
      return -1;
    }

    // save config
    blanking->line_length = best_line_length;
    blanking->frame_length = best_frame_length_lines;
  }

  // compute final fps
  ctx->realFps = clock->vt_pix_clk*1000000/(float)(blanking->line_length*blanking->frame_length);
  ctx->FramePeriod_us = 1000000/ctx->realFps;
  HAL_MT9F002_PRINT("info, final framerate %f fps\n",ctx->realFps);

	// compute inter frame blanking length
  ctx->blanking.blkHV_us =  (float)(blanking->line_length*(blanking->frame_length-ctx->sensorRes.height))/(clock->vt_pix_clk);
  HAL_MT9F002_PRINT("info, HV blanking %d µs\n",ctx->blanking.blkHV_us);

  // compute line_lengh_ms
  ctx->blanking.line_length_ms =  (float)blanking->line_length/(clock->vt_pix_clk)/1000;
#if 0
  HAL_MT9F002_PRINT("TotalW %d (vt_pix_clk base) TotalW %d (clk_op base) TotalH %d\n",blanking->line_length,blanking->line_length*clkRatio_den/clkRatio_num,blanking->frame_length);
  HAL_MT9F002_PRINT("Min TotalW %d (vt_pix_clk base) Min TotalW %d (clk_op base) Min TotalH %d\n",blanking->minimum_line_length,blanking->minimum_line_length*clkRatio_den/clkRatio_num,blanking->minimum_frame_length_lines);
#endif

  // update max exposition
  exposureMax(ctx);

  return res;
}

int blankingWrite(HAL_MT9F002_context_t* ctx)
{
  int res=0;
  HAL_private_MT9F002_blanking_t* blanking = &ctx->blanking;

  // write blanking
  res |= writeReg(&ctx->i2c, LINE_LENGTH_PCK, blanking->line_length);
  res |= writeReg(&ctx->i2c, FRAME_LENGTH_LINES, blanking->frame_length);

  return res;
}

/////////////////////////
// INIT
/////////////////////////

/*
 * Prepare a camera (open i2c, initialize camera context) (mandatory)
 */
int HAL_MT9F002_Init(HAL_MT9F002_context_t* ctx, char* i2cPath, float  inputClkFreq, float outputClkFreq, HAL_MT9F002_interface_t interface)
{
  int res;

  // check argument
  CHECK_NULL_CTX(ctx);

  if (i2cPath == NULL)
  {
    if (ctx == NULL)
    {
      HAL_MT9F002_PRINT("fatal error, i2cPath is null\n");
      ctx->state = HAL_MT9F002_FATAL_ERROR;
      return -1;
    }
  }

  // check interface
  if (interface !=  HAL_MT9F002_Parallel && interface != HAL_MT9F002_MIPI && interface != HAL_MT9F002_HiSPi)
  {
    if (ctx == NULL)
    {
      HAL_MT9F002_PRINT("fatal error, unknown interface\n");
      ctx->state = HAL_MT9F002_FATAL_ERROR;
      return -1;
    }
  }

  // check clocks
  if (inputClkFreq < EXT_CLK_MIN || inputClkFreq > EXT_CLK_MAX)
  {
    HAL_MT9F002_PRINT("fatal error, input clock (%f MHz) not in the allowed range [%fMHz - %fMHz]\n",inputClkFreq,EXT_CLK_MIN,EXT_CLK_MAX);
    ctx->state = HAL_MT9F002_FATAL_ERROR;
    return -1;
  }

  switch(interface)
  {
    case HAL_MT9F002_Parallel:
      if (outputClkFreq > PIXEL_CLOCK_MAX)
      {
        HAL_MT9F002_PRINT("fatal error, desired output clock (%f MHz) too high (>%f MHz)\n", outputClkFreq, PIXEL_CLOCK_MAX);
        ctx->state = HAL_MT9F002_FATAL_ERROR;
        return -1;
      }
      break;
    case HAL_MT9F002_HiSPi:
    case HAL_MT9F002_MIPI:
      if (outputClkFreq > SERIAL_CLOCK_MAX)
      {
        HAL_MT9F002_PRINT("fatal error, desired output clock (%f MHz) too high (>%f MHz)\n", outputClkFreq, SERIAL_CLOCK_MAX);
        return -1;
      }
      break;
    default:
      HAL_MT9F002_PRINT("fatal error, unknown interface type (%d)\n",interface);
      ctx->state = HAL_MT9F002_FATAL_ERROR;
      return -1;
  }

  // try to open i2c interface
  res = HAL_i2ctool_open(&ctx->i2c, i2cPath, HAL_MT9F002_I2C_ADDR, HAL_MT9F002_I2C_REGADDR_SIZE, HAL_MT9F002_I2C_REG_SIZE);
  if (res)
  {
    HAL_MT9F002_PRINT("error, could not open i2c '%s'\n", i2cPath);
    ctx->state = HAL_MT9F002_FATAL_ERROR;
    return -1;
  }

  // init mutex
  pthread_mutex_init(&ctx->stateEstimatorMutex,NULL);

  // set user param
  ctx->inputClkFreq = inputClkFreq;
  ctx->targetOutputClkFreq = outputClkFreq;
  ctx->interface = interface;

  // set default param
  ctx->pixelDepth        = init_MT9F002_pixelDepth;
  ctx->numberOfLanes     = init_MT9F002_serial_lanes;
  ctx->colorBar          = init_MT9F002_Colorabar;
  ctx->streamOn          = init_MT9F002_StreamOn;
  ctx->colorGain         = init_MT9F002_colorGain;
  ctx->res               = init_MT9F002_res;
  ctx->sensorRes         = init_MT9F002_res;
  ctx->scaler            = init_MT9F002_scaler;
  ctx->pos               = init_MT9F002_pos;

  // init state estimator
  ctx->stateEstimatorI=0;
  ctx->stateEstimator[0].set=1;
  ctx->stateEstimator[0].pos=init_MT9F002_pos;

  ctx->targetFps         = init_MT9F002_fps;
  ctx->targetExposure_ms = init_MT9F002_exposure_ms;
  ctx->lastFrameTimestamp.tv_sec = 0;
  ctx->lastFrameTimestamp.tv_nsec = 0;

  // everything ok, move to next state
  ctx->state = HAL_MT9F002_INIT_STATE;

  return 0;
}

/////////////////////////
// CONGFIGURATION
/////////////////////////

/* after HAL_MTPF002_Open() following functions can be optionaly called before HAL_MT9F002_Open() */
int HAL_MT9F002_SetPixelDepth(HAL_MT9F002_context_t* ctx, HAL_MT9F002_pixelDepth_t pixelDepth)
{
  // check argument
  CHECK_NULL_CTX(ctx);

  // check state
  CHECK_STATE(ctx,HAL_MT9F002_INIT_STATE);

  // check pixelDepth is correct
  switch(pixelDepth)
  {
     case HAL_MT9F002_8bits:
     case HAL_MT9F002_10bits:
     case HAL_MT9F002_12bits:
       // correct value, save value
       ctx->pixelDepth = pixelDepth;
       break;

     default:
       HAL_MT9F002_PRINT("error, unsupported pixelDepth (%d)\n",pixelDepth);
       break;
  }

  return 0;
}

int HAL_MT9F002_SetResolution(HAL_MT9F002_context_t* ctx, HAL_MT9F002_res_t* res, float scaler)
{
  // check argument
  CHECK_NULL_CTX(ctx);

  // check state
  CHECK_STATE(ctx,HAL_MT9F002_INIT_STATE);
  // fit scaler to sensor
  uint32_t scaleFactor = ceil(SCALER_N/scaler);
  scaler = (float)SCALER_N/scaleFactor;

  // check resolution is correct
  uint32_t widthOnSensor = ceil((float)res->width/scaler);
  uint32_t heightOnSensor = ceil((float)res->height/scaler);
  if ( widthOnSensor > HAL_MT9F002_SENSOR_WIDTH ||
       heightOnSensor > HAL_MT9F002_SENSOR_HEIGHT)
  {
    HAL_MT9F002_PRINT("resolution error, tried to set %dx%d whereas sensor dimensions are %dx%d, lower resolution and/or increase scaler\n",widthOnSensor,heightOnSensor,HAL_MT9F002_SENSOR_WIDTH,HAL_MT9F002_SENSOR_HEIGHT);
    return -1;
  }

  // check scaler is in the allowed range
  if(scaler < SCALER_N/SCALER_M_MAX || scaler > SCALER_N/SCALER_M_MIN)
  {
    HAL_MT9F002_PRINT("scaler error, scaler (%f) out of range [%f,%f]\n",scaler,(float)SCALER_N/SCALER_M_MAX,(float)SCALER_N/SCALER_M_MIN);
    return -1;
  }

  // save resolution
  ctx->res = *res;
  ctx->sensorRes.width  = widthOnSensor;
  ctx->sensorRes.height = heightOnSensor;
  ctx->scaler = scaler;
  return 0;
}


int HAL_MT9F002_GetResolution(HAL_MT9F002_context_t* ctx, HAL_MT9F002_res_t* res, float* scaler)
{
  // check argument
  CHECK_NULL_CTX(ctx);

  // check state
  CHECK_STATE(ctx,HAL_MT9F002_INIT_STATE);

  if (res)
  {
    *res = ctx->res;
  }

  if (scaler)
  {
    *scaler = ctx->scaler;
  }

  return 0;
}

//TODO serial 3 lanes doesn't seem to be supported, check that
int HAL_MT9F002_SetNumberOfLanes(HAL_MT9F002_context_t* ctx, HAL_MT9F002_Serial_lanes_t numberOfLanes)
{
  // check argument
  CHECK_NULL_CTX(ctx);

  // check state
  CHECK_STATE(ctx,HAL_MT9F002_INIT_STATE);

  if (ctx->interface != HAL_MT9F002_MIPI && ctx->interface != HAL_MT9F002_HiSPi)
  {
    HAL_MT9F002_PRINT("warning, SetNumberOfLanes not available for parallel interface\n");
    return -1;
  }

  // check numberOfLane is correct
  switch(numberOfLanes)
  {
     case HAL_MT9F002_1lane:
     case HAL_MT9F002_2lane:
     case HAL_MT9F002_3lane:
     case HAL_MT9F002_4lane:
       // correct value, save it
       ctx->numberOfLanes = numberOfLanes;
       break;

     default:
       HAL_MT9F002_PRINT("error, unsupported number of lanes (%d)\n",numberOfLanes);
       return -1;
       break;
  }

  return 0;
}

/* next function flush config, after it has been call, configuration function are no more available */
int HAL_MT9F002_Open(HAL_MT9F002_context_t* ctx)
{
  // check argument
  CHECK_NULL_CTX(ctx);

  // check state
  CHECK_STATE(ctx,HAL_MT9F002_INIT_STATE);

  //reset camera
  if (softReset(ctx))
  {
    HAL_MT9F002_PRINT("fatal error, could not reset sensor\n");
    ctx->state = HAL_MT9F002_FATAL_ERROR;
    return -1;
  }

  if (ctx->interface == HAL_MT9F002_MIPI ||
      ctx->interface == HAL_MT9F002_HiSPi )
  {
    // mipi stage 1
    if (mipi_stage1_conf(ctx))
    {
      HAL_MT9F002_PRINT("fatal error, mipi conf stage 1 failed\n");
      ctx->state = HAL_MT9F002_FATAL_ERROR;
      return -1;
    }
  }
  else
  {
    parallel_stage1_conf(ctx);
  }

  // pll configuration
  if (pllConf(ctx))
  {
    HAL_MT9F002_PRINT("fatal error, could not configure sensor pll\n");
    ctx->state = HAL_MT9F002_FATAL_ERROR;
    return -1;
  }

  // pll final check
  if (pllCheck(ctx))
  {
    HAL_MT9F002_PRINT("fatal error, the generated pll configuration is bad, blame pierre.eline@parrot.com\n");
    ctx->state = HAL_MT9F002_FATAL_ERROR;
    return -1;
  }

  // configure pll
  if (pllWrite(ctx))
  {
    HAL_MT9F002_PRINT("fatal error, could not write pll configuration\n");
    ctx->state = HAL_MT9F002_FATAL_ERROR;
    return -1;
  }

  if (ctx->interface == HAL_MT9F002_MIPI ||
      ctx->interface == HAL_MT9F002_HiSPi )
  {
    // mipi stage 2
    if (mipi_stage2_conf(ctx))
    {
      HAL_MT9F002_PRINT("fatal error, mipi conf stage 2 failed\n");
      ctx->state = HAL_MT9F002_FATAL_ERROR;
      return -1;
    }
  }
  else
  {
    parallel_stage2_conf(ctx);
  }

  if (resWrite(ctx))
  {
    HAL_MT9F002_PRINT("fatal error, resolution configuration failed\n");
    ctx->state = HAL_MT9F002_FATAL_ERROR;
    return -1;
  }

  if (posWrite(ctx))
  {
    HAL_MT9F002_PRINT("fatal error, position configuration failed\n");
    ctx->state = HAL_MT9F002_FATAL_ERROR;
    return -1;
  }

  // read some releavant register for blanking computatation
  if (blankingInit(ctx))
  {
    HAL_MT9F002_PRINT("fatal error, blanking initialization failed\n");
    ctx->state = HAL_MT9F002_FATAL_ERROR;
    return -1;
  }

  // read some releavant register for exposition computation
  if (exposureInit(ctx))
  {
    HAL_MT9F002_PRINT("fatal error, exposition initialization failed\n");
    ctx->state = HAL_MT9F002_FATAL_ERROR;
    return -1;
  }

  // conf blanking
  blankingConf(ctx);

  if (blankingWrite(ctx))
  {
    HAL_MT9F002_PRINT("fatal error, blanking configuration failed\n");
    ctx->state = HAL_MT9F002_FATAL_ERROR;
    return -1;
  }

  // configure exposition
  exposureConf(ctx);

  if (exposureWrite(ctx))
  {
    HAL_MT9F002_PRINT("fatal error, exposure configuration failed\n");
    ctx->state = HAL_MT9F002_FATAL_ERROR;
    return -1;
  }

  //initialize gain
  gainsWrite(ctx);

  if (ctx->interface == HAL_MT9F002_MIPI ||
      ctx->interface == HAL_MT9F002_HiSPi )
  {
    // mipi stage 3
    if (mipi_stage3_conf(ctx))
    {
      HAL_MT9F002_PRINT("fatal error, mipi conf stage 2 failed\n");
      ctx->state = HAL_MT9F002_FATAL_ERROR;
      return -1;
    }
  }

  ctx->state = HAL_MT9F002_OPEN_STATE;
  return 0;
}


/////////////////////////
// RUNTIME
/////////////////////////
/* next function are available after HAL_MT9F002_Open() and can be call at any time */
HAL_MT9F002_Set_Result_t HAL_MT9F002_SetFrameRate(HAL_MT9F002_context_t* ctx, float fps)
{
  // check argument
  CHECK_NULL_CTX(ctx);

  // check state
  CHECK_STATE(ctx,HAL_MT9F002_OPEN_STATE);

  if (fps < 0)
  {
    HAL_MT9F002_PRINT("error, fps (%f) canno't be negative\n",fps);
    return -1;
  }

  // save fps
  ctx->targetFps = fps;

  HAL_MT9F002_Set_Result_t res = HAL_MT9F002_Set_Success;
  // compute blanking to reach target framerate
  if(blankingConf(ctx))
  {
    res = HAL_MT9F002_Set_Saturated;
  }

  // exposition depends on blanking, recompute it
  if(exposureConf(ctx))
  {
    res = HAL_MT9F002_Set_Saturated;
  }

  // write framerate and exposition
  int writeRes=0;
  writeRes |= writeReg(&ctx->i2c, GROUPED_PARAMETER_HOLD,1);
  writeRes |= blankingWrite(ctx);
  writeRes |= exposureWrite(ctx);
  writeRes |= writeReg(&ctx->i2c, GROUPED_PARAMETER_HOLD,0);

  if (writeRes)
  {
    res = HAL_MT9F002_Set_Failed;
  }

  return res;
}

int HAL_MT9F002_GetFrameRate(HAL_MT9F002_context_t* ctx, float* fps)
{
  // check argument
  CHECK_NULL_CTX(ctx);

  // check state
  CHECK_STATE(ctx,HAL_MT9F002_OPEN_STATE);

  if (!fps)
  {
    HAL_MT9F002_PRINT("error, fps is null\n");
    return -1;
  }

  *fps = ctx->realFps;

  return 0;
}

int HAL_MT9F002_GetTiming(HAL_MT9F002_context_t* ctx, HAL_MT9F002_timing_t* timing)
{
  // check argument
  CHECK_NULL_CTX(ctx);

  // check state
  CHECK_STATE(ctx,HAL_MT9F002_OPEN_STATE);

  if (!timing)
  {
    HAL_MT9F002_PRINT("error, timing is null\n");
    return -1;
  }

  timing->lineAcqDuration_s = ctx->blanking.line_length_ms/1000.0;
  timing->frameAcqDuration_s = timing->lineAcqDuration_s*(float)ctx->sensorRes.height;
  timing->framePeriod_s = 1.0/ctx->realFps;
  timing->frameBlanking_s = timing->framePeriod_s-timing->frameAcqDuration_s;

  return 0;
}


int HAL_MT9F002_GetMaxFrameRate(HAL_MT9F002_context_t* ctx, float* fps)
{
  // check argument
  CHECK_NULL_CTX(ctx);

  // check state
  CHECK_STATE(ctx,HAL_MT9F002_OPEN_STATE);

  *fps = ctx->maxFps;

  return 0;
}

int HAL_MT9F002_StreamOn(HAL_MT9F002_context_t* ctx, HAL_MT9F002_StreamOn_t on)
{
  // check argument
  CHECK_NULL_CTX(ctx);

  // check state
  CHECK_STATE(ctx,HAL_MT9F002_OPEN_STATE);

  int res=0;

  res = streamOn(ctx,on);

  if (res)
  {
    HAL_MT9F002_PRINT("error, failed to turn (%d) stream on(1)/off(0/-1)\n",on);
  }
  return res;
}

int HAL_MT9F002_SetColorBar(HAL_MT9F002_context_t* ctx, HAL_MT9F002_Colorbar_t colorBar)
{
  // check argument
  CHECK_NULL_CTX(ctx);

  // check state
  CHECK_STATE(ctx,HAL_MT9F002_OPEN_STATE);

  int res=0;
  if (colorBar == HAL_MT9F002_ColorBar_off)
  {
    res = writeReg(&ctx->i2c, TEST_PATTERN_MODE,0);
    /*if (ctx->interface == HAL_MT9F002_MIPI ||
        ctx->interface == HAL_MT9F002_HiSPi )
    {
      // HisPi test mode
      uint32_t hispi_control_status;
      res |= readReg(&ctx->i2c, HISPI_CONTROL_STATUS,&hispi_control_status);
      hispi_control_status &= (~0x80); // clear test_enable
      res |= writeReg(&ctx->i2c, HISPI_CONTROL_STATUS,hispi_control_status);

    }*/
  }
  else
  {
    /*if (ctx->interface == HAL_MT9F002_MIPI ||
        ctx->interface == HAL_MT9F002_HiSPi )
    {
      //res = writeReg(&ctx->i2c, TEST_PATTERN_MODE,4);


      // HisPi test mode
      uint32_t hispi_control_status;
      res |= readReg(&ctx->i2c, HISPI_CONTROL_STATUS,&hispi_control_status);
      hispi_control_status &= (~0xF0); // clear test_enable + test mode
      hispi_control_status |= 0xC0;    // set test_enable + square mode)
      res |= writeReg(&ctx->i2c, HISPI_CONTROL_STATUS,hispi_control_status);

    }
    else*/
    {
      res = writeReg(&ctx->i2c, TEST_PATTERN_MODE,1);
    }
  }

  if (res)
  {
    HAL_MT9F002_PRINT("error, failed to change test pattern mode (%d)\n",colorBar);
  }

  return res;
}

/*
 * print the estimated state buffer
 */
static void HAL_MT9F002_printEstState(HAL_MT9F002_context_t* ctx)
{
    int i;
    for (i=0;i<HAL_MT9F002_STATE_ESTIMATOR_SIZE;i++)
    {
        int index = (ctx->stateEstimatorI+i)%HAL_MT9F002_STATE_ESTIMATOR_SIZE;
        if (ctx->stateEstimator[index].set)
        {
            fprintf(stderr,"[%dx%d][bad_%d] ",ctx->stateEstimator[index].pos.offsetX,
                    ctx->stateEstimator[index].pos.offsetY,
                    ctx->stateEstimator[index].badFrame);
        }
        else
        {
            break;
        }
    }
    fprintf(stderr,"\n");
}

/*
 * fill stateEstimator array for frame n+stateOffset
 * @param  ctx                  camera context
 * @param  stateOffset          frame index to set (0 means current, 1 next frame, .... )
 * @return                      0 in case of success, -1 in case of error
 *
 * This function doesn't perform any synchronization. The caller is responsible for locking mutex.
 */
static int HAL_MT9F002_setEstState(HAL_MT9F002_context_t* ctx, uint32_t stateOffset)
{
    /* check stateOffset */
    if (stateOffset >= HAL_MT9F002_STATE_ESTIMATOR_SIZE)
    {
        HAL_MT9F002_PRINT("error, max state exceeded\n");
        return -1;
    }

    /* retrieve state @ stateOffset */
    HAL_private_MT9F002_stateEstimator_t* state = &ctx->stateEstimator[(ctx->stateEstimatorI+stateOffset)%HAL_MT9F002_STATE_ESTIMATOR_SIZE];
    HAL_private_MT9F002_stateEstimator_t* prevState = &ctx->stateEstimator[(ctx->stateEstimatorI+stateOffset-1)%HAL_MT9F002_STATE_ESTIMATOR_SIZE];

    /* fill state */
    state->pos = ctx->pos;
    state->set = 1;

    /*
     * Fill undetermined states.
     * Example :
     * Imagine User change the position for frame (n+2) with HAL_MT9F002_setEstState(ctx,2);
     * the stateEstimator array will looks like :
     * [CurrentState, UndeterminedState, Frame_n+2_state, .... ]
     *
     * To deal with this case, CurrentState is copied to state_n+1.
     */

    /* fill unset state */
    uint32_t i;
    for (i=1; i<stateOffset; i++)
    {
        if (!ctx->stateEstimator[(ctx->stateEstimatorI+i)%HAL_MT9F002_STATE_ESTIMATOR_SIZE].set)
        {
            /* copy state */
            ctx->stateEstimator[(ctx->stateEstimatorI+i)%HAL_MT9F002_STATE_ESTIMATOR_SIZE].pos = ctx->stateEstimator[(ctx->stateEstimatorI+i-1)%HAL_MT9F002_STATE_ESTIMATOR_SIZE].pos;
            ctx->stateEstimator[(ctx->stateEstimatorI+i)%HAL_MT9F002_STATE_ESTIMATOR_SIZE].set = 1;
        }
    }

    /*
     * set bad frame status
     * When acquisition window is moved along the Y axis, a bad frame is generated and skipped
     */
    if (state->pos.offsetY != prevState->pos.offsetY)
    {
        state->badFrame = 1;
    }

    return 0;
}

/*
 * Pop state estimation on incomming frame
 * @param ctx   camera context
 *
 * This function doesn't perform any synchronization. The caller is responsible for locking mutex.
 */
static int HAL_MT9F002_popEstState(HAL_MT9F002_context_t* ctx)
{
    /*
     * State can be popped, if next state exists.
     * While next state is a bad frame we must keep popping.
     */
    HAL_private_MT9F002_stateEstimator_t* currState = &ctx->stateEstimator[ctx->stateEstimatorI];
    HAL_private_MT9F002_stateEstimator_t* nextState = &ctx->stateEstimator[(ctx->stateEstimatorI+1)%HAL_MT9F002_STATE_ESTIMATOR_SIZE];

    /* pop */
    if  (nextState->set)
    {
        /* reset current state */
        currState->set = 0;
        currState->badFrame = 0;

        /* go to next state */
        ctx->stateEstimatorI = (ctx->stateEstimatorI+1)%HAL_MT9F002_STATE_ESTIMATOR_SIZE;

    }

    /* while we have bad frame, keep popping */
    currState = &ctx->stateEstimator[ctx->stateEstimatorI];
    nextState = &ctx->stateEstimator[(ctx->stateEstimatorI+1)%HAL_MT9F002_STATE_ESTIMATOR_SIZE];
    while (currState->badFrame && nextState->set)
    {
        /* reset current state */
        currState->set = 0;
        currState->badFrame = 0;

        /* update current state */
        currState = nextState;
        ctx->stateEstimatorI = (ctx->stateEstimatorI+1)%HAL_MT9F002_STATE_ESTIMATOR_SIZE;

        /* update next state */
        nextState = &ctx->stateEstimator[(ctx->stateEstimatorI+1)%HAL_MT9F002_STATE_ESTIMATOR_SIZE];
    }

    return 0;
}

/*
 * Get the idx th frame estimated state
 * @param ctx   camera context
 * @param idx   frame index, last received frame idx=0
 *
 * This function doesn't perform any synchronization. The caller is responsible for locking mutex.
 *
 */
static HAL_private_MT9F002_stateEstimator_t* HAL_MT9F002_getEstState(HAL_MT9F002_context_t* ctx, int idx)
{
    /*
     * While state is a bad frame we must keep running through the list.
     */
    int stateEstimatorI = ctx->stateEstimatorI;
    HAL_private_MT9F002_stateEstimator_t* currState = &ctx->stateEstimator[stateEstimatorI];
    HAL_private_MT9F002_stateEstimator_t* lastState=currState;
    HAL_private_MT9F002_stateEstimator_t* res=currState;
    while(currState->set && idx >= 0)
    {
        /* bad frames are not transmitted and don't count */
        if(!currState->badFrame)
        {
            idx--;
            res = currState;
        }
        lastState = currState;
        stateEstimatorI = (stateEstimatorI+1)%HAL_MT9F002_STATE_ESTIMATOR_SIZE;
        currState = &ctx->stateEstimator[stateEstimatorI];
    }

    if (idx >= 0)
    {
        /* we didn't find any record for the frame.
         * let's assume the last state record is fine.
         */
        res = lastState;
    }

    return res;
}

/*
 * @brief Get the next safe window to change position. The computed
 * safe window is delayed to be as close as possible to the next frame start.
 *
 * @param[in]  ctx                      camera context
 * @param[in]  userDelay_us             the safe window will be at least userDelay_us large
 * @param[out] safeWindowDelay_us       the safe window will open in safeWindowDelay_us (relative to function call date)
 * @param[out] nextStartFrameDate_us    changing position in the computed safe window will apply for frame
 *                                      starting in nextStartFrameDate_us (machine time)
 * @return                              0 in case of success, -1 in case of error
 *
 * The function doesn't look for bad frame. Thus the returned safe window may corresponds
 * to a bad frame that won't be output.
 */
int HAL_MT9F002_getNextSafeWindow(HAL_MT9F002_context_t* ctx,
                                  uint32_t userDelay_us,
                                  HAL_MT9F002_PosSafeWindow_t safeWindowConstraint,
                                  uint32_t* safeWindowDelay_us,
                                  uint64_t* nextStartFrameDate_us)
{
    // check argument
    CHECK_NULL_CTX(ctx);

    // check state
    CHECK_STATE(ctx,HAL_MT9F002_OPEN_STATE);

    // check argument
    if (!safeWindowDelay_us)
    {
        HAL_MT9F002_PRINT("error, safeWindowDelay_us is null\n");
        return -1;
    }

    if (!nextStartFrameDate_us)
    {
        HAL_MT9F002_PRINT("error, nextStartFrameDate_us is null\n");
        return -1;
    }

    int res = 0;

    /* does the user use the SignalFrame() function ? */
    if (ctx->lastFrameTimestamp.tv_sec==0 && ctx->lastFrameTimestamp.tv_nsec==0)
    {
        res = -1;
        HAL_MT9F002_PRINT("error, unable to find a safe window, missing SignalFrame() \n");
    }

    /*
     * The position must be set with care to be able to track the acquisition window
     * position.
     * The following scheme illustrates when it's safe to change the position and
     * when it will apply.
     *
     *
     * Safe window       |    *2*    |//|    *3*    |//|    *4*    |//|
     * Signal Frame             ->             ->             ->             ->
     * FV             ___|---1---|______|---2---|______|---3---|______|---4---|___
     *
     *
     * |//| represents the unsafe zone. It is (SET_POSITION_WRITE_DURATION_US + SET_POSITION_DEADLINE_WINDOW_US + userDelay_us)
     * long and ends when a new frame starts.
     *
     */

    /*
     * Check that the safe window is large enough.
     */
    uint32_t set_position_deadline_window_us = SET_POSITION_DEADLINE_WINDOW_US;
    if (safeWindowConstraint == HAL_MT9F002_PosSafeWindow_FrameOnly &&
            ctx->blanking.blkHV_us >  set_position_deadline_window_us  )
    {
        /*
         * user wants a safe window located in frame transmission window
         * we cheat by setting the sensor position deadline to the blanking duration
         */
        set_position_deadline_window_us = ctx->blanking.blkHV_us;
    }

    uint32_t unsafeWindowWidth_us = SET_POSITION_WRITE_DURATION_US + set_position_deadline_window_us + userDelay_us;
    if (ctx->FramePeriod_us < unsafeWindowWidth_us)
    {
        res = -1;
        HAL_MT9F002_PRINT("error, unable to find a safe window, userDelay_us (%d us) too large\n",userDelay_us);
    }

    if (!res)
    {
        /*
         * First identify if we are in a safe window and which one
         */

        /* get current time */
        struct timespec currentTime;
        clock_gettime(CLOCK_MONOTONIC,&currentTime);

        /* get last EOF timestamp */
        struct timespec lastFrameTimestamp;
        pthread_mutex_lock(&ctx->stateEstimatorMutex);
        {
            lastFrameTimestamp=ctx->lastFrameTimestamp;
        }
        pthread_mutex_unlock(&ctx->stateEstimatorMutex);

        /* how much time elapsed since last frame has been received ? */
        uint32_t diff = diffTime(&currentTime,&lastFrameTimestamp);

        /* identify in which safe window we are */
        uint32_t safeWindowIndex=0;

        /* we use a modulo to deal with the case of several trailing bad frame */
        safeWindowIndex = diff/ctx->FramePeriod_us;
        uint32_t diffModulo = diff%ctx->FramePeriod_us;

        /*
         * Where are we exactly compared to last EOF ?
         *
         * case 1 (low framerate or small unsafe zone) :
         *
         * Safe window               | 1 |/2/|   3   |
         * Signal Frame             ->              ->
         * FV             ___|-------|_______|-------|
         *
         * case 2 (high framerate and/or large unsafe zone) :
         * Safe window             |////2////|  3  |////4////|
         * Signal Frame             ->              ->
         * FV             ___|-------|_______|-------|_______|-------|
         */

        uint32_t frameStart_us=0; // relative to last EOF timestamp

        if((int)diffModulo < (int)(ctx->blanking.blkHV_us - unsafeWindowWidth_us))
        {
            /* we are in zone 1, it's safe */
        }
        else
        {
            /* we are in zone 2/3 or 4, the position will be set for frame n+2 */
            safeWindowIndex++;
            if (diffModulo > (ctx->FramePeriod_us + ctx->blanking.blkHV_us - unsafeWindowWidth_us))
            {
                /* we are in zone 4, the position will be set  for frame n+3 */
                safeWindowIndex++;
            }
        }

        /* set frame start delay relative to last EOF timestamp */
        frameStart_us = safeWindowIndex*ctx->FramePeriod_us + ctx->blanking.blkHV_us;

        /* set safe window opening delay (relative to current time)*/
        *safeWindowDelay_us = frameStart_us - unsafeWindowWidth_us - diff;

        /* set nextStartFrameDate_us */
        *nextStartFrameDate_us = ((uint64_t)lastFrameTimestamp.tv_sec)*1000000 + ((uint64_t)lastFrameTimestamp.tv_nsec/1000) + ((uint64_t)frameStart_us);

        /* rough check of the final value */
        if (*safeWindowDelay_us > 2*ctx->FramePeriod_us)
        {
            res = -1;
            HAL_MT9F002_PRINT("error, safeWindowDelay_us (%d) is too high, this should not happen (see dump below):\n",safeWindowIndex);
            HAL_MT9F002_PRINT("diff %d\n",diff);
            HAL_MT9F002_PRINT("safeWindowIndex %d\n",safeWindowIndex);
            HAL_MT9F002_PRINT("diffModulo %d\n",diffModulo);
            HAL_MT9F002_PRINT("ctx->blanking.blkHV_us %d\n",ctx->blanking.blkHV_us);
            HAL_MT9F002_PRINT("userDelay_us %d\n",userDelay_us);
            HAL_MT9F002_PRINT("frameStart_us %d\n",frameStart_us);
            HAL_MT9F002_PRINT("*safeWindowDelay_us %d\n",*safeWindowDelay_us);
        }
    }
    return res;
}

HAL_MT9F002_Set_Result_t HAL_MT9F002_SetPosition(HAL_MT9F002_context_t* ctx, HAL_MT9F002_pos_t* pos)
{
  // check argument
  CHECK_NULL_CTX(ctx);

  // check state
  CHECK_STATE(ctx,HAL_MT9F002_OPEN_STATE);

  HAL_MT9F002_Set_Result_t res = HAL_MT9F002_Set_Success;


  // check user position
  if ((pos->offsetX + ctx->sensorRes.width) > HAL_MT9F002_SENSOR_WIDTH)
  {
    HAL_MT9F002_PRINT("error, xOffset (%d) too high, not compatible with sensor width (%d)\n",pos->offsetX,ctx->sensorRes.width);
    res = HAL_MT9F002_Set_Failed;
  }
  if ((pos->offsetY + ctx->sensorRes.height) > HAL_MT9F002_SENSOR_HEIGHT)
  {
    HAL_MT9F002_PRINT("error, yOffset (%d) too high, not compatible with sensor height (%d)\n",pos->offsetY,ctx->sensorRes.height);
    res = HAL_MT9F002_Set_Failed;
  }

  // check x_addr_start is a multiple of x_skip_factor*8
  uint32_t xMultiple = 8*((ctx->blanking.x_odd_inc+1)/2);

  if (pos->offsetX % xMultiple)
  {
      HAL_MT9F002_PRINT("warning, xOffset (%d) not a multiple of %d\n",pos->offsetX,xMultiple);
      pos->offsetX -= (pos->offsetX % xMultiple);
      res = HAL_MT9F002_Set_Saturated;
  }

  if (res != HAL_MT9F002_Set_Failed )
  {
    if (pos->offsetX != ctx->pos.offsetX ||
        pos->offsetY != ctx->pos.offsetY)
    {
      // does the user use the SignalFrame() function ?
      if (ctx->lastFrameTimestamp.tv_sec==0 && ctx->lastFrameTimestamp.tv_nsec==0)
      {
        // nop, user probably don't want to know the position of acquisition window
        // therefore we don't care at what moment we set the new position

        // copy user position
        pthread_mutex_lock(&ctx->stateEstimatorMutex);
        {
          ctx->pos = *pos;
          HAL_MT9F002_setEstState(ctx,0);
        }
        pthread_mutex_unlock(&ctx->stateEstimatorMutex);
        // send new position to camera
        int writeRes;
        writeRes  = writeReg(&ctx->i2c, GROUPED_PARAMETER_HOLD,1);
        writeRes |= posWrite(ctx);
        writeRes |= writeReg(&ctx->i2c, GROUPED_PARAMETER_HOLD,0);
        if (writeRes)
        {
          HAL_MT9F002_PRINT("error, failed to write sensor position\n");
          res = HAL_MT9F002_Set_Failed;
        }
      }
      else
      {
        /*
         * The position must be set with care to be able to track the acquisition window
         * position.
         * The following scheme illustrates when it's safe to change the position and
         * when it will apply.
         *
         *
         * Safe window       |    *2*    |//|    *3*    |//|    *4*    |//|
         * Signal Frame             ->             ->             ->             ->
         * FV             ___|---1---|______|---2---|______|---3---|______|---4---|___
         *
         *
         * |//| represent the unsafe zone. It is SET_POSITION_DEADLINE_WINDOW long and ends
         * when a new frame starts.
         *
         * For example, if the user send a new position during Safe window *2*, the modification
         * will be effective for frame -2-.
         *
         * A displacement along the Y axis will generate a bad/masked frame that won't be output
         * by the sensor. Thus the Y displacement introduces at least one frame delay.
         *
         */

        /*
         * First identify if we are in a safe window and which one
         */
        /* get current time */
        struct timespec currentTime;
        clock_gettime(CLOCK_MONOTONIC,&currentTime);

        /* get last EOF time */
        struct timespec lastFrameTimestamp;
        pthread_mutex_lock(&ctx->stateEstimatorMutex);
        {
          lastFrameTimestamp=ctx->lastFrameTimestamp;
        }
        pthread_mutex_unlock(&ctx->stateEstimatorMutex);

        /* how much time elapsed since last frame has been received ? */
        uint32_t diff = diffTime(&currentTime,&lastFrameTimestamp);

        /* identify in which safe window we are */
        uint32_t safeWindowIndex=0;
        safeWindowIndex = diff/ctx->FramePeriod_us;
        /* we use a modulo to deal with the case of several trailing bad frame */
        uint32_t diffModulo = diff%ctx->FramePeriod_us;

        /*
         * Where are we exactly compared to last EOF ?
         *
         * case 1 (low framerate or small unsafe zone) :
         *
         * Safe window               | 1 |/2/|   3   |
         * Signal Frame             ->              ->
         * FV             ___|-------|_______|-------|
         *
         * case 2 (high framerate and/or large unsafe zone) :
         * Safe window             |////2////|  3  |////4////|
         * Signal Frame             ->              ->
         * FV             ___|-------|_______|-------|_______|-------|
         */

        uint32_t deadline=0;
        uint32_t delay=0;
        uint32_t zone=0;
        if((int)diffModulo < (int)(ctx->blanking.blkHV_us - SET_POSITION_WRITE_DURATION_US - SET_POSITION_DEADLINE_WINDOW_US))
        {
            /* we are in zone 1, it's safe */
            zone=1;

            /* set deadline for position write */
            deadline = safeWindowIndex*ctx->FramePeriod_us + ctx->blanking.blkHV_us - SET_POSITION_DEADLINE_WINDOW_US;
        }
        else if (diffModulo < (ctx->blanking.blkHV_us))
        {
            /* we are in zone2, it's not safe, wait for zone 3 */
            zone=2;
            delay = ctx->blanking.blkHV_us-diffModulo;
            usleep(delay);

            /* since we wait, the position will be set for the frame n+2 */
            safeWindowIndex++;

            /* set deadline for position write */
            deadline = safeWindowIndex*ctx->FramePeriod_us + ctx->blanking.blkHV_us - SET_POSITION_DEADLINE_WINDOW_US;
        }
        else if (diffModulo < (ctx->FramePeriod_us + ctx->blanking.blkHV_us - SET_POSITION_WRITE_DURATION_US - SET_POSITION_DEADLINE_WINDOW_US))
        {
            /* we are in zone 3, it's safe, but the position will be set for frame n+2 */
            zone = 3;
            safeWindowIndex++;

            /* set deadline for position write */
            deadline = safeWindowIndex*ctx->FramePeriod_us + ctx->blanking.blkHV_us - SET_POSITION_DEADLINE_WINDOW_US;
        }
        else
        {
            /*
             * we are in zone 4, it's not safe.
             * This should only happen in case 2.
             * wait for zone 3 in next frame.
             */
            zone = 4;
            delay = ctx->FramePeriod_us + ctx->blanking.blkHV_us - diffModulo;
            usleep(delay);

            /* since we wait for another frame, the position will be set for the frame n+3 */
            safeWindowIndex+=2;

            HAL_MT9F002_PRINT("warning, try to change position in an unsafe window (zone 4), wait %d µs\n",delay);

            /* set deadline for position write */
            deadline = safeWindowIndex*ctx->FramePeriod_us + ctx->blanking.blkHV_us - SET_POSITION_DEADLINE_WINDOW_US;
        }

        /*
         * Now we are in safe place to change position, do it
         */
        ctx->pos = *pos;
        if (posWrite(ctx))
        {
            res = HAL_MT9F002_Set_Failed;
        }


        /*
         * Take current date to be sure the position set occurs before the deadline
         */
        struct timespec setPosTime;
        clock_gettime(CLOCK_MONOTONIC,&setPosTime);
        diff = diffTime(&setPosTime,&currentTime);
        if (diff > deadline)
        {
            HAL_MT9F002_PRINT("error, write position was too long and may has taken place outside the safe window (%d µs after deadline)\n",diff-deadline);
            res = HAL_MT9F002_Set_Failed;
        }

        /*
         * Update state estimation
         */
        pthread_mutex_lock(&ctx->stateEstimatorMutex);
        {
            /* Since last time we checked in which part of the
             * safe window we where, an EOF might have been signaled.
             *
             * Example :
             *
             * Safe window       |     (1)     (2)   |//|
             * Signal Frame                ->          ->
             * FV             ___|----------|___________|----------|___________
             *
             * Imagine we where in (1) when we enter the function.
             * Then we set the position and now we are in position (2)
             * to update the stateEstimation buffer.
             * In (1) the position was for frame n+2.
             * But an EOF occurred (->), so the last position set is for frame n+1.
             *
             * We deal with this situation by comparing the lastFrameTimestamp in (1) and
             * the current lastFrameTimestamp.
             * If they are different, it means we are in the particular case described above.
             */
            if (diffTime(&ctx->lastFrameTimestamp,&lastFrameTimestamp) != 0)
            {
                /* An EOF occurred, update safeWindowIndex */
                safeWindowIndex--;
            }

            /*
             * update estimation buffer
             */
            HAL_MT9F002_setEstState(ctx,safeWindowIndex+1);
        }
        pthread_mutex_unlock(&ctx->stateEstimatorMutex);

        if (delay)
        {
            HAL_MT9F002_PRINT("warning, try to change position in an unsafe window (zone %d), wait %d µs\n",zone,delay);
        }
      }
    }
  }
  return res;
}

/*
 * @brief inform driver that a new frame has been received
 * @param ctx   camera driver instance
 * @return       0 success
 *              -1 failure
 * If you are interested by MT9F002_GetEstimatedPosition() this function
 * should be called right after an incomming frame before any other MT9F002
 * functions. Otherwise the position estimation could be wrong.
 *
 */
int HAL_MT9F002_SignalFrame(HAL_MT9F002_context_t* ctx)
{
  // check argument
  CHECK_NULL_CTX(ctx);

  // check state
  CHECK_STATE(ctx,HAL_MT9F002_OPEN_STATE);

  // timestamp incomming frame
  struct timespec lastFrameTimestamp;
  clock_gettime(CLOCK_MONOTONIC,&lastFrameTimestamp);

  pthread_mutex_lock(&ctx->stateEstimatorMutex);
  {
    ctx->lastFrameTimestamp = lastFrameTimestamp;

    // update position estimation
    HAL_MT9F002_popEstState(ctx);
  }
  pthread_mutex_unlock(&ctx->stateEstimatorMutex);



  return 0;
}

int HAL_MT9F002_GetEstimatedPosition(HAL_MT9F002_context_t* ctx, HAL_MT9F002_pos_t* pos, int idx)
{
  // check argument
  CHECK_NULL_CTX(ctx);

  // check state
  CHECK_STATE(ctx,HAL_MT9F002_OPEN_STATE);

  // a new frame has been received
  // copy last set position in estimated position
  pthread_mutex_lock(&ctx->stateEstimatorMutex);
  {
    HAL_private_MT9F002_stateEstimator_t* state_idx = HAL_MT9F002_getEstState(ctx,idx);
    *pos = state_idx->pos;
  }
  pthread_mutex_unlock(&ctx->stateEstimatorMutex);

  return 0;
}

int HAL_MT9F002_SetColorGain(HAL_MT9F002_context_t* ctx, HAL_MT9F002_colorGain_t* colorGain)
{
  // check argument
  CHECK_NULL_CTX(ctx);

  // check state
  CHECK_STATE(ctx,HAL_MT9F002_OPEN_STATE);

  ctx->colorGain = *colorGain;

  return gainsWrite(ctx);
}

int HAL_MT9F002_GetColorGain(HAL_MT9F002_context_t* ctx, HAL_MT9F002_colorGain_t* colorGain)
{
  // check argument
  CHECK_NULL_CTX(ctx);

  // check state
  CHECK_STATE(ctx,HAL_MT9F002_OPEN_STATE);

  // TODO not correct, gain is not necessarely the one setted by user, it can have been saturated
  *colorGain = ctx->colorGain;

  return 0;
}

void HAL_MT9F002_GetMinGain(float* gain) // in fact can be called at any time
{
  *gain =  MIN_GAIN;
}

HAL_MT9F002_Set_Result_t HAL_MT9F002_SetExposure(HAL_MT9F002_context_t* ctx, float time_ms)
{
  // check argument
  CHECK_NULL_CTX(ctx);

  // check state
  CHECK_STATE(ctx,HAL_MT9F002_OPEN_STATE);

  if (time_ms<0)
  {
    HAL_MT9F002_PRINT("error, exposure time (%f) canno't be negative\n",time_ms);
    return HAL_MT9F002_Set_Failed;
  }

  ctx->targetExposure_ms = time_ms;

  HAL_MT9F002_Set_Result_t res = HAL_MT9F002_Set_Success;
  if (exposureConf(ctx))
  {
    res = HAL_MT9F002_Set_Saturated;
  }

  int writeRes = 0;
  writeRes |= writeReg(&ctx->i2c, GROUPED_PARAMETER_HOLD,1);
  writeRes |= exposureWrite(ctx);
  writeRes |= writeReg(&ctx->i2c, GROUPED_PARAMETER_HOLD,0);

  if (writeRes)
  {
    res = HAL_MT9F002_Set_Failed;
  }
  return res;
}

int HAL_MT9F002_GetExposure(HAL_MT9F002_context_t* ctx, float* time_ms)
{
  // check argument
  CHECK_NULL_CTX(ctx);

  // check state
  CHECK_STATE(ctx,HAL_MT9F002_OPEN_STATE);

  *time_ms = ctx->realExposure_ms;
  return 0;
}

int HAL_MT9F002_GetMaxExposure(HAL_MT9F002_context_t* ctx, float* maxExposure)
{
  // check argument
  CHECK_NULL_CTX(ctx);

  // check state
  CHECK_STATE(ctx,HAL_MT9F002_OPEN_STATE);

  *maxExposure = ctx->maxExposure_ms;

  return -1;
}

/////////////////////////
// Close
/////////////////////////
/* stop stream and free context, it won't be possible to use the camera anymore with current context */
int HAL_MT9F002_Close(HAL_MT9F002_context_t* ctx)
{
  // check argument
  CHECK_NULL_CTX(ctx);

  // check state
  // TODO: in fact any state except UNINIT, think better ;)
  CHECK_STATE(ctx,HAL_MT9F002_OPEN_STATE);

  // TODO: do close
  HAL_MT9F002_PRINT("error, not yet implemented\n");

  //  HAL_MT9F002_StreamOn(ctx, HAL_MT9F002_Stream_off);

    ctx->state = HAL_MT9F002_INIT_STATE;

    // Close i2c
    close(ctx->i2c.fd);
    pthread_mutex_destroy(&ctx->stateEstimatorMutex);

    memset(ctx, 0, sizeof(HAL_MT9F002_context_t));

  return -1;
}
