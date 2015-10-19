
#ifndef _MT9F002_CAMERA_C_
#define _MT9F002_CAMERA_C_

#include <stdint.h>
#include <HAL_i2ctool.h>
#include <sys/time.h>
#include <pthread.h>
#include <stdio.h>

#define HAL_MT9F002_SENSOR_WIDTH  4640
#define HAL_MT9F002_SENSOR_HEIGHT 3320

#define HAL_MT9F002_STATE_ESTIMATOR_SIZE 8 // private

typedef enum
{
  HAL_MT9F002_Set_Saturated=-2, /* value has been set but was saturated (unreachable value) */
  HAL_MT9F002_Set_Failed=-1,    /* value has not been set (communication error, or forbidden value)*/
  HAL_MT9F002_Set_Success=0,    /* value has been set*/
}HAL_MT9F002_Set_Result_t;

typedef enum
{
  HAL_MT9F002_UNINIT_STATE, /* Uninitialized state */
  HAL_MT9F002_INIT_STATE,   /* camera context initialized */
  HAL_MT9F002_OPEN_STATE,   /* camera has been opened */
  HAL_MT9F002_CLOSE_STATE,  /* camera is closed */
  HAL_MT9F002_FATAL_ERROR,  /* an error occurred, context is dead */

}HAL_MT9F002_state_t;

typedef enum
{
  HAL_MT9F002_Parallel,
  HAL_MT9F002_MIPI,
  HAL_MT9F002_HiSPi,
}HAL_MT9F002_interface_t;

typedef enum
{
  HAL_MT9F002_8bits = 8,
  HAL_MT9F002_10bits = 10,
  HAL_MT9F002_12bits = 12,
}HAL_MT9F002_pixelDepth_t;

typedef enum
{
  HAL_MT9F002_1lane = 1,
  HAL_MT9F002_2lane = 2,
  HAL_MT9F002_3lane = 3,
  HAL_MT9F002_4lane = 4,
}HAL_MT9F002_Serial_lanes_t;

typedef enum
{
  HAL_MT9F002_ColorBar_off = 0,
  HAL_MT9F002_ColorBar_on = 1,
} HAL_MT9F002_Colorbar_t;

typedef enum
{
    HAL_MT9F002_PosSafeWindow_any = 0,           /* no constraint */
    HAL_MT9F002_PosSafeWindow_FrameOnly = 1,     /* HAL_MT9F002_getNextSafeWindow() will return a safe window to set position
                                                    located inside a frame */
    /* HAL_MT9F002_PosSafeWindow_BlkOnly = 2,*/  /* not yet supported
                                                    HAL_MT9F002_getNextSafeWindow() will return a safe window to set position
                                                    located between 2 frames (blanking area) */

} HAL_MT9F002_PosSafeWindow_t;

typedef enum
{
  HAL_MT9F002_Stream_off_with_PxlClock = -1, // no frame
  HAL_MT9F002_Stream_off = 0,
  HAL_MT9F002_Stream_on = 1,
} HAL_MT9F002_StreamOn_t;

typedef struct HAL_MT9F002_colorGain_t_
{
  float greenR;
  float blue;
  float red;
  float greenB;
} HAL_MT9F002_colorGain_t;

typedef struct HAL_MT9F002_res_t_
{
  uint32_t width;
  uint32_t height;
} HAL_MT9F002_res_t;

typedef struct HAL_MT9F002_pos_t_
{
  uint32_t offsetX;
  uint32_t offsetY;
} HAL_MT9F002_pos_t;

typedef struct HAL_MT9F002_timing_t_
{
  float lineAcqDuration_s;   /* duration (in second) to acquire and transmit one line */
  float frameAcqDuration_s;  /* duration (in second) to acquire and transmit one frame */
  float framePeriod_s;       /* duration (in second) between 2 frames  start when no y-axis movement occurs */
  float frameBlanking_s;     /* blanking duration (in second) between 2 frames when no y-axis movement occurs */
} HAL_MT9F002_timing_t;

typedef struct HAL_private_MT9F002_pll_t_
{
  uint32_t pre_pll_clk_div;
  uint32_t pll_multiplier;
  uint32_t vt_sys_clk_div;
  uint32_t vt_pix_clk_div;
  uint32_t rowSpeed_2_0;
  uint32_t op_sys_clk_div;
  uint32_t row_speed_10_8;
  uint32_t op_pix_clk_div;
  uint32_t shift_vt_pix_clk_div;
} HAL_private_MT9F002_pll_t;

typedef struct HAL_private_MT9F002_blanking_t_
{
  // register involved in blanking computation
  uint32_t min_line_blanking_pck;
  uint32_t x_odd_inc;
  uint32_t min_frame_blanking_lines;
  uint32_t min_line_length_pck;

  // computed value determining mas fps
  uint32_t minimum_line_length;
  uint32_t minimum_frame_length_lines;

  // horizontal& vertical blanking results
  uint32_t line_length;
  float line_length_ms;
  uint32_t frame_length;

  // HV blanking length in µs
  uint32_t blkHV_us;


} HAL_private_MT9F002_blanking_t;

typedef struct HAL_private_MT9F002_clock_t_
{
  float vt_pix_clk;
  float op_pix_clk;
}HAL_private_MT9F002_clock_t;

typedef struct HAL_private_MT9F002_exposure_t_
{
  // register involved in exposure computation
  uint32_t fine_integration_time_min;
  uint32_t fine_integration_time_max_margin;
  uint32_t coarse_integration_time_min;
  uint32_t coarse_integration_time_max_margin;

  // exposure results
  uint32_t coarse_integration_time;
  uint32_t fine_integration_time;
} HAL_private_MT9F002_exposure_t;

/*
 * A private structure to track sensor state
 */
typedef struct HAL_private_MT9F002_stateEstimator_t_
{
  /* position estimation */
  HAL_MT9F002_pos_t pos;
  /* exposition */
  /* gain */

  /* private data use to manage the stack */
  uint8_t set;      // this state has been set
  uint8_t badFrame; // this state will generate a bad frame
} HAL_private_MT9F002_stateEstimator_t;

typedef struct HAL_MT9F002_context_t_
{
  // mandatory param
  HAL_MT9F002_interface_t interface;        /* interface type */
  HAL_i2ctoolContext_t i2c;                 /* i2c communication */
  float inputClkFreq;                       /* sensor input clock in MHz */
  float targetOutputClkFreq;                /* desired sensor output clock in MHz */
  float outputClkFreq;                      /* real sensor output clock in MHz */
  HAL_MT9F002_Serial_lanes_t numberOfLanes; /* for MIPI/HiSPi only, number of lanes used for transmission */

  // optional param, preset by MTPF002_Open()
  HAL_MT9F002_res_t res;                    /* user resolution */
  HAL_MT9F002_res_t sensorRes;              /* sensor resolution */
  float scaler;                             /* ratio between sensor resolution and ouput resolution */
  HAL_MT9F002_pixelDepth_t pixelDepth;      /* number of bits per pixel */
  HAL_MT9F002_colorGain_t colorGain;        /* color gain */
  float targetFps;                          /* user desired frame rate */
  float realFps;                            /* real framerate */
  uint32_t FramePeriod_us;                  /* frame period in µs */
  float maxFps;                             /* maximum reachable framerate */

  // runtime param
  HAL_MT9F002_Colorbar_t colorBar;          /* colorbar mode enabler */
  float targetExposure_ms;                  /* user exposure time in ms */
  float realExposure_ms;                    /* real exposure time in ms */
  float maxExposure_ms;                     /* maximum exposure time in ms */
  HAL_MT9F002_pos_t pos;                    /* coordinate of the top left pixel on the sensor (last written one)*/

  HAL_private_MT9F002_stateEstimator_t stateEstimator[HAL_MT9F002_STATE_ESTIMATOR_SIZE]; /* list of estimated state (ring buffer)*/
  uint32_t stateEstimatorI;                 /* stateEstimator index  */
  pthread_mutex_t stateEstimatorMutex;      /* mutex to protect SignalFrame() SetPosition() getEstimatedPosition() */

  HAL_MT9F002_StreamOn_t streamOn;          /* stream state */

  // driver state
  HAL_MT9F002_state_t state;                /* driver state */

  // other
  HAL_private_MT9F002_pll_t pll;
  HAL_private_MT9F002_blanking_t blanking;
  HAL_private_MT9F002_clock_t clock;
  HAL_private_MT9F002_exposure_t exposure;
  struct timespec lastFrameTimestamp;    /* last frame timestamp, used for position estimation */
}HAL_MT9F002_context_t;

/////////////////////////
// INIT
/////////////////////////

/*
 * Prepare a camera (open i2c, initialize camera context) (mandatory)
 */
int HAL_MT9F002_Init(HAL_MT9F002_context_t* ctx, char* i2cPath, float  inputClkFreq, float outputClkFreq, HAL_MT9F002_interface_t interface);

/////////////////////////
// CONGFIGURATION
/////////////////////////

/* after HAL_MTPF002_Init() following functions can be optionaly called before HAL_MT9F002_Open() */
int HAL_MT9F002_SetPixelDepth(HAL_MT9F002_context_t* ctx, HAL_MT9F002_pixelDepth_t pixelDepth);

/*
 * @brief Set resolution and scale factor
 * @param ctx    camera driver instance
 * @param res    stream resolution
 * @param scaler scale factor
 * @return       0 success
 *              -1 failure
 *
 * The acquired frame dimensions are [width/scaler x height/scaler]. The frame is scaled by the sensor so
 * that ouput frame dimensions are [width x height]. Note that the scaler provided by user may be modified
 * to fit sensor requirements. HAL_MT9F002_GetResolution() will tell what scaler value has been choosen.
 * The user output resolution is not modified.
 *
 */
int HAL_MT9F002_SetResolution(HAL_MT9F002_context_t* ctx, HAL_MT9F002_res_t* res, float scaler);
/*
 * @brief Get resolution and scale factor
 * @param ctx    camera driver instance
 * @param res    stream resolution is returned here (can be NULL)
 * @param scaler scale factor is returned here (can be NULL)
 * @return       0 success
 *              -1 failure
 *
 * Return resolution and scaler used to configure the sensor
 */
int HAL_MT9F002_GetResolution(HAL_MT9F002_context_t* ctx, HAL_MT9F002_res_t* res, float* scaler);
int HAL_MT9F002_SetNumberOfLanes(HAL_MT9F002_context_t* ctx, HAL_MT9F002_Serial_lanes_t numberOfLanes);

/* next function flush config, after it has been call, configuration function are no more available */
int HAL_MT9F002_Open(HAL_MT9F002_context_t* ctx);


/////////////////////////
// RUNTIME
/////////////////////////
/* next function are available after HAL_MT9F002_Open() and can be call at any time */
HAL_MT9F002_Set_Result_t HAL_MT9F002_SetFrameRate(HAL_MT9F002_context_t* ctx, float fps);

int HAL_MT9F002_GetFrameRate(HAL_MT9F002_context_t* ctx, float* fps);

/*
 * @brief Get timing information about capture
 * @param[in]  ctx      camera context
 * @param[out] timing   timing structure
 * @return       0 success
 *              -1 failure
 *
 * Timings can be modified by HAL_MT9F002_SetFrameRate(), HAL_MT9F002_SetResolution(), ...
 *
 */
int HAL_MT9F002_GetTiming(HAL_MT9F002_context_t* ctx, HAL_MT9F002_timing_t* timing);

int HAL_MT9F002_GetMaxFrameRate(HAL_MT9F002_context_t* ctx, float* fps);

int HAL_MT9F002_StreamOn(HAL_MT9F002_context_t* ctx, HAL_MT9F002_StreamOn_t on);
int HAL_MT9F002_SetColorBar(HAL_MT9F002_context_t* ctx, HAL_MT9F002_Colorbar_t colorBar);

/*
 * @brief move acquisition window
 * @param ctx   camera driver instance
 * @param pos   new position
 * @return      HAL_MT9F002_Set_Saturated, position has been modified before set
 *              HAL_MT9F002_Set_Success, position set successfully
 *              HAL_MT9F002_Set_Failed, position has not been set
 * The new position is not necessarily applied immediately.
 * User position can be modified if it doesn't meet the sensor requirements. In that
 * case the function returns HAL_MT9F002_Set_Saturated pos is modified so that user can
 * know what position has been set.
 * This function can be blocking when HAL_MT9F002_SignalFrame() is used.
 */
HAL_MT9F002_Set_Result_t HAL_MT9F002_SetPosition(HAL_MT9F002_context_t* ctx, HAL_MT9F002_pos_t* pos);
int HAL_mt9f002_check(int verbose, FILE* FD);

/*
 * @brief inform driver that a new frame has been received
 * @param ctx   camera driver instance
 * @return       0 success
 *              -1 failure
 * If you are interested by HAL_MT9F002_GetEstimatedPosition() this function
 * should be called right after an incomming frame before any other HAL_MT9F002
 * functions. Otherwise the postion estimation could be wrong.
 *
 */
int HAL_MT9F002_SignalFrame(HAL_MT9F002_context_t* ctx);

/*
 * @brief return window position of the last received frame
 * @param ctx   camera driver instance
 * @param pos   the resulting position is write here
 * @param idx   idx=0 get position of last received frame, idx=1 position for next frame, ...
 * @return       0 success
 *              -1 failure
 * /!\ See MT9F002_SetPosition() and MT9F002_SignalFrame() usage.
 * This function works only when MT9F002_SignalFrame() is used
 * properly. Otherwise behaviour is undetermined.
 *
 */
int HAL_MT9F002_GetEstimatedPosition(HAL_MT9F002_context_t* ctx, HAL_MT9F002_pos_t* pos, int idx);

/*
 * @brief Get the next safe window to change position. The computed
 * safe window is delayed to be as close as possible to the next frame start.
 *
 * @param[in]  ctx                      camera context
 * @param[in]  userDelay_us             the safe window will be at least userDelay_us large
 * @param[in]  safeWindowConstraint     user can specify where the safe window must be located (during blanking or frame transmission)
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
                                  uint64_t* nextStartFrameDate_us);

int HAL_MT9F002_SetColorGain(HAL_MT9F002_context_t* ctx, HAL_MT9F002_colorGain_t* colorGain);
int HAL_MT9F002_GetColorGain(HAL_MT9F002_context_t* ctx, HAL_MT9F002_colorGain_t* colorGain);
void HAL_MT9F002_GetMinGain(float* gain); // in fact can be called at any time

HAL_MT9F002_Set_Result_t HAL_MT9F002_SetExposure(HAL_MT9F002_context_t* ctx, float time_ms);
int HAL_MT9F002_GetExposure(HAL_MT9F002_context_t* ctx, float* time_ms);
int HAL_MT9F002_GetMaxExposure(HAL_MT9F002_context_t* ctx, float* maxExposure);

/////////////////////////
// Close
/////////////////////////
/* stop stream and free context, it won't be possible to use the camera anymore with current context */
int HAL_MT9F002_Close(HAL_MT9F002_context_t* ctx);

#endif
