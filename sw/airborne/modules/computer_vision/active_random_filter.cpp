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

#include "active_random_filter.h"
#include <vector>
#include <ctime>

#define BOARD_CONFIG "boards/bebop.h"       // TODO: WHYYYY?

extern "C" {
    #include "boards/bebop.h"                       // C header used for bebop specific settings
    #include <state.h>                              // C header used for state functions and data
    #include <sys/time.h>
    #include "mcu_periph/sys_time.h"
}

using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
using namespace cv;


#define PRINT(string,...) fprintf(stderr, "[AR-FILTER->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

#define AR_FILTER_VERBOSE FALSE
#if AR_FILTER_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

#define xSign(x) ( ( x ) >= ( 0 ) ? ( 1 ) : ( -1 ) )

extern struct mt9f002_t mt9f002;

#define AR_FILTER_ISP_CROP      1   // Use the ISP to crop the frame according to FOV-Y
#define AR_FILTER_SHOW_REJECT   0   // Print why shapes are rejected
#define AR_FILTER_MOD_VIDEO     1   // Modify the frame to show relevant info
#define AR_FILTER_CROSSHAIR     1   // Show centre of frame with crosshair
#define AR_FILTER_MARK_CONTOURS 1   // Mark all contour pixels green on sourceframe
#define AR_FILTER_DRAW_CIRCLES  0   // Draw circles
#define AR_FILTER_DRAW_BOXES 	1   // Draw boxes
#define AR_FILTER_SHOW_MEM      1   // Print object locations to terminal
#define AR_FILTER_SAVE_FRAME    0   // Save a frame for post-processing
#define AR_FILTER_MEASURE_FPS   1   // Measure average FPS
#define AR_FILTER_CALIBRATE_CAM 0   // Calibrate camera
#define AR_FILTER_WORLDPOS 		1   // Use world coordinates
#define AR_FILTER_NOYAW 		1   // Output in body horizontal XY
#define AR_FILTER_TIMEOUT       150 // Frames from start
#define AR_FILTER_USE_ALTITUDE  0   // Use own altitude for world pos

static void             active_random_filter_header ( Mat& sourceFrame );
static void             active_random_filter_footer ( void );
static void 			trackObjects	    ( Mat& sourceFrame, Mat& greyFrame );
static void             identifyObject      ( trackResults* trackRes );
static bool 			addContour			( vector<Point> contour, uint16_t offsetX, uint16_t offsetY, double minDist = 0.0, double maxDist = 0.0);
static void 			cam2body 			( trackResults* trackRes );
static void 			body2world 			( trackResults* trackRes );

static double 			correctRadius		( double r, double f, double k );
static double           invertRadius        ( double r, double f, double k );
static void             inputCoord          ( double x_out, double y_out, double *x_in, double *y_in );
static void             outputCoord         ( double x_in, double y_in, double *x_out, double *y_out );
static void             getMVP              ( double MVP[16] );
static Rect 			setISPvars 			( uint16_t width, uint16_t height );
static uint16_t         horizonPos          ( double y_orig );
static void             plotHorizon         (Mat& sourceFrameCrop);
static void             inversePoint        (double x_out, double y_out, double *x_in, double *y_in);
static void             angles2point        (double xAngle, double yAngle, double *x_out, double * y_out);
static void             correctPoint        (double x_in, double y_in, double *x_out, double *y_out);
static void 	        estimatePosition	( uint16_t xp, uint16_t yp, uint32_t area, double position[3]);
static void             point2angles        (double x_out, double y_out, double *xAngle, double *yAngle);
static bool 			getNewPosition		( uint8_t nextDir, uint16_t* newRow, uint16_t* newCol, int* maxRow, int* maxCol );
static void             eraseMemory         ( void );
static void             getYUVColours       ( Mat& sourceFrame, uint16_t row, uint16_t col, uint8_t* Y, uint8_t* U, uint8_t* V );
static void             createSearchGrid    ( uint16_t x_p, uint16_t y_p, Point searchGrid[], uint8_t searchLayer, uint16_t sGridSize, int* maxRow, int* maxCol);
// Flood CW declarations
static bool             processImage_cw     ( Mat& sourceFrame, Mat& destFrame, uint16_t sampleSize );
static int              pixFindContour_cw   ( Mat& sourceFrame, Mat& destFrame, uint16_t row, uint16_t col, uint8_t prevDir, bool cascade );
static int              pixFollowContour_cw ( Mat& sourceFrame, Mat& destFrame, uint16_t row, uint16_t col, uint8_t prevDir, bool cascade );
static void             getNextDirection_cw ( uint8_t prevDir, uint8_t* nextDir, uint8_t* nextDirCnt );
static bool             objCont_add         ( double minDist = 0.0, double maxDist = 0.0);
static void             objCont_addPoint    ( uint16_t* row, uint16_t* col );
static Moments          objCont_moments     ( void );
// Flood omni declarations
static bool             processImage_omni   ( Mat& sourceFrame, Mat& destFrame, uint16_t sampleSize );
static int              pixFindContour_omni ( Mat& sourceFrame, Mat& destFrame, uint16_t row, uint16_t col, uint8_t prevDir, bool cascade );
static void             getNextDirection_omni( uint8_t prevDir, uint8_t* nextDir, uint8_t* nextDirCnt );
static void             processCrops        ( Mat& frameGrey);
static void             addCrop             ( void );
static Rect             enlargeRectangle    ( Mat& sourceFrame, Rect rectangle, double scale );
static bool             inRectangle         ( Point pt, Rect rectangle );
// Set up trackRes
static uint8_t          trackRes_size = 0;
static bool             trackRes_findMax    ( void );
static bool             trackRes_add        ( trackResults newRes, uint8_t overwriteId = trackRes_size);
static bool             trackRes_clear      ( void );
// Set up neighbourMem
uint8_t                 neighbourMem_size = 0;
static bool             neighbourMem_findMax( void );
static bool             neighbourMem_add    ( memoryBlock newRes, uint8_t overwriteId = neighbourMem_size);
// Stabilization functions
static void             setPerspectiveMat   (double m[16]);
static void             setRotationMat      (float, float, float, double m[16]);
static void             setIdentityMatrix   (double m[16]);
static void             matrixMultiply      (double m1[16], double m2[16], double result[16]);
static void             matvecMultiply      (double m1[16], double v1[4], double result[4]);
static void             view_set_lookat     (double result[16], double eye[4], double center[4], double up[4]);
static float            vector_length       (const float x, const float y, const float z);
static void             translate_xyz       (double result[16], const float translatex, const float translatey, const float translatez);
static void             rotateVector        (float x, float y, float z, double vector[4]);
static void             setTranslationMat   (float x, float y, float z, double outputMat[16]);

#if AR_FILTER_MOD_VIDEO
static void             mod_video           (Mat& sourceFrame, Mat& frameGrey);
#endif
#if AR_FILTER_CALIBRATE_CAM
static void 			calibrateEstimation (void);
#endif
#if AR_FILTER_SAVE_FRAME
static void 			saveBuffer			(Mat sourceFrame, const char *filename);
#endif

#if AR_FILTER_MEASURE_FPS
static uint32_t curT;
#endif

// Set up tracking parameters
#define     AR_FILTER_MAX_OBJCONT_SIZE  10000
#define     AR_FILTER_OBJ_X_OFFSET      0.0             // Offset x from object centre to object c.g. in world frame
#define     AR_FILTER_OBJ_Y_OFFSET      0.0             // Offset y from object centre to object c.g. in world frame
#define     AR_FILTER_OBJ_Z_OFFSET      0.1             // Offset z from object centre to object c.g. in world frame
uint8_t     AR_FILTER_FLOOD_STYLE       = AR_FILTER_FLOOD_CW;
uint8_t     AR_FILTER_SAMPLE_STYLE      = AR_FILTER_STYLE_RANDOM;

double      AR_FILTER_CAM_RANGE         = 5.0;          // Maximum r_c of newly added objects
uint16_t    AR_FILTER_RND_PIX_SAMPLE    = 2500;         // Random pixel sample size
uint16_t    AR_FILTER_MAX_LAYERS        = 5000;         // Maximum recursive depth of CW flood
double 	    AR_FILTER_MAX_CIRCLE_DEF 	= 0.815;         // Max contour eccentricity
double      AR_FILTER_MIN_CIRCLE_PERC   = 0.50;         // Minimum percentage of circle in view
// Automatically calculated parameters
uint16_t    AR_FILTER_MIN_POINTS;           // Mimimum contour length
double      AR_FILTER_MIN_CIRCLE_SIZE;      // Minimum contour area
uint16_t    AR_FILTER_MIN_LAYERS;           // Miminum recursive depth of CW flood

uint16_t    AR_FILTER_MIN_CROP_AREA     = 100;          // Minimal area of a crop rectangle
uint8_t     AR_FILTER_CDIST_YTHRES      = 2;
uint8_t     AR_FILTER_CDIST_UTHRES      = 0;
uint8_t     AR_FILTER_CDIST_VTHRES      = 0;
// 1.525 too much - 1.5 (just) too little
double      AR_FILTER_VIEW_R = 0.0091;
double      default_k                   = 1.22471; //1.118 based on horizon - 1.22425040841 max                                                      // Fisheye correction factor (1.085)
uint16_t    default_calArea             = 8600;  // Calibrate at full resolution (5330)
double      default_orbDiag             = 2 * CFG_MT9F002_FISHEYE_RADIUS;  // Measured circular image diagonal using full resolution
double      scale_f                     = 1.0;
// Lens correction parameter k
float near          = 0.0000000075;
float far           = 2.5;
float angleOfView   = 178.87; // 160 degrees

uint8_t     AR_FILTER_GREY_THRES        = 15;
/* FAKE LIGHT
uint8_t     AR_FILTER_Y_MIN             = 0;                            // 0  [0,65 84,135 170,255]zoo 45
uint8_t     AR_FILTER_Y_MAX             = 255;                          // 255
uint8_t     AR_FILTER_U_MIN             = 0;                          // 84
uint8_t     AR_FILTER_U_MAX             = 255;                          // 113
uint8_t     AR_FILTER_V_MIN             = 158;                          // 218 -> 150?
uint8_t     AR_FILTER_V_MAX             = 255;                          // 240 -> 255?
*/

/* DAYLIGHT */
uint8_t 	AR_FILTER_Y_MIN 			= 150;                           // 0  [0,65 84,135 170,255]zoo 45
uint8_t 	AR_FILTER_Y_MAX 			= 255;                          // 255
uint8_t 	AR_FILTER_U_MIN 			= 95;                          // 84
uint8_t 	AR_FILTER_U_MAX 			= 131;                          // 113
uint8_t 	AR_FILTER_V_MIN 			= 145;                          // 218 -> 150?
uint8_t 	AR_FILTER_V_MAX 			= 188;                          // 240 -> 255?

/* DAYLIGHT 2
uint8_t     AR_FILTER_Y_MIN             = 123;                           // 0  [0,65 84,135 170,255]zoo 45
uint8_t     AR_FILTER_Y_MAX             = 222;                          // 255
uint8_t     AR_FILTER_U_MIN             = 109;                          // 84
uint8_t     AR_FILTER_U_MAX             = 130;                          // 113
uint8_t     AR_FILTER_V_MIN             = 150;                          // 218 -> 150?
uint8_t     AR_FILTER_V_MAX             = 209;                          // 240 -> 255?
*/

/* Cyberzoo
uint8_t     AR_FILTER_Y_MIN             = 70;                           // 0  [0,65 84,135 170,255]zoo 45
uint8_t     AR_FILTER_Y_MAX             = 230;                          // 255
uint8_t     AR_FILTER_U_MIN             = 105;                          // 84
uint8_t     AR_FILTER_U_MAX             = 155;                          // 113
uint8_t     AR_FILTER_V_MIN             = 176;                          // 218 -> 150?
uint8_t     AR_FILTER_V_MAX             = 210;                          // 240 -> 255?
*/
double 	    AR_FILTER_IMAGE_CROP_FOVY 	= 60 * M_PI / 180.0; 		    // Radians
double 	    AR_FILTER_CROP_X 			= 1.2;
uint8_t     AR_FILTER_MEMORY 			= 40;
double      AR_FILTER_FPS               = 15.0;
double      AR_FILTER_VMAX              = 3.5;

// Initialize parameters to be assigned during runtime
static uint16_t 	        pixCount            = 0;
static uint16_t 	        pixSucCount         = 0;
static uint16_t             pixDupCount         = 0;
static uint16_t             pixSrcCount         = 0;
static uint16_t             pixNofCount         = 0;
static uint16_t 	        layerDepth          = 0;
static uint16_t 	        sample              = 0;
static uint16_t 	        runCount            = 0;
static uint16_t 	        maxId 		        = 0;
static uint8_t              trackRes_maxId      = 0;
static double               trackRes_maxVal     = 0;
static uint8_t              trackRes_lastId     = 0;
static uint8_t              neighbourMem_maxId  = 0;
static double               neighbourMem_maxVal = 0;
static uint8_t              neighbourMem_lastId = 0;
static uint16_t             ispWidth            = MT9F002_OUTPUT_HEIGHT;
static uint16_t             ispHeight           = MT9F002_OUTPUT_WIDTH;
static uint16_t             initialWidth;
static uint16_t             initialHeight;
static uint16_t             cropCol;
static double               ispScalar;
static Rect 			    objCrop;
static vector<Rect> 	    cropAreas;
static struct FloatEulers*  eulerAngles;
static trackResults         trackRes[AR_FILTER_MAX_OBJECTS];
memoryBlock                 neighbourMem[AR_FILTER_MAX_OBJECTS];

#if AR_FILTER_MEASURE_FPS
    static struct timespec time_now;
    static struct timespec time_prev;
    static struct timespec time_init;
#endif

// Flood CW parameters
static Point            objCont_store[AR_FILTER_MAX_OBJCONT_SIZE];
static uint16_t         objCont_size    = 0;
static uint16_t         objCont_sCol    = 0;
static uint16_t         objCont_sRow    = 0;
static uint8_t          cmpY            = 0;
static uint8_t          cmpU            = 0;
static uint8_t          cmpV            = 0;

static double MVP[16];

void active_random_filter_init(void){
    ispScalar                   = mt9f002.output_scaler * 2.0/((double) mt9f002.y_odd_inc + 1.0);
    ispHeight                   = round((CFG_MT9F002_X_ADDR_MAX - CFG_MT9F002_X_ADDR_MIN) * ispScalar);
    ispWidth                    = round((CFG_MT9F002_Y_ADDR_MAX - CFG_MT9F002_Y_ADDR_MIN) * ispScalar);
    initialWidth                = mt9f002.output_width;
    initialHeight               = mt9f002.output_height;
#if AR_FILTER_MEASURE_FPS
    clock_gettime(CLOCK_MONOTONIC, &time_prev);
    time_init = time_prev;
#endif
}

void active_random_filter(char* buff, uint16_t width, uint16_t height, struct FloatEulers* curEulerAngles){
    eulerAngles = curEulerAngles;
    Mat sourceFrame (height, width, CV_8UC2, buff);                 // Initialize current frame in openCV (UYVY) 2 channel
    Mat frameGrey   (height, width, CV_8UC1, cvScalar(0.0));        // Initialize an empty 1 channel frame
    active_random_filter_header(sourceFrame);                       // Mostly printing and storing
    Rect crop 	        = setISPvars( width, height); 	            // Calculate ISP related parameters
    if(runCount < AR_FILTER_TIMEOUT) {
        runCount++;
        PRINT("Timeout %d\n", AR_FILTER_TIMEOUT - runCount);
        return;
    }
    Mat sourceFrameCrop = sourceFrame(crop); 				                // Crop the frame
	trackObjects(sourceFrameCrop, frameGrey);                       // Track objects in sourceFrame
	eraseMemory();
	uint8_t r;
	for(r=0; r < trackRes_size; r++){                             // Convert angles & Write/Print output
		cam2body(&trackRes[r]);						                // Convert from camera angles to body angles (correct for roll)
		body2world(&trackRes[r]); 		                            // Convert from body angles to world coordinates (correct yaw and pitch)
		identifyObject(&trackRes[r]);                               // Identify the spotted neighbours
	}
#if AR_FILTER_MOD_VIDEO
	mod_video(sourceFrameCrop, frameGrey);                              // Modify the sourceframesourceFrame.cols-1
#endif
#if AR_FILTER_CROSSHAIR
#if AR_FILTER_ISP_CROP
	//double halfPoint = round((CFG_MT9F002_X_ADDR_MAX - CFG_MT9F002_X_ADDR_MIN) * 0.5 * ispScalar);
	//if(halfPoint > cropCol && halfPoint < cropCol + sourceFrameCrop.cols){
	//    line(sourceFrameCrop, Point(halfPoint - cropCol, 0), Point(halfPoint - cropCol, sourceFrame.rows-1), Scalar(255,127), 2);
	//}
	/*
	line(sourceFrame, Point(crop.x, sourceFrame.rows / 2 + 0.25 * ispHeight * (tan(eulerAngles->phi))), Point(crop.x + crop.width, sourceFrame.rows / 2 - 0.25 * ispHeight * (tan(eulerAngles->phi))), Scalar(0,127), 1);
	line(sourceFrame, Point(sourceFrame.cols / 2 - 0.25 * ispHeight * (tan(eulerAngles->phi)), 0), Point(sourceFrame.cols / 2 + 0.25 * ispWidth * (tan(eulerAngles->phi)), sourceFrame.rows-1), Scalar(0,127), 1);
*/
	//circle(sourceFrame,Point(ispHeight/2 - cropCol + crop.x, ispWidth/2), CFG_MT9F002_FISHEYE_RADIUS * ispScalar, cvScalar(0,255), 1);
	plotHorizon(sourceFrameCrop);
#else
	double halfPoint = round((CFG_MT9F002_X_ADDR_MAX - CFG_MT9F002_X_ADDR_MIN) * 0.5 * ispScalar);
	double horizonPos = reversePixel(-eulerAngles->theta - MT9F002_THETA_OFFSET);
	line(sourceFrame, Point(halfPoint - 0.25 * ispWidth * (tan(eulerAngles->phi)), 0), Point(halfPoint + 0.25 * ispWidth * (tan(eulerAngles->phi)), sourceFrame.rows-1), Scalar(0,127), 1);
	//double horizonPos = reversePixel(-eulerAngles->theta - MT9F002_THETA_OFFSET);
	line(sourceFrame, Point(crop.x, sourceFrame.rows / 2 + 0.25 * ispHeight * (tan(eulerAngles->phi))), Point(crop.x + crop.width, sourceFrame.rows / 2 - 0.25 * ispHeight * (tan(eulerAngles->phi))), Scalar(0,0), 1);
	line(sourceFrame, Point(horizonPos - 0.25 * ispWidth * (tan(eulerAngles->phi)),0), Point(horizonPos + 0.25 * ispWidth * (tan(eulerAngles->phi)), sourceFrame.rows-1), Scalar(0,0), 1);
#endif
#endif // AR_FILTER_MOD_VIDEO
	frameGrey.release(); 			                                // Release Mat
	sourceFrameCrop.release();
	sourceFrame.release();                                          // Release Mat
	active_random_filter_footer();
	return;
}

Rect setISPvars( uint16_t width, uint16_t height){
    // This function computes the cropping according to the desires FOV Y and the current euler angles
    getMVP(MVP);
    AR_FILTER_MIN_CIRCLE_SIZE   = pow(sqrt((double) default_calArea  * pow(ispScalar,2.0)) / AR_FILTER_CAM_RANGE, 2.0);
    AR_FILTER_MIN_LAYERS        = (uint16_t) round(AR_FILTER_MIN_CIRCLE_PERC * 2 * M_PI * sqrt((double) default_calArea  * pow(ispScalar,2.0)/ M_PI) / AR_FILTER_CAM_RANGE);
    AR_FILTER_MIN_POINTS        = (uint16_t) round(0.25 * AR_FILTER_MIN_LAYERS);
    double f, x1, y1, x1p, y1p, r;
    double left[2], center[2], right[2];
    /* Do some simple unit tests
    double x1_o, y1_o, r_o, corR;
    f       = scale_f * default_orbDiag * ispScalar / (4 * sin(M_PI / 4));
    x1_o    = ((float) 2 * rand()) / RAND_MAX - 1;
    y1_o    = ((float) 2 * rand()) / RAND_MAX - 1;
    outputCoord(x1_o, y1_o, &x1p, &y1p);
    inputCoord(x1p, y1p, &x1, &y1);
    r_o     = sqrt(pow(x1_o, 2.0) + pow(y1_o, 2.0)) * CFG_MT9F002_FISHEYE_RADIUS * ispScalar;
    corR    = correctRadius(r_o, f, default_k);
    r       = invertRadius(corR, f, default_k);
    PRINT("in(%0.2f, %0.2f) -> out(%0.2f, %0.2f) -> in(%0.2f, %0.2f)\n", x1_o, y1_o, x1p, y1p, x1, y1);
    PRINT("r(%0.2f) -> corR(%0.2f) -> r(%0.2f)\n", r_o, corR, r);
    double xAngle, yAngle, xAngle_o, yAngle_o;
    xAngle = (((float) 2 * rand()) / RAND_MAX - 1) * 0.5*M_PI;
    yAngle = (((float) 2 * rand()) / RAND_MAX - 1) * AR_FILTER_IMAGE_CROP_FOVY;
    angles2point(xAngle, yAngle, &x1, &y1);
    inversePoint(x1, y1, &x1p, &y1p);
    correctPoint(x1p, y1p, &x1_o, &y1_o);
    point2angles(x1_o, y1_o, &xAngle_o, &yAngle_o);
    PRINT("Angles (%0.1f %0.1f) point(%0.1f, %0.1f) pixel(%0.1f %0.1f) point(%0.1f %0.1f) angles(%0.1f %0.1f)\n", xAngle / M_PI * 180, yAngle / M_PI * 180, x1, y1, x1p, y1p, x1_o, y1_o, xAngle_o / M_PI * 180, yAngle_o / M_PI * 180);
    */
    uint16_t horizPos             = horizonPos(0.0);
    // Top
    angles2point(-75.0/180.0*M_PI, 0.5 * AR_FILTER_IMAGE_CROP_FOVY, &x1, &y1);
    inversePoint(x1,y1,&left[0],&left[1]);
    angles2point(0.0, 0.5 * AR_FILTER_IMAGE_CROP_FOVY, &x1, &y1);
    inversePoint(x1,y1,&center[0],&center[1]);
    angles2point(75.0/180.0*M_PI, 0.5 * AR_FILTER_IMAGE_CROP_FOVY, &x1, &y1);
    inversePoint(x1,y1,&right[0],&right[1]);
    uint16_t top        = (uint16_t) round( max( right[1], max( left[1], center[1])));
    // Top
    angles2point(-75.0/180.0*M_PI, -0.5 * AR_FILTER_IMAGE_CROP_FOVY, &x1, &y1);
    inversePoint(x1,y1,&left[0],&left[1]);
    angles2point(0.0, -0.5 * AR_FILTER_IMAGE_CROP_FOVY, &x1, &y1);
    inversePoint(x1,y1,&center[0],&center[1]);
    angles2point(75.0/180.0*M_PI, -0.5 * AR_FILTER_IMAGE_CROP_FOVY, &x1, &y1);
    inversePoint(x1,y1,&right[0],&right[1]);
    int16_t desOffset           = (int16_t) round( min( right[1], min( left[1], center[1])));
    uint16_t desHeight          = (top - desOffset);
#if AR_FILTER_ISP_CROP
    int16_t fillHeight          = (int16_t) round( 0.5*initialWidth - (horizPos - desOffset) );
    desOffset                  -= fillHeight;
    desHeight                  += fillHeight;
    if(desOffset < -MT9F002_INITIAL_OFFSET_X){
            desOffset = -MT9F002_INITIAL_OFFSET_X;
    }
#endif
    VERBOSE_PRINT("ispScalar: %0.2f, desOffset: %d, ispHeight: %d, desHeight: %d\n",ispScalar, desOffset,ispHeight,desHeight);

    if(desHeight > ispHeight){
        desOffset = 0;
        desHeight = ispHeight;
    }
    if(desHeight > initialWidth){
        desOffset                  += (uint16_t) round((desHeight - initialWidth) / 2.0);
        desHeight                   = initialWidth;
    }
    if((desHeight + desOffset) > ispHeight){
        desHeight                   = ispHeight - desOffset;
        //desOffset                   = ispHeight - desHeight;
    }
    if((desOffset & 1) != 0){
        desOffset--;
    }
#if AR_FILTER_ISP_CROP
    cropCol                     = desOffset + fillHeight;
    mt9f002.offset_x            = MT9F002_INITIAL_OFFSET_X + desOffset / ispScalar;
    mt9f002.output_width        = desHeight;
    mt9f002.sensor_width        = desHeight / ispScalar;
    Rect crop;
    if(fillHeight >= 0 && desHeight <= width){
        crop                   = cvRect(fillHeight, 0, desHeight - fillHeight, height);
    }
    else{
        crop                   = cvRect(0, 0, width, height);
    }
    mt9f002_update_resolution(&mt9f002);
#else
    cropCol                     = desOffset;
    Rect crop                   = cvRect(desOffset,0,desHeight,ispWidth);
#endif
    return crop;
}

void correctPoint(double x_in, double y_in, double *x_out, double *y_out){
    double f, r, theta, corR, maxR;
    f       = scale_f * default_orbDiag * ispScalar / (4 * sin(M_PI / 4));

    x_in   -= ispWidth * 0.5;
    y_in   -= ispHeight * 0.5;
    r       = sqrt( pow(x_in, 2.0) + pow(y_in, 2.0) );
    theta   = atan2( y_in, x_in );
    corR    = correctRadius(r, f, default_k);
    maxR    = correctRadius(CFG_MT9F002_FISHEYE_RADIUS * ispScalar, f, default_k);
    x_in    = (corR * cos(theta)) / maxR;
    y_in    = (corR * sin(theta)) / maxR;

    outputCoord(x_in, y_in, x_out, y_out);
}

void angles2point(double xAngle, double yAngle, double *x_out, double * y_out){
    *x_out = AR_FILTER_VIEW_R * tan(xAngle);
    *y_out = AR_FILTER_VIEW_R * tan(yAngle);
}

void point2angles(double x_out, double y_out, double *xAngle, double *yAngle){
    *xAngle = atan(x_out / AR_FILTER_VIEW_R);
    *yAngle = atan(y_out / AR_FILTER_VIEW_R);
}

void inversePoint(double x_out, double y_out, double *x_in, double *y_in){
    double f, r, theta,corR, maxR;
    f           = scale_f * default_orbDiag * ispScalar / (4 * sin(M_PI / 4));

    inputCoord(x_out, y_out, x_in, y_in);
    maxR        = correctRadius(CFG_MT9F002_FISHEYE_RADIUS * ispScalar, f, default_k);
    r           = maxR * sqrt( pow( *x_in, 2.0 ) + pow( *y_in, 2.0 ) );
    theta       = atan2( *y_in, *x_in);
    corR        = invertRadius(r, f, default_k);
    *x_in       = ispWidth * 0.5 + corR * cos(theta);
    *y_in       = ispHeight * 0.5 + corR * sin(theta) ;
}

void plotHorizontalLine(Mat& sourceFrameCrop, double yAngle, double xResDeg){
    double x1, y1, x1p, y1p, x2, y2, x2p, y2p;
    // Plot horizontal line
    angles2point(-70.0/180.0*M_PI, yAngle, &x1, &y1);
    inversePoint(x1, y1, &x1p, &y1p);
    char text[50];
    for(double x_angle = -76+xResDeg; x_angle<=76; x_angle+=xResDeg){
        angles2point(x_angle / 180 * M_PI, yAngle, &x2, &y2);
        inversePoint(x2, y2, &x2p, &y2p);
        line(sourceFrameCrop, Point(y1p - cropCol,x1p), Point(y2p - cropCol,x2p), Scalar(0,127), 1);
        x1p = x2p;
        y1p = y2p;
    }
    for(double i = -75; i <= 75; i+=15){
        angles2point(i / 180 * M_PI, yAngle, &x1, &y1);
        inversePoint(x1, y1, &x1p, &y1p);
        sprintf(text,"- %0.0f", i);
        putText(sourceFrameCrop, text, Point(y1p - cropCol,x1p), FONT_HERSHEY_PLAIN, 1, Scalar(0,255), 1);

    }
}

void plotVerticalLine(Mat& sourceFrameCrop, double xAngle, double yResDeg){
    double x1, y1, x1p, y1p, x2, y2, x2p, y2p;
    // Plot vertical line
    angles2point(xAngle, -0.5*M_PI, &x1, &y1);
    inversePoint(x1, y1, &x1p, &y1p);
    char text[50];
    for(double y_angle = (-90+yResDeg)/180*M_PI; y_angle<=0.5*M_PI + MT9F002_THETA_OFFSET + eulerAngles->theta; y_angle+=yResDeg / 180 * M_PI){
        angles2point(xAngle, y_angle, &x2, &y2);
        inversePoint(x2, y2, &x2p, &y2p);
        line(sourceFrameCrop, Point(y1p - cropCol,x1p), Point(y2p - cropCol,x2p), Scalar(0,127), 1);
        x1p = x2p;
        y1p = y2p;
    }
    for(double i = -0.5*M_PI; i <= 0.5*M_PI + MT9F002_THETA_OFFSET + eulerAngles->theta; i+=15.0/180.0*M_PI){
        angles2point(xAngle, i, &x1, &y1);
        inversePoint(x1, y1, &x1p, &y1p);
        sprintf(text,"| %0.0f", i / M_PI * 180);
        putText(sourceFrameCrop, text, Point(y1p - cropCol,x1p), FONT_HERSHEY_PLAIN, 1, Scalar(0,255), 1);
    }
}

void plotHorizon(Mat& sourceFrameCrop){
    //plotHorizontalLine(sourceFrameCrop, -0.5 * AR_FILTER_IMAGE_CROP_FOVY, 5);
    plotHorizontalLine(sourceFrameCrop,   0.0 / 180.0 * M_PI, 5);
    //plotHorizontalLine(sourceFrameCrop,  0.5 * AR_FILTER_IMAGE_CROP_FOVY, 5);
    //plotVerticalLine(sourceFrameCrop, -45.0 / 180.0 * M_PI, 5);
    //plotVerticalLine(sourceFrameCrop,   0.0 / 180.0 * M_PI, 5);
    //plotVerticalLine(sourceFrameCrop,  45.0 / 180.0 * M_PI, 5);
}

void trackObjects(Mat& sourceFrame, Mat& frameGrey){
    // Main function for tracking multiple objects on the frame
    pixCount    = 0;
    pixSucCount = 0;
    pixSrcCount = 0;
    pixNofCount = 0;
    pixDupCount = 0;
    if(AR_FILTER_FLOOD_STYLE != AR_FILTER_FLOOD_CW){
        processImage_omni(sourceFrame, frameGrey, AR_FILTER_RND_PIX_SAMPLE);
        processCrops(frameGrey);
    }
    else{
        processImage_cw(sourceFrame, frameGrey, AR_FILTER_RND_PIX_SAMPLE);
    }
#if AR_FILTER_BENCHMARK
    addBenchmark("image Thresholded");
#endif // AR_FILTER_BENCHMARK
    return;
}

void processCrops(Mat& frameGrey){
    vector<vector<Point> > contours;
    for(unsigned int r=0; r < cropAreas.size(); r++)
    {
        if(cropAreas[r].x != 0 && cropAreas[r].width != 0)
        {
            contours.clear();
#if AR_FILTER_MOD_VIDEO && AR_FILTER_DRAW_BOXES
            findContours(frameGrey(cropAreas[r]).clone(), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
#else
            findContours(frameGrey(cropAreas[r]), contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
#endif //AR_FILTER_MOD_VIDEO && AR_FILTER_DRAW_BOXES
            for(unsigned int tc=0; tc < contours.size(); tc++)
            {
                if(contours[tc].size() > AR_FILTER_MIN_POINTS){
                    addContour(contours[tc], (uint16_t) cropAreas[r].x, (uint16_t) cropAreas[r].y);
                }
            }
        }
    }
#if AR_FILTER_BENCHMARK
    addBenchmark("Contours found");
#endif //AR_FILTER_BENCHMARK
}

void eraseMemory(void){
    // This function let's the agent forget previous measurements
    uint8_t index   = 0;
    uint8_t i       = 0;
    for(i=0; i < neighbourMem_size; i++){
        if((runCount - neighbourMem[i].lastSeen) <= AR_FILTER_MEMORY){
            if(i != index){
                neighbourMem[index] = neighbourMem[i];
            }
            index++;
        }
    }
    neighbourMem_size -= i - index;
}

void identifyObject(trackResults* trackRes){
	// We now only have memory samples from the past AR_FILTER_MEMORY frames so lets try to identify the neighbours we saw
    for(unsigned int i=0; i < neighbourMem_size; i++)
    {
        double radius	= (runCount - neighbourMem[i].lastSeen) * 1.0 / AR_FILTER_FPS * AR_FILTER_VMAX;
        double dx 		= trackRes->x_w - neighbourMem[i].x_w;
        double dy       = trackRes->y_w - neighbourMem[i].y_w;
        if(dx <= radius && dy <= radius && sqrt(pow(dx, 2.0) + pow(dy, 2.0)) <= radius)
        {
            PRINT("Identified object %d at (%d, %d)p (%f, %f)w\n", neighbourMem[i].id, trackRes->x_p, trackRes->y_p, trackRes->x_w, trackRes->y_w);
            neighbourMem[i].lastSeen 	= runCount;
            neighbourMem[i].x_w 		= trackRes->x_w;
            neighbourMem[i].y_w 		= trackRes->y_w;
            neighbourMem[i].z_w 		= trackRes->z_w;
            neighbourMem[i].x_p 		= trackRes->x_p;
            neighbourMem[i].y_p 		= trackRes->y_p;
            neighbourMem[i].area_p      = trackRes->area_p;
            neighbourMem[i].r_c         = trackRes->r_c;
            return;
        }
    }
    // We haven't identified
    PRINT("New object at (%4d, %4d)p (%6.2f, %6.2f)c (%5.2f, %5.2f)b (%5.2f, %5.2f)w\n", trackRes->x_p, trackRes->y_p, trackRes->x_c * 180 / M_PI, trackRes->y_c * 180 / M_PI, trackRes->x_b, trackRes->y_b, trackRes->x_w, trackRes->y_w);
    memoryBlock curN;
    curN.lastSeen 	= runCount;
    curN.id 		= maxId;
    curN.x_w 		= trackRes->x_w;
    curN.y_w 		= trackRes->y_w;
    curN.z_w 		= trackRes->z_w;
    curN.x_p 		= trackRes->x_p;
    curN.y_p 		= trackRes->y_p;
    curN.area_p     = trackRes->area_p;
    curN.r_c        = trackRes->r_c;
    neighbourMem_add(curN);
    maxId++;
    return;
}

void estimatePosition(uint16_t xp, uint16_t yp, uint32_t area, double position[3]){
    // This function estimates the 3D position (in camera  coordinate system) according to pixel position
    // Calculate corrected calibration parameters
    double x_in, y_in, xAngle, yAngle;
    double calArea          = round(default_calArea * pow(ispScalar,2.0));
    double fovDiag          = M_PI;                                                         // [radian] Diagonal field of view (see bebop manual)
    uint16_t cX             = round(ispWidth * 0.5);
    uint16_t cY             = round(ispHeight * 0.5);
    double f                = scale_f * default_orbDiag * ispScalar / (4 * sin(fovDiag / 4));                       // [mm]
    int16_t x               = yp - cX;                                                      // rotate frame cc 90 degrees (ISP is rotated 90 degrees cc)
    int16_t y               = xp - cY;                                                      // rotate frame cc 90 degrees (ISP is rotated 90 degrees cc)
    double r                = sqrt( pow( (double) x, 2.0 ) + pow( (double) y, 2.0 ) );   // [mm] radial distance from middle of CMOS
    double corR             = correctRadius(r, f, default_k);
    double corArea          = area * pow(corR / r, 2.0);
    double dist             = sqrt((double) calArea) / sqrt(corArea);
    correctPoint((double) yp, (double) xp, &x_in, &y_in);
    point2angles(x_in, y_in, &xAngle, &yAngle);
    VERBOSE_PRINT("pixel(%d, %d) point(%0.2f, %0.2f) angles(%0.2f, %0.2f)\n",xp, yp, x_in, y_in, xAngle / M_PI * 180, yAngle / M_PI * 180);
    position[0]             = xAngle;
    position[1]             = yAngle;
    position[2]             = dist;
    return;
}

double correctRadius(double r, double f, double k){
    // This function calculates the corrected radius for radial distortion
    // According to the article "A Generic Non-Linear Method for Fisheye Correction" by Dhane, Kutty and Bangadkar
    return f * tan( asin( sin( atan( r / f ) ) * k ) );
}

double invertRadius(double r, double f, double k){
    // This function calculates the inverted radius for corrected radial distortion
    // According to the article "A Generic Non-Linear Method for Fisheye Correction" by Dhane, Kutty and Bangadkar
    return f * tan( asin( sin( atan( r / f ) ) / k ) );
}

void showMat( double mat[16] );

uint16_t horizonPos( double y_orig ){
    double x_in, y_in;
    inversePoint(0.0, y_orig, &x_in, &y_in);
    return (uint16_t) round(y_in);
}

void showMat( double mat[16] ) {
    printf("| %6.3f  %6.3f  %6.3f  %6.3f |\n", mat[0], mat[4], mat[8], mat[12]);
    printf("| %6.3f  %6.3f  %6.3f  %6.3f |\n", mat[1], mat[5], mat[9], mat[13]);
    printf("| %6.3f  %6.3f  %6.3f  %6.3f |\n", mat[2], mat[6], mat[10], mat[14]);
    printf("| %6.3f  %6.3f  %6.3f  %6.3f |\n", mat[3], mat[7], mat[11], mat[15]);
}

void outputCoord(double x_in, double y_in, double *x_out, double *y_out){
    *x_out = (MVP[0] * x_in + MVP[4] * y_in + MVP[12]) / (MVP[3] * x_in + MVP[7] * y_in + MVP[15]);
    *y_out = (MVP[1] * x_in + MVP[5] * y_in + MVP[13]) / (MVP[3] * x_in + MVP[7] * y_in + MVP[15]);
}

void inputCoord(double x_out, double y_out, double *x_in, double *y_in){
    double a = MVP[0];
    double b = MVP[4];
    double c = MVP[12];

    double d = MVP[1];
    double e = MVP[5];
    double f = MVP[13];

    double k = MVP[3];
    double l = MVP[7];
    double m = MVP[15];

    double den   = (a*e - a*l*y_out - b*d + b*k*y_out + d*l*x_out - e*k*x_out);
    double x_num = (-b*f + b*m*y_out + c*e - c*l*y_out - e*m*x_out + f*l*x_out);
    double y_num = (-a*f + a*m*y_out + c*(d - k*y_out) - d*m*x_out + f*k*x_out);

    *x_in       =  x_num / (-den);
    *y_in       =  y_num / den;
}

void getMVP(double MVP[16]){
    double modelMat[16], viewMat[16], modelviewMat[16], projectionMat[16];
    double sensorRotation[16], eyeTranslationF[16], headingRotation[16], eyeTranslationR[16], modelMat_tmp1[16], modelMat_tmp2[16];
    double eye[4], forward[4], up[4];
    eye[0]      = 0.0;  eye[1]      =  0.0; eye[2]      = -AR_FILTER_VIEW_R;    eye[3]      = 1.0;
    forward[0]  = 0.0;  forward[1]  =  0.0; forward[2]  =  1.0;     forward[3]  = 1.0;
    up[0]       = 0.0;  up[1]       =  1.0; up[2]       =  0.0;     up[3]       = 1.0;
    // Create The model matrix
    setRotationMat(0.0, M_PI, 0.0, modelMat); // The ISP is 90 degrees rotated wrt the camera and the pinhole camera model flips the image
    /*setTranslationMat(-eye[0], -eye[1], -eye[2], eyeTranslationF);
    setRotationMat(0.0, eulerAngles->psi, 0.0, headingRotation);
    setTranslationMat( eye[0],  eye[1],  eye[2], eyeTranslationR);
    matrixMultiply(eyeTranslationF, sensorRotation, modelMat_tmp1);
    matrixMultiply(headingRotation, modelMat_tmp1, modelMat_tmp2);
    matrixMultiply(eyeTranslationR, modelMat_tmp2, modelMat);*/
    // Create the view matrix
    rotateVector(-MT9F002_THETA_OFFSET - eulerAngles->theta, 0.0, -eulerAngles->phi, forward);
    rotateVector(-MT9F002_THETA_OFFSET - eulerAngles->theta, 0.0, -eulerAngles->phi, up);
    double center[4];
    center[0] = eye[0] + forward[0];
    center[1] = eye[1] + forward[1];
    center[2] = eye[2] + forward[2];
    center[3] = 1.0;
    view_set_lookat(viewMat, eye,  center,  up);
    setPerspectiveMat(projectionMat);
    matrixMultiply(viewMat, modelMat, modelviewMat);
    matrixMultiply(projectionMat, modelviewMat, MVP);
}

void setPerspectiveMat(double perspectiveMat[16])
{
    // Create perspective matrix
    // These paramaters are about lens properties.
    // The "near" and "far" create the Depth of Field.
    // The "angleOfView", as the name suggests, is the angle of view.
    // The "aspectRatio" is the cool thing about this matrix. OpenGL doesn't
    // has any information about the screen you are rendering for. So the
    // results could seem stretched. But this variable puts the thing into the
    // right path. The aspect ratio is your device screen (or desired area) width divided
    // by its height. This will give you a number < 1.0 the the area has more vertical
    // space and a number > 1.0 is the area has more horizontal space.
    // Aspect Ratio of 1.0 represents a square area.
    // Some calculus before the formula.
    float size      =  near * tanf((angleOfView / 180.0 * M_PI) / 2.0);
    float left      = -size;
    float right     =  size;
    float bottom    = -size;
    float top       =  size;
    // First Column
    perspectiveMat[0]  = 2 * near / (right - left);
    perspectiveMat[1]  = 0.0;
    perspectiveMat[2]  = 0.0;
    perspectiveMat[3]  = 0.0;
    // Second Column
    perspectiveMat[4]  = 0.0;
    perspectiveMat[5]  = 2 * near / (top - bottom);
    perspectiveMat[6]  = 0.0;
    perspectiveMat[7]  = 0.0;
    // Third Column
    perspectiveMat[8]  = (right + left) / (right - left);
    perspectiveMat[9]  = (top + bottom) / (top - bottom);
    perspectiveMat[10] = -(far + near) / (far - near);
    perspectiveMat[11] = -1;
    // Fourth Column
    perspectiveMat[12] = 0.0;
    perspectiveMat[13] = 0.0;
    perspectiveMat[14] = -(2 * far * near) / (far - near);
    perspectiveMat[15] = 0.0;
}

void translate_xyz(double result[16], const float translatex,
        const float translatey, const float translatez) {
    result[12] += result[0] * translatex + result[4] * translatey
            + result[8] * translatez;
    result[13] += result[1] * translatex + result[5] * translatey
            + result[9] * translatez;
    result[14] += result[2] * translatex + result[6] * translatey
            + result[10] * translatez;
    result[15] += result[3] * translatex + result[7] * translatey
            + result[11] * translatez;
}

float vector_length(const float x, const float y, const float z) {
    return (float) sqrt(x*x + y*y +z*z);
}

void view_set_lookat(double result[16], double eye[4], double center[4], double up[4]) {
    float fx = center[0] - eye[0];
    float fy = center[1] - eye[1];
    float fz = center[2] - eye[2];

    // normalize f
    float rlf = 1.0f / vector_length(fx, fy, fz);
    fx *= rlf;
    fy *= rlf;
    fz *= rlf;

    // compute s = f x up (x means "cross product")
    float sx = fy * up[2] - fz * up[1];
    float sy = fz * up[0] - fx * up[2];
    float sz = fx * up[1] - fy * up[0];

    // and normalize s
    float rls = 1.0f / vector_length(sx, sy, sz);
    sx *= rls;
    sy *= rls;
    sz *= rls;

    //The up vector must not be parallel to the line of sight from the eye point to the reference point.
    if((0 == sx)&&(0 == sy)&&(0 == sz))
        return;

    // compute u = s x f
    float ux = sy * fz - sz * fy;
    float uy = sz * fx - sx * fz;
    float uz = sx * fy - sy * fx;

    result[0] = sx;
    result[1] = ux;
    result[2] = -fx;
    result[3] = 0.0f;

    result[4] = sy;
    result[5] = uy;
    result[6] = -fy;
    result[7] = 0.0f;

    result[8] = sz;
    result[9] = uz;
    result[10] = -fz;
    result[11] = 0.0f;

    result[12] = 0.0f;
    result[13] = 0.0f;
    result[14] = 0.0f;
    result[15] = 1.0f;

    translate_xyz(result, -eye[0], -eye[1], -eye[2]);
}

void rotateVector(float x, float y, float z, double vector[4])
{
    double rotX[16];
    double rotY[16];
    double rotZ[16];
    setRotationMat(x, 0.0, 0.0, rotX);
    setRotationMat(0.0, y, 0.0, rotY);
    setRotationMat(0.0, 0.0, z, rotZ);
    // Multiply
    double rotZYXvec[4];
    double tmp1[16], tmp2[16];
    matrixMultiply(rotY, rotZ, tmp1);
    matrixMultiply(rotX, tmp1, tmp2);
    matvecMultiply(tmp2, vector, rotZYXvec);
    vector[0] = rotZYXvec[0];
    vector[1] = rotZYXvec[1];
    vector[2] = rotZYXvec[2];
    vector[3] = rotZYXvec[3];
}


void matrixMultiply(double m1[16], double m2[16], double result[16])
{
    // Fisrt Column
    result[0]  = m1[0]*m2[0] + m1[4]*m2[1] + m1[8]*m2[2] + m1[12]*m2[3];
    result[1]  = m1[1]*m2[0] + m1[5]*m2[1] + m1[9]*m2[2] + m1[13]*m2[3];
    result[2]  = m1[2]*m2[0] + m1[6]*m2[1] + m1[10]*m2[2] + m1[14]*m2[3];
    result[3]  = m1[3]*m2[0] + m1[7]*m2[1] + m1[11]*m2[2] + m1[15]*m2[3];
    // Second Column
    result[4]  = m1[0]*m2[4] + m1[4]*m2[5] + m1[8]*m2[6] + m1[12]*m2[7];
    result[5]  = m1[1]*m2[4] + m1[5]*m2[5] + m1[9]*m2[6] + m1[13]*m2[7];
    result[6]  = m1[2]*m2[4] + m1[6]*m2[5] + m1[10]*m2[6] + m1[14]*m2[7];
    result[7]  = m1[3]*m2[4] + m1[7]*m2[5] + m1[11]*m2[6] + m1[15]*m2[7];
    // Third Column
    result[8]  = m1[0]*m2[8] + m1[4]*m2[9] + m1[8]*m2[10] + m1[12]*m2[11];
    result[9]  = m1[1]*m2[8] + m1[5]*m2[9] + m1[9]*m2[10] + m1[13]*m2[11];
    result[10] = m1[2]*m2[8] + m1[6]*m2[9] + m1[10]*m2[10] + m1[14]*m2[11];
    result[11] = m1[3]*m2[8] + m1[7]*m2[9] + m1[11]*m2[10] + m1[15]*m2[11];
    // Fourth Column
    result[12] = m1[0]*m2[12] + m1[4]*m2[13] + m1[8]*m2[14] + m1[12]*m2[15];
    result[13] = m1[1]*m2[12] + m1[5]*m2[13] + m1[9]*m2[14] + m1[13]*m2[15];
    result[14] = m1[2]*m2[12] + m1[6]*m2[13] + m1[10]*m2[14] + m1[14]*m2[15];
    result[15] = m1[3]*m2[12] + m1[7]*m2[13] + m1[11]*m2[14] + m1[15]*m2[15];
}

void matvecMultiply(double m1[16], double v1[4], double result[4])
{
    // Fisrt Column
    result[0]  = m1[0]*v1[0] + m1[4]*v1[1] + m1[8]*v1[2] + m1[12]*v1[3];
    result[1]  = m1[1]*v1[0] + m1[5]*v1[1] + m1[9]*v1[2] + m1[13]*v1[3];
    result[2]  = m1[2]*v1[0] + m1[6]*v1[1] + m1[10]*v1[2] + m1[14]*v1[3];
    result[3]  = m1[3]*v1[0] + m1[7]*v1[1] + m1[11]*v1[2] + m1[15]*v1[3];
}

void setIdentityMatrix(double m[16])
{
    m[0]  = m[5]  = m[10] = m[15] = 1.0;
    m[1]  = m[2]  = m[3]  = m[4]  = 0.0;
    m[6]  = m[7]  = m[8]  = m[9]  = 0.0;
    m[11] = m[12] = m[13] = m[14] = 0.0;
}

void setRotationMat(float x, float y, float z, double outputMat[16])
{
    double rotX[16];
    double rotY[16];
    double rotZ[16];
    setIdentityMatrix(outputMat);
    setIdentityMatrix(rotX);
    setIdentityMatrix(rotY);
    setIdentityMatrix(rotZ);
    // Set z rotation
    rotZ[0]  =  cosf(z);
    rotZ[1]  =  sinf(z);
    rotZ[4]  = -rotZ[1];
    rotZ[5]  =  rotZ[0];
    // Set y rotation
    rotY[0]  =  cosf(y);
    rotY[2]  =  sinf(y);
    rotY[8]  = -rotY[2];
    rotY[10] =  rotY[0];
    // Set x rotation
    rotX[5] = cosf(x);
    rotX[6] = -sinf(x);
    rotX[9] = -rotX[6];
    rotX[10] = rotX[5];

    // Multiply
    double rotXY[16];
    matrixMultiply(rotY, rotX, rotXY);
    matrixMultiply(rotZ, rotXY, outputMat);
}

void setTranslationMat(float x, float y, float z, double outputMat[16])
{
    setIdentityMatrix(outputMat);
    outputMat[12] = x;
    outputMat[13] = y;
    outputMat[14] = z;
}

void cam2body(trackResults* trackRes){
    // Neighbour position returned in 2 angles and a radius.
    // x_c is the angle wrt vertical camera axis.   Defined clockwise/right positive
    // y_c is angle wrt camera horizon axis.        Defined upwards positive
    // r_c is radial distance in m.
    trackRes->x_b = trackRes->r_c * cos( -trackRes->y_c) * cos( trackRes->x_c ) + MT9F002_X_OFFSET;
    trackRes->y_b = trackRes->r_c * cos( -trackRes->y_c) * sin( trackRes->x_c ) + MT9F002_Y_OFFSET;
    trackRes->z_b = trackRes->r_c * sin( -trackRes->y_c) + MT9F002_Z_OFFSET;
    VERBOSE_PRINT("camera (%0.2f deg, %0.2f deg, %0.2f m) -> body (%0.2f m, %0.2f m, %0.2f m)\n", trackRes->x_c * 180/M_PI, trackRes->y_c * 180/M_PI, trackRes->r_c, trackRes->x_b, trackRes->y_b, trackRes->z_b);
    return;
}

void body2world(trackResults* trackRes){
    struct NedCoor_f *pos;
#if AR_FILTER_WORLDPOS
    pos     = stateGetPositionNed_f();      // Get your current position
#else
    struct NedCoor_f fakePos;
    fakePos.x       = 0.0;
    fakePos.y       = 0.0;
    fakePos.z       = 0.0;
    pos             = &fakePos;
#endif
#if AR_FILTER_NOYAW
    double psi      = 0.0;
#else
    double psi      = eulerAngles->psi;
#endif
    Matx33f rotX(    1,                         0,                      0,
                     0,                         cos(eulerAngles->phi), -sin(eulerAngles->phi),
                     0,                         sin(eulerAngles->phi),  cos(eulerAngles->phi));

    Matx33f rotY(    cos(eulerAngles->theta),   0,                      sin(eulerAngles->theta),
                     0,          1,             0,
                    -sin(eulerAngles->theta),   0,                      cos(eulerAngles->theta));

    Matx33f rotZ(    cos(psi),                 -sin(psi),               0,
                     sin(psi),                  cos(psi),               0,
                     0,                         0,                      1);
    Matx31f bPos(trackRes->x_b, trackRes->y_b, trackRes->z_b);
    Matx31f wPos    = rotZ * rotY * rotX * bPos;
    trackRes->x_w   = wPos(0,0) + pos->x + AR_FILTER_OBJ_X_OFFSET;
    trackRes->y_w   = wPos(1,0) + pos->y + AR_FILTER_OBJ_Y_OFFSET;
#if AR_FILTER_USE_ALTITUDE
    trackRes->z_w   = wPos(2,0) + pos->z + AR_FILTER_OBJ_Z_OFFSET;
#else
    trackRes->z_w   = wPos(2,0) + AR_FILTER_OBJ_Z_OFFSET;
#endif
    VERBOSE_PRINT("body (%0.2f m, %0.2f m, %0.2f m) + pos(%0.2f m, %0.2f m, %0.2f m) + euler (%0.2f deg, %0.2f deg, %0.2f deg) -> world (%0.2f m, %0.2f m, %0.2f m)\n", trackRes->x_b, trackRes->y_b, trackRes->z_b, pos->x, pos->y, pos->z, eulerAngles->phi, eulerAngles->theta, psi, trackRes->x_w, trackRes->y_w, trackRes->z_w);
    return;
}

bool addContour(vector<Point> contour, uint16_t offsetX, uint16_t offsetY, double minDist, double maxDist){
    Moments m;
    if(AR_FILTER_FLOOD_STYLE == AR_FILTER_FLOOD_CW){
        VERBOSE_PRINT("Analyzing contour of length %d\n", objCont_size);
        m = objCont_moments();
    }
    else{
        VERBOSE_PRINT("Analyzing contour of length %d\n", contour.size());
        m = moments(contour);
    }
    VERBOSE_PRINT("m00: %f  x: %f  y: %f\n",m.m00, m.m10 / m.m00, m.m01 / m.m00);
    //double e = (pow(m.mu20 - m.mu02,2.0) - 4 * pow(m.mu11,2.0)) / pow(m.mu20 + m.mu02,2.0);
    double semi_major   = sqrt( 2 * ( m.mu20 + m.mu02 + sqrt( pow(m.mu20 - m.mu02, 2.0) + 4 * pow( m.mu11, 2.0 ) ) ) / m.m00 );
    double semi_minor   = sqrt( 2 * ( m.mu20 + m.mu02 - sqrt( pow(m.mu20 - m.mu02, 2.0) + 4 * pow( m.mu11, 2.0 ) ) ) / m.m00 );
    double e            = sqrt( 1 - pow( semi_minor, 2.0 ) / pow( semi_major, 2.0 ));
    double corArea      = M_PI * pow( semi_major, 2.0 );
    if(e < AR_FILTER_MAX_CIRCLE_DEF)
    {
        if ( m.m00 / corArea > AR_FILTER_MIN_CIRCLE_PERC && corArea >= AR_FILTER_MIN_CIRCLE_SIZE)
        {
            VERBOSE_PRINT("ecc: %f   smax: %f    smin: %f    corArea: %f\n", e, semi_major, semi_minor, corArea);
            trackResults curRes;
            double position[3];
            curRes.x_p 		    = m.m10 / m.m00 + cropCol + offsetX;
            curRes.y_p 		    = m.m01 / m.m00 + offsetY;
            //curRes.area_p       = (uint32_t) m.m00;
            curRes.area_p 	    = (uint32_t) M_PI * pow( semi_major, 2.0 );
            estimatePosition(curRes.x_p, curRes.y_p, curRes.area_p, position);  // Estimate position in camera reference frame based on pixel location and area
            curRes.x_c 		    = position[0];
            curRes.y_c 		    = position[1];
            curRes.r_c 		    = position[2];
            if((!minDist || minDist <= curRes.r_c) && (!maxDist || maxDist >= curRes.r_c)){
                if(curRes.r_c <= AR_FILTER_CAM_RANGE){
                    uint8_t overwriteId = trackRes_size;    // Invalid index, so won't overwrite
                    for(uint8_t tr = 0; tr < trackRes_size; tr++){
                        if( sqrt( pow( (double) (curRes.x_p - trackRes[tr].x_p), 2.0) + pow( (double) (curRes.y_p - trackRes[tr].y_p),2.0)) < sqrt( max( curRes.area_p, trackRes[tr].area_p ) / M_PI ) ){
                            if(curRes.area_p < trackRes[tr].area_p){
                                return false;
                            }
                            overwriteId = tr;               // Mark this result for overwriting
                        }
                    }
                    trackRes_add(curRes, overwriteId);     // Save results and push into trackRes
                    return true;
                }
            }
        }else if(AR_FILTER_SHOW_REJECT){
            VERBOSE_PRINT("Rejected. contour area %0.1f, circle area %0.1f\n",m.m00,corArea);
        }
    }else if(AR_FILTER_SHOW_REJECT) {
        VERBOSE_PRINT("Rejected. contour area %f, eccentricity: %f.\n",m.m00, e);
    }
	return false;
}

static Moments objCont_moments( void ){
    Moments m;
    if( objCont_size == 0 )
        return m;

    double a00 = 0, a10 = 0, a01 = 0, a20 = 0, a11 = 0, a02 = 0, a30 = 0, a21 = 0, a12 = 0, a03 = 0;
    double xi, yi, xi2, yi2, xi_1, yi_1, xi_12, yi_12, dxy, xii_1, yii_1;

    xi_1 = objCont_store[objCont_size-1].x;
    yi_1 = objCont_store[objCont_size-1].y;

    xi_12 = xi_1 * xi_1;
    yi_12 = yi_1 * yi_1;

    for( int i = 0; i < objCont_size; i++ )
    {
        xi = objCont_store[i].x;
        yi = objCont_store[i].y;

        xi2 = xi * xi;
        yi2 = yi * yi;
        dxy = xi_1 * yi - xi * yi_1;
        xii_1 = xi_1 + xi;
        yii_1 = yi_1 + yi;

        a00 += dxy;
        a10 += dxy * xii_1;
        a01 += dxy * yii_1;
        a20 += dxy * (xi_1 * xii_1 + xi2);
        a11 += dxy * (xi_1 * (yii_1 + yi_1) + xi * (yii_1 + yi));
        a02 += dxy * (yi_1 * yii_1 + yi2);
        a30 += dxy * xii_1 * (xi_12 + xi2);
        a03 += dxy * yii_1 * (yi_12 + yi2);
        a21 += dxy * (xi_12 * (3 * yi_1 + yi) + 2 * xi * xi_1 * yii_1 +
                   xi2 * (yi_1 + 3 * yi));
        a12 += dxy * (yi_12 * (3 * xi_1 + xi) + 2 * yi * yi_1 * xii_1 +
                   yi2 * (xi_1 + 3 * xi));
        xi_1 = xi;
        yi_1 = yi;
        xi_12 = xi2;
        yi_12 = yi2;
    }

    if( fabs(a00) > 1.19209289550781250000e-7F)
    {
        double db1_2, db1_6, db1_12, db1_24, db1_20, db1_60;

        if( a00 > 0 )
        {
            db1_2 = 0.5;
            db1_6 = 0.16666666666666666666666666666667;
            db1_12 = 0.083333333333333333333333333333333;
            db1_24 = 0.041666666666666666666666666666667;
            db1_20 = 0.05;
            db1_60 = 0.016666666666666666666666666666667;
        }
        else
        {
            db1_2 = -0.5;
            db1_6 = -0.16666666666666666666666666666667;
            db1_12 = -0.083333333333333333333333333333333;
            db1_24 = -0.041666666666666666666666666666667;
            db1_20 = -0.05;
            db1_60 = -0.016666666666666666666666666666667;
        }

        // spatial moments
        m.m00 = a00 * db1_2;
        m.m10 = a10 * db1_6;
        m.m01 = a01 * db1_6;
        m.m20 = a20 * db1_12;
        m.m11 = a11 * db1_24;
        m.m02 = a02 * db1_12;
        m.m30 = a30 * db1_20;
        m.m21 = a21 * db1_60;
        m.m12 = a12 * db1_60;
        m.m03 = a03 * db1_20;

        double cx = 0, cy = 0;
        double mu20, mu11, mu02;
        double inv_m00 = 0.0;

        if( fabs(m.m00) > double(2.22044604925031308085e-16L))
        {
            inv_m00 = 1. / m.m00;
            cx = m.m10 * inv_m00;
            cy = m.m01 * inv_m00;
        }

        mu20 = m.m20 - m.m10 * cx;
        mu11 = m.m11 - m.m10 * cy;
        mu02 = m.m02 - m.m01 * cy;

        m.mu20 = mu20;
        m.mu11 = mu11;
        m.mu02 = mu02;

        m.mu30 = m.m30 - cx * (3 * mu20 + cx * m.m10);
        mu11 += mu11;
        m.mu21 = m.m21 - cx * (mu11 + cx * m.m01) - cy * mu20;
        m.mu12 = m.m12 - cy * (mu11 + cy * m.m10) - cx * mu02;
        m.mu03 = m.m03 - cy * (3 * mu02 + cy * m.m01);

        double inv_sqrt_m00 = std::sqrt(std::abs(inv_m00));
        double s2 = inv_m00*inv_m00, s3 = s2*inv_sqrt_m00;

        m.nu20 = m.mu20*s2; m.nu11 = m.mu11*s2; m.nu02 = m.mu02*s2;
        m.nu30 = m.mu30*s3; m.nu21 = m.mu21*s3; m.nu12 = m.mu12*s3; m.nu03 = m.mu03*s3;
    }
    return m;
}

void createSearchGrid(uint16_t x_p, uint16_t y_p, Point searchGrid[], uint8_t searchLayer, uint16_t sGridSize, int* maxRow, int* maxCol){
    if(searchLayer > 0){
        int16_t curY   = y_p - searchLayer * sGridSize;
        int16_t curX   = x_p - searchLayer * sGridSize;
        int8_t  dX,dY;
        for(unsigned int s = 0; s < 4; s++){
            switch(s){
            case 0 :    dX =  sGridSize;    dY =  0;            break; // Right
            case 1 :    dX =  0;            dY = -sGridSize;    break; // Down
            case 2 :    dX = -sGridSize;    dY =  0;            break; // Left
            case 3 :    dX =  0;            dY =  sGridSize;    break; // Up
            }
            for(unsigned int i = 0; i < searchLayer * 2; i++){
                curY += dY;
                curX += dX;
                if(curY >= 0 && curX >= 0 && curY < *maxRow && curX < *maxCol){
                    searchGrid[ s * 2 * searchLayer + i] = Point(curY , curX);
                }
                else{
                    searchGrid[ s * 2 * searchLayer + i] = Point(y_p, x_p); // Add duplicate point
                }
            }
        }
    }
    else{
        searchGrid[0] = Point( y_p, x_p );
    }
}

bool processImage_cw(Mat& sourceFrame, Mat& destFrame, uint16_t sampleSize){
    bool obj_detected   = false;
    bool foundObj       = false;
    objCont_size        = 0;
    if (sourceFrame.cols > 0 && sourceFrame.rows > 0)
    {
        if (AR_FILTER_SAMPLE_STYLE > 0){
            const uint8_t maxLayer  = 12;
            Point searchGrid[8*maxLayer];
            for(unsigned int rnm=0; rnm < neighbourMem_size; rnm++)
            {
                if( ( ( (int16_t) neighbourMem[rnm].x_p ) - cropCol ) >= 0 && ( ( (int16_t) neighbourMem[rnm].x_p ) - cropCol ) < sourceFrame.cols && neighbourMem[rnm].y_p < sourceFrame.rows){
                    VERBOSE_PRINT("Looking for object %d\n",neighbourMem[rnm].id);
                    uint8_t searchLayer     = 0;
                    uint8_t searchPoints    = 1;
                    uint16_t sGridSize      = 0.1 * sqrt(((float) neighbourMem[rnm].area_p) / M_PI);
                    foundObj                = false; // We're pessimistic that we can find the same object
                    while(!foundObj && searchLayer < maxLayer){
                        createSearchGrid(neighbourMem[rnm].x_p - cropCol, neighbourMem[rnm].y_p, searchGrid, searchLayer, sGridSize, &sourceFrame.rows, &sourceFrame.cols);
                        for(uint8_t rsg = 0; rsg < searchPoints; rsg++)
                        {
                            layerDepth              = 0;
                            //VERBOSE_PRINT("searching at (%d, %d) (w:%d, h:%d)\n",searchGrid[rsg].x, searchGrid[rsg].y, sourceFrame.rows, sourceFrame.cols);
                            if(pixFindContour_cw(sourceFrame, destFrame, searchGrid[rsg].x, searchGrid[rsg].y, ARF_SEARCH, true) == ARF_FINISHED){
                                double r_margin = (runCount - neighbourMem[rnm].lastSeen) * AR_FILTER_VMAX * 1 / AR_FILTER_FPS;
                                if(objCont_add(neighbourMem[rnm].r_c - r_margin, neighbourMem[rnm].r_c + r_margin)){
                                    VERBOSE_PRINT("Found object %d from (%d, %d) at (%d, %d) after %d layers\n",neighbourMem[rnm].id, neighbourMem[rnm].x_p, neighbourMem[rnm].y_p, trackRes[trackRes_lastId].x_p, trackRes[trackRes_lastId].y_p, searchLayer);
                                    foundObj                = true;
                                    obj_detected            = true;
                                    break;
                                }
                            }
                        }
                        searchLayer++;
                        searchPoints = 8 * searchLayer;
                    }
                    if(!foundObj){
                        objCont_size = 0;
                        VERBOSE_PRINT("Could not find object %d\n",neighbourMem[rnm].id);
                    }
                }
                else{
                    VERBOSE_PRINT("Object %d is not within valid bounds (%d, %d) [0-%d][0-%d]\n",neighbourMem[rnm].id, ( (int16_t) neighbourMem[rnm].x_p ) - cropCol, neighbourMem[rnm].y_p, sourceFrame.cols, sourceFrame.rows);
                }
            }
        }
        switch(AR_FILTER_SAMPLE_STYLE){
        case AR_FILTER_STYLE_FULL : {
            for(int r = 0; r < sourceFrame.rows; r++)
            {
                for(int c= 0; c < sourceFrame.cols; c++)
                {
                    layerDepth              = 0;
                    if(pixFindContour_cw(sourceFrame, destFrame, r, c, ARF_UP, false) == ARF_SUCCESS)
                    {
                        obj_detected            = true;
                    }
                }
            }
            Rect fullCrop;
            fullCrop.x 		= 1;
            fullCrop.y 		= 0;
            fullCrop.width 	= sourceFrame.cols-1;
            fullCrop.height = sourceFrame.rows;
            cropAreas.push_back(fullCrop);
            break;
        }
        case AR_FILTER_STYLE_GRID : {
            int spacing     = (int) sqrt((sourceFrame.rows * sourceFrame.cols) / sampleSize);
            for(int r = spacing; r < sourceFrame.rows; r+=spacing)
            {
                for(int c=spacing; c < sourceFrame.cols; c+=spacing)
                {
                    sample++;
                    layerDepth      = 0;
                    //objCont.clear();
                    objCont_size    = 0;
                    if(pixFindContour_cw(sourceFrame, destFrame, r, c, ARF_SEARCH, true) == ARF_FINISHED)
                    {
                        if(objCont_add()){
                            obj_detected = true;
                        }
                    }
                }
            }
            break;
        }
        case AR_FILTER_STYLE_RANDOM : {
            int rndRow, rndCol;
            for(int i = 0; i<sampleSize; i++)
            {
                layerDepth  = 0;
                sample++;
                rndRow      = (int) round(((double) rand())/((double) RAND_MAX)*(sourceFrame.rows-1));
                rndCol      = (int) round(((double) rand())/((double) RAND_MAX)*(sourceFrame.cols-1));
                objCont_size    = 0;
                bool too_close  = false;
                for(unsigned int tr = 0; tr < trackRes_size; tr++){
                    if(sqrt(pow((double) (rndCol - (trackRes[tr].x_p - cropCol)),2.0)+pow((double) (rndRow - trackRes[tr].y_p),2.0)) <= 1.15 * sqrt(trackRes[tr].area_p / M_PI)){
                        //VERBOSE_PRINT("Not using starting point %d, %d due to distance to object %d at %d, %d (dist: %f < %f)\n", rndRow, rndCol, tr, trackRes[tr].y_p, trackRes[tr].x_p - cropCol, 1.15 * sqrt(pow((double) (rndCol - (trackRes[tr].x_p - cropCol)),2.0)+pow((double) (rndRow - trackRes[tr].y_p),2.0)), sqrt(trackRes[tr].area_p / M_PI));
                        too_close = true;
                        break;
                    }

                }
                if(too_close){
                    continue;
                }
                if(pixFindContour_cw(sourceFrame, destFrame, rndRow, rndCol, ARF_SEARCH, true) == ARF_FINISHED)
                {
                    if(objCont_add()){
                        obj_detected = true;
                    }
                }
            }
            break;
        }
        }
    }
    return obj_detected;
}

bool processImage_omni(Mat& sourceFrame, Mat& destFrame, uint16_t sampleSize){
    bool obj_detected   = false;
    bool foundObj       = false;
    cropAreas.clear();
    if (sourceFrame.cols > 0 && sourceFrame.rows > 0)
    {
        if(AR_FILTER_SAMPLE_STYLE > 0){
            const uint8_t maxLayer  = 5;
            Point searchGrid[8*maxLayer];
            for(unsigned int rnm=0; rnm < neighbourMem_size; rnm++)
            {
                objCrop.x               = neighbourMem[rnm].x_p - cropCol;
                objCrop.y               = neighbourMem[rnm].y_p;
                objCrop.width           = 0;
                objCrop.height          = 0;
                uint8_t searchLayer     = 0;
                uint8_t searchPoints    = 1;
                uint16_t sGridSize      =  0.5 * sqrt(((float) neighbourMem[rnm].area_p) / M_PI);
                foundObj                = false; // We're pessimistic that we can find the same object
                while(!foundObj && searchLayer < maxLayer){
                    createSearchGrid(neighbourMem[rnm].x_p - cropCol, neighbourMem[rnm].y_p, searchGrid, searchLayer, sGridSize, &sourceFrame.rows, &sourceFrame.cols);
                    for(uint8_t rsg=0; rsg < searchPoints; rsg++)
                    {
                        layerDepth              = 0;
                        if(pixFindContour_omni(sourceFrame, destFrame, searchGrid[rsg].x, searchGrid[rsg].y, ARF_SEARCH, true) == ARF_FINISHED){
                            if(layerDepth > AR_FILTER_MIN_LAYERS){
                                objCrop                 = enlargeRectangle(sourceFrame, objCrop, AR_FILTER_CROP_X);
                                addCrop();
                                foundObj                = true;
                                obj_detected            = true;
                                break;
                            }
                        }
                    }
                    searchLayer++;
                    searchPoints = 8 * searchLayer;
                }
                if(!foundObj){
                    VERBOSE_PRINT("Could not find object %d\n",neighbourMem[rnm].id);
                }
            }
        }
        switch(AR_FILTER_SAMPLE_STYLE){
        case AR_FILTER_STYLE_FULL : {
            for(int r = 0; r < sourceFrame.rows; r++)
            {
                for(int c= 0; c < sourceFrame.cols; c++)
                {
                    layerDepth              = 0;
                    if(pixFindContour_omni(sourceFrame, destFrame, r, c, ARF_UP, false) == ARF_SUCCESS)
                    {
                        obj_detected            = true;
                    }
                }
            }
            Rect fullCrop;
            fullCrop.x      = 1;
            fullCrop.y      = 0;
            fullCrop.width  = sourceFrame.cols-1;
            fullCrop.height = sourceFrame.rows;
            cropAreas.push_back(fullCrop);
            break;
        }
        case AR_FILTER_STYLE_GRID : {
            int spacing     = (int) sqrt((sourceFrame.rows * sourceFrame.cols) / sampleSize);
            for(int r = spacing; r < sourceFrame.rows; r+=spacing)
            {
                for(int c=spacing; c < sourceFrame.cols; c+=spacing)
                {
                    sample++;
                    layerDepth      = 0;
                    objCrop.x       = c;
                    objCrop.y       = r;
                    objCrop.width   = 0;
                    objCrop.height  = 0;
                    if(pixFindContour_omni(sourceFrame, destFrame, r, c, ARF_SEARCH, true) == ARF_FINISHED)
                    {
                        if(layerDepth > AR_FILTER_MIN_LAYERS){
                            objCrop         = enlargeRectangle(sourceFrame, objCrop, AR_FILTER_CROP_X);
                            obj_detected    = true;
                            addCrop();
                        }
                    }
                }
            }
            break;
        }
        case AR_FILTER_STYLE_RANDOM : {
            int rndRow, rndCol;
            for(int i = 0; i<sampleSize; i++)
            {
                layerDepth  = 0;
                sample++;
                rndRow      = (int) round(((double) rand())/((double) RAND_MAX)*(sourceFrame.rows-1));
                rndCol      = (int) round(((double) rand())/((double) RAND_MAX)*(sourceFrame.cols-1));
                objCrop.x       = rndCol;
                objCrop.y       = rndRow;
                objCrop.width   = 0;
                objCrop.height  = 0;
                bool too_close  = false;
                for(unsigned int tr = 0; tr < trackRes_size; tr++){
                    if(sqrt(pow((double) (rndCol - (trackRes[tr].x_p - cropCol)),2.0)+pow((double) (rndRow - trackRes[tr].y_p),2.0)) <= 1.15 * sqrt(trackRes[tr].area_p / M_PI)){
                        //VERBOSE_PRINT("Not using starting point %d, %d due to distance to object %d at %d, %d (dist: %f < %f)\n", rndRow, rndCol, tr, trackRes[tr].y_p, trackRes[tr].x_p - cropCol, 1.15 * sqrt(pow((double) (rndCol - (trackRes[tr].x_p - cropCol)),2.0)+pow((double) (rndRow - trackRes[tr].y_p),2.0)), sqrt(trackRes[tr].area_p / M_PI));
                        too_close = true;
                        break;
                    }

                }
                if(too_close){
                    continue;
                }
                if(pixFindContour_omni(sourceFrame, destFrame, rndRow, rndCol, ARF_SEARCH, true) == ARF_FINISHED)
                {
                    if(layerDepth > AR_FILTER_MIN_LAYERS){
                        objCrop         = enlargeRectangle(sourceFrame, objCrop, AR_FILTER_CROP_X);
                        obj_detected    = true;
                        addCrop();
                    }
                }
            }
            //VERBOSE_PRINT("Found %i cropAreas, processing %i of %i pixels (%0.2f%%)\n", cropAreas.size(), pixCount, sourceFrame.rows * sourceFrame.cols, 100 * pixCount / ((float) sourceFrame.rows * sourceFrame.cols));
            break;
        }
        }
    }
    return obj_detected;
}

bool objCont_add( double minDist, double maxDist){
    if(layerDepth > AR_FILTER_MIN_LAYERS && objCont_size > AR_FILTER_MIN_POINTS){
        vector<Point> objCont;
        if(addContour(objCont, (uint16_t) 0, (uint16_t) 0, minDist, maxDist)){
            return true;
        }
    }
    objCont_size = 0;
    return false;
}

void objCont_addPoint(uint16_t* row, uint16_t* col){
    objCont_store[objCont_size] = Point(*col,*row);
    objCont_size++;
}

bool pixTest(uint8_t *Y, uint8_t *U, uint8_t *V, uint8_t *prevDir){
    if(*V > (*U + AR_FILTER_GREY_THRES) && *Y >= AR_FILTER_Y_MIN && *Y <= AR_FILTER_Y_MAX && *U >= AR_FILTER_U_MIN && *U <= AR_FILTER_U_MAX && *V >= AR_FILTER_V_MIN && *V <= AR_FILTER_V_MAX){
        return true;
    }
    else{
        /*
        if(*prevDir != ARF_SEARCH){
            if(abs(cmpY - *Y) <= AR_FILTER_CDIST_YTHRES && abs(cmpU - *U) <= AR_FILTER_CDIST_UTHRES && abs(cmpV - *V) <= AR_FILTER_CDIST_VTHRES){
                PRINT("(cmpY: %d cmpU: %d cmpV: %d) (Y: %d U: %d V: %d) (dY: %d  dU: %d  dV: %d)\n", cmpY, cmpU, cmpV, *Y, *U, *V,(cmpY - *Y),(cmpU - *U),(cmpV - *V));
                return true;
            }
            else{
                return false;
            }
        }
        else{
        */
            return false;
        //}
    }
}

int pixFindContour_cw(Mat& sourceFrame, Mat& destFrame, uint16_t row, uint16_t col, uint8_t prevDir, bool cascade){
    layerDepth++;
    pixCount++;
    if(destFrame.at<uint8_t>(row, col) >= 75){
            pixDupCount++;
            return ARF_DUPLICATE;
    }
    uint8_t U, Y, V;
    getYUVColours(sourceFrame, row, col, &Y, &U, &V);
    if(pixTest(&Y, &U, &V, &prevDir)){
        destFrame.at<uint8_t>(row, col) = 75;
        if(cascade){
            uint8_t nextDirCnt, nextDir[6];
            uint16_t newRow, newCol;
            bool success = false;
            uint8_t d = 0, edge = 0;
            getNextDirection_cw(prevDir, nextDir, &nextDirCnt);
            while(layerDepth < AR_FILTER_MAX_LAYERS && d < nextDirCnt && success == false){
                cmpY                = Y;
                cmpU                = U;
                cmpV                = V;
                newRow              = row;
                newCol              = col;
                if(getNewPosition(nextDir[d], &newRow, &newCol, &sourceFrame.rows, &sourceFrame.cols)){
                    int result;
                    if(d == 0){
                        result = pixFindContour_cw(sourceFrame, destFrame, newRow, newCol, nextDir[d], true);
                    }
                    else{
                        destFrame.at<uint8_t>(row, col) = 76;
                        //objCont_sCol                    = row;
                        //objCont_sRow                    = col;
                        objCont_size                    = 0;
                        result                          = pixFollowContour_cw(sourceFrame, destFrame, newRow, newCol, nextDir[d], true);
                    }
                    switch(result){ // Catch the proper response for the tested pixel
                    case ARF_FINISHED : {
                        pixSrcCount++;
                        return ARF_FINISHED;
                        break;
                    }
                    case ARF_SUCCESS : {
                        pixSucCount++;
                        return ARF_SUCCESS;
                        break;
                    }
                    case ARF_NO_FOUND : {
                        edge++;
                        break;
                    }
                    case ARF_DUPLICATE : {
                        pixDupCount++;
                        //destFrame.at<uint8_t>(row, col) = 0;
                        return ARF_DUPLICATE;
                        break;
                    }
                    case ARF_ERROR : {
                        if(layerDepth > AR_FILTER_MAX_LAYERS){
                            //destFrame.at<uint8_t>(row, col) = 0;
                            return ARF_FINISHED;
                        }
                        else{
                            edge++;
                        }
                        break;
                    }
                    default : {
                        //destFrame.at<uint8_t>(row, col) = 0;
                        return ARF_ERROR;
                    }
                    }
                }
                else{
                    edge++;
                }
                d++;
            }
            pixNofCount++;
            //destFrame.at<uint8_t>(row, col) = 0;
            return ARF_NO_FOUND; // Dead end
        }
        else{
            pixSucCount++;
            return ARF_SUCCESS;
        }
    }
    else{
        pixNofCount++;
        return ARF_NO_FOUND;
    }
}

int pixFollowContour_cw(Mat& sourceFrame, Mat& destFrame, uint16_t row, uint16_t col, uint8_t prevDir, bool cascade){
    layerDepth++;
    pixCount++;
    if(destFrame.at<uint8_t>(row, col) == 76){
        // This is my starting position, finished!
        if(layerDepth > AR_FILTER_MIN_LAYERS){
            destFrame.at<uint8_t>(row, col) = 255;
            objCont_addPoint(&row,&col);
            VERBOSE_PRINT("ARF_FINISHED back at (%d, %d) near startpos (%d, %d) after %d pixels\n",row, col, objCont_sRow, objCont_sCol, layerDepth);
            pixSucCount++;
            return ARF_FINISHED;
        }
        else{
            VERBOSE_PRINT("ARF_NO_FOUND back at (%d, %d) near startpos (%d, %d) after only %d pixels\n",row, col, objCont_sRow, objCont_sCol, layerDepth);
            pixNofCount++;
            objCont_size = 0;
            return ARF_NO_FOUND; // TODO: Should this be ARF_NO_FOUND?
        }

    }
    if(destFrame.at<uint8_t>(row, col) >= 75){
        // I've already searched this pixel before
        pixDupCount++;
        return ARF_DUPLICATE;
    }
    uint8_t U, Y, V;
    getYUVColours(sourceFrame, row, col, &Y, &U, &V);
    if(pixTest(&Y, &U, &V, &prevDir)){
        destFrame.at<uint8_t>(row, col) = 255;
        uint8_t nextDirCnt, nextDir[6];
        uint16_t newRow, newCol;
        bool success = false;
        uint8_t d = 0, edge = 0;
        getNextDirection_cw(prevDir, nextDir, &nextDirCnt);
        while(layerDepth < AR_FILTER_MAX_LAYERS && d < nextDirCnt && success == false){
            cmpY                = Y;
            cmpU                = U;
            cmpV                = V;
            newRow              = row;
            newCol              = col;
            if(getNewPosition(nextDir[d], &newRow, &newCol, &sourceFrame.rows, &sourceFrame.cols)){
                switch(pixFollowContour_cw(sourceFrame, destFrame, newRow, newCol, nextDir[d], true)){ // Catch the proper response for the tested pixel
                case ARF_FINISHED : {
                    if(prevDir != nextDir[d]){
                        objCont_addPoint(&row,&col);
                    }
#if AR_FILTER_MARK_CONTOURS
                    sourceFrame.at<Vec2b>(row, col)[0] = 0;
                    sourceFrame.at<Vec2b>(row, col)[1] = 255;
#endif
                    pixSucCount++;
                    return ARF_FINISHED;
                    break;
                }
                case ARF_SUCCESS : {
                    pixSucCount++;
                    return ARF_SUCCESS;
                    break;
                }
                case ARF_NO_FOUND : {
                    edge++;
                    break;
                }
                case ARF_DUPLICATE : {
                    pixDupCount++;
                    destFrame.at<uint8_t>(row, col) = 0;
                    return ARF_DUPLICATE;
                    break;
                }
                case ARF_ERROR : {
                    if(layerDepth > AR_FILTER_MAX_LAYERS){
                        destFrame.at<uint8_t>(row, col) = 0;
                        return ARF_FINISHED;
                    }
                    else{
                        edge++;
                    }
                    break;
                }
                default : {
                    destFrame.at<uint8_t>(row, col) = 0;
                    return ARF_ERROR;
                }
                }
            }
            else{
                edge++;
            }
            d++;
        }
        pixNofCount++;
        destFrame.at<uint8_t>(row, col) = 0;
        return ARF_NO_FOUND; // Dead end
    }
    else{
        pixNofCount++;
        return ARF_NO_FOUND;
    }
}

int pixFindContour_omni(Mat& sourceFrame, Mat& destFrame, uint16_t row, uint16_t col, uint8_t prevDir, bool cascade){
    layerDepth++;
    pixCount++;
    if(prevDir == ARF_SEARCH){
        prevDir = ARF_UP;
    }
    if(destFrame.at<uint8_t>(row, col) == 255){
        pixDupCount++;
        return ARF_DUPLICATE;
    }
    uint8_t U, Y, V;
    getYUVColours(sourceFrame, row, col, &Y, &U, &V);
    if(pixTest(&Y, &U, &V, &prevDir))
    {
        if(prevDir != ARF_SEARCH)
        {
            destFrame.at<uint8_t>(row, col) = 255;
        }
        if(cascade)
        {
            pixSucCount++;
            uint8_t nextDir[3];
            uint8_t nextDirCnt  = 3;
            getNextDirection_omni(prevDir, nextDir, &nextDirCnt);
            bool success        = false;
            uint16_t d          = 0;
            uint16_t newRow, newCol;
            int res[4]          = {ARF_NO_FOUND,ARF_NO_FOUND,ARF_NO_FOUND,ARF_NO_FOUND};
            while(d < nextDirCnt && success == false)
            {
                newRow              = row;
                newCol              = col;
                if(getNewPosition(nextDir[d], &newRow, &newCol, &sourceFrame.rows, &sourceFrame.cols)){
                    res[d]              = pixFindContour_omni(sourceFrame, destFrame, newRow, newCol, nextDir[d], true);
                }
                else{
                    res[d]              = ARF_NO_FOUND;
                }
                d++;
            }
            if(res[0] <= ARF_NO_FOUND && res[1] <= ARF_NO_FOUND && res[2] <= ARF_NO_FOUND && res[3] <= ARF_NO_FOUND)
            {
                return ARF_FINISHED;
            }
            objCrop.width       = max<uint16_t>(objCrop.x + objCrop.width,  col) - min<uint16_t>(objCrop.x, col);
            objCrop.height      = max<uint16_t>(objCrop.y + objCrop.height, row) - min<uint16_t>(objCrop.y, row);
            objCrop.x           = min<uint16_t>(objCrop.x, col);
            objCrop.y           = min<uint16_t>(objCrop.y, row);
            return ARF_FINISHED;
        }else{
            pixSucCount++;
            return ARF_SUCCESS;
        }
    }else{
        pixNofCount++;
        return ARF_NO_FOUND;
    }
}

void getYUVColours(Mat& sourceFrame, uint16_t row, uint16_t col, uint8_t* Y, uint8_t* U, uint8_t* V){
    if (((col & 1) == 0 && (cropCol & 1) == 0) || ((col & 1) == 1 && (cropCol & 1) == 1))
    {
        // Even col number
        *U = sourceFrame.at<Vec2b>(row, col)[0]; // U1
        *Y = sourceFrame.at<Vec2b>(row, col)[1]; // Y1
        if(col + 1 < sourceFrame.cols)
        {
            *V = sourceFrame.at<Vec2b>(row, col + 1)[0]; // V2
        }else{
            *V = sourceFrame.at<Vec2b>(row, col - 1)[0]; // V2
        }
    }else{
        // Uneven col number
        *V = sourceFrame.at<Vec2b>(row, col)[0]; // V2
        *Y  = sourceFrame.at<Vec2b>(row, col)[1]; // Y2
        if(col > 0)
        {
            *U = sourceFrame.at<Vec2b>(row, col - 1)[0]; // U1
        }else{
            *U = sourceFrame.at<Vec2b>(row, col + 1)[0]; // U1
        }
    }
}

void getNextDirection_cw(uint8_t prevDir, uint8_t* nextDir, uint8_t* nextDirCnt){
	*nextDirCnt = 5;
	switch(prevDir) // Find out which directions to try next
	{
	default:
	case ARF_SEARCH :
	    nextDir[0] = ARF_SEARCH;
	    nextDir[1] = ARF_UP_RIGHT;
	    nextDir[2] = ARF_RIGHT;
	    nextDir[3] = ARF_RIGHT_DOWN;
	    *nextDirCnt = 4;
	    break;
	case ARF_UP :
	    //nextDir[0] = ARF_LEFT;
	    nextDir[0] = ARF_LEFT_UP;
	    nextDir[1] = ARF_UP;
	    nextDir[2] = ARF_UP_RIGHT;
	    nextDir[3] = ARF_RIGHT;
	    nextDir[4] = ARF_RIGHT_DOWN;
	    break;
	case ARF_UP_RIGHT :
	    nextDir[0] = ARF_LEFT_UP;
	    nextDir[1] = ARF_UP;
	    nextDir[2] = ARF_UP_RIGHT;
	    nextDir[3] = ARF_RIGHT;
	    nextDir[4] = ARF_RIGHT_DOWN;
	    nextDir[5] = ARF_DOWN;
	    *nextDirCnt = 6;
	    break;
	case ARF_RIGHT :
	    //nextDir[0] = ARF_UP;
	    nextDir[0] = ARF_UP_RIGHT;
	    nextDir[1] = ARF_RIGHT;
	    nextDir[2] = ARF_RIGHT_DOWN;
	    nextDir[3] = ARF_DOWN;
	    nextDir[4] = ARF_DOWN_LEFT;
	    break;
	case ARF_RIGHT_DOWN :
	    nextDir[0] = ARF_UP_RIGHT;
	    nextDir[1] = ARF_RIGHT;
	    nextDir[2] = ARF_RIGHT_DOWN;
	    nextDir[3] = ARF_DOWN;
	    nextDir[4] = ARF_DOWN_LEFT;
	    nextDir[5] = ARF_LEFT;
	    *nextDirCnt = 6;
	    break;
	case ARF_DOWN :
	    //nextDir[0] = ARF_RIGHT;
	    nextDir[0] = ARF_RIGHT_DOWN;
	    nextDir[1] = ARF_DOWN;
	    nextDir[2] = ARF_DOWN_LEFT;
	    nextDir[3] = ARF_LEFT;
	    nextDir[4] = ARF_LEFT_UP;
	    break;
	case ARF_DOWN_LEFT :
	    nextDir[0] = ARF_RIGHT_DOWN;
	    nextDir[1] = ARF_DOWN;
	    nextDir[2] = ARF_DOWN_LEFT;
	    nextDir[3] = ARF_LEFT;
	    nextDir[4] = ARF_LEFT_UP;
	    nextDir[5] = ARF_UP;
	    *nextDirCnt = 6;
	    break;
	case ARF_LEFT :
	    //nextDir[0] = ARF_DOWN;
	    nextDir[0] = ARF_DOWN_LEFT;
	    nextDir[1] = ARF_LEFT;
	    nextDir[2] = ARF_LEFT_UP;
	    nextDir[3] = ARF_UP;
	    nextDir[4] = ARF_UP_RIGHT;
	    break;
	case ARF_LEFT_UP :
	    nextDir[0] = ARF_DOWN_LEFT;
	    nextDir[1] = ARF_LEFT;
	    nextDir[2] = ARF_LEFT_UP;
	    nextDir[3] = ARF_UP;
	    nextDir[4] = ARF_UP_RIGHT;
	    nextDir[5] = ARF_RIGHT;
	    *nextDirCnt = 6;
	    break;
	}
	return;
}

void getNextDirection_omni(uint8_t prevDir, uint8_t* nextDir, uint8_t* nextDirCnt){
   switch(prevDir)
    {
    case ARF_UP :
        nextDir[0] = ARF_LEFT;
        nextDir[1] = ARF_UP;
        nextDir[2] = ARF_RIGHT;
        *nextDirCnt = 3;
        break;
    case ARF_RIGHT :
        nextDir[0] = ARF_UP;
        nextDir[1] = ARF_RIGHT;
        nextDir[2] = ARF_DOWN;
        *nextDirCnt = 3;
        break;
    case ARF_DOWN :
        nextDir[0] = ARF_RIGHT;
        nextDir[1] = ARF_DOWN;
        nextDir[2] = ARF_LEFT;
        *nextDirCnt = 3;
        break;
    case ARF_LEFT :
        nextDir[0] = ARF_DOWN;
        nextDir[1] = ARF_LEFT;
        nextDir[2] = ARF_UP;
        *nextDirCnt = 3;
        break;
    }
    return;
}

bool getNewPosition(uint8_t nextDir, uint16_t* newRow, uint16_t* newCol, int* maxRow, int* maxCol){
	switch(nextDir) // Set the location of the next pixel to test
	{
	case ARF_SEARCH :
	    if(*newRow > 0){
	        *newRow += -1;
	    }
	    else{
	        return false;
	    }
	    break;
	case ARF_UP :
	    if(*newRow > 0){
	        *newRow += -1;
	    }
	    else{
            return false;
        }
	    break;
	case ARF_UP_RIGHT :
	    if(*newRow > 0){
	        *newRow += -1;
	    }
	    else{
	        return false;
	    }
	    if(*newCol < *maxCol - 1)
	    {
	        *newCol += 1;
	    }
	    else{
	        return false;
	    }
	    break;
	case ARF_RIGHT :
	    if(*newCol < *maxCol - 1){
	        *newCol += 1;
	    }
	    else{
	        return false;
	    }
	    break;
	case ARF_RIGHT_DOWN :
	    if(*newRow < *maxRow - 1){
	        *newRow += 1;
	    }
	    else{
	        return false;
	    }
	    if(*newCol < *maxCol - 1){
	        *newCol += 1;
	    }
	    else{
	        return false;
	    }
	    break;
	case ARF_DOWN :
	    if(*newRow < *maxRow - 1){
	        *newRow += 1;
	    }
	    else{
	        return false;
	    }
		break;
	case ARF_DOWN_LEFT :
	    if(*newRow < *maxRow - 1){
	        *newRow += 1;
	    }
	    else{
	        return false;
	    }
		if(*newCol > 0){
		    *newCol += -1;
		}
		else{
		    return false;
		}
		break;
	case ARF_LEFT :
	    if(*newCol > 0){
	        *newCol += -1;
	    }
	    else{
	        return false;
	    }
	    break;
	case ARF_LEFT_UP :
	    if(*newRow > 0){
	        *newRow += -1;
	    }
	    else{
	        return false;
	    }
	    if(*newCol > 0){
	        *newCol += -1;
	    }
	    else{
	        return false;
	    }
	    break;
	default:
	    VERBOSE_PRINT("[AR_FILTER-ERR] Invalid next-dir: %i\n", nextDir);
		return false;
		break;
	}
	return true;
}

#if AR_FILTER_MOD_VIDEO
void mod_video(Mat& sourceFrame, Mat& frameGrey){
	char text[200];
#if AR_FILTER_MEASURE_FPS
	sprintf(text,"%5.2f %5.d %8.2fs", AR_FILTER_FPS,(runCount - AR_FILTER_TIMEOUT), curT / 1000000.0);
#else
	sprintf(text,"frame %i", runCount);
#endif // AR_FILTER_MEASURE_FPS
	if(AR_FILTER_FLOOD_STYLE != AR_FILTER_FLOOD_CW){
#if AR_FILTER_DRAW_BOXES
		for(unsigned int r=0; r < cropAreas.size(); r++)
		{
			if(cropAreas[r].x != 0 && cropAreas[r].width != 0)
			{
				vector<Mat> channels;
				Mat thr_frame(cropAreas[r].height, cropAreas[r].width, CV_8UC2, cvScalar(0.0,0.0));
				Mat emptyCH(cropAreas[r].height, cropAreas[r].width, CV_8UC1, cvScalar(127.0));
				channels.push_back(emptyCH);
				channels.push_back(frameGrey(cropAreas[r]));
				merge(channels, thr_frame);
				thr_frame.copyTo(sourceFrame(cropAreas[r])); 			               // Copy threshold result to black frame
				emptyCH.release();
				thr_frame.release();
				rectangle(sourceFrame, cropAreas[r], Scalar(0,255), 2);
			}
		}
#endif //AR_FILTER_DRAW_BOXES
	}
#if AR_FILTER_DRAW_CIRCLES
	for(unsigned int r=0; r < trackRes_size; r++)         // Convert angles & Write/Print output
	{
		circle(sourceFrame,cvPoint(trackRes[r].x_p - cropCol, trackRes[r].y_p), sqrt(trackRes[r].area_p / M_PI), cvScalar(0,255), 2);
	}
#endif //AR_FILTER_DRAW_CIRCLES
	putText(sourceFrame, text, Point(10,sourceFrame.rows-40), FONT_HERSHEY_PLAIN, 1, Scalar(0,255,255), 1);
	for(unsigned int r=0; r < neighbourMem_size; r++)         // Convert angles & Write/Print output
	{
	    if(neighbourMem[r].lastSeen < runCount){
	        uint8_t tColor = (uint8_t) round(255 * (AR_FILTER_MEMORY - (runCount - neighbourMem[r].lastSeen)) / ((float) AR_FILTER_MEMORY));
	        sprintf(text,"x%5.2f", neighbourMem[r].x_w);
	        putText(sourceFrame, text, Point(neighbourMem[r].x_p - cropCol + sqrt(neighbourMem[r].area_p / M_PI) + 10, neighbourMem[r].y_p - 15), FONT_HERSHEY_PLAIN, 1, Scalar(0,tColor), 1);
	        sprintf(text,"y%5.2f", neighbourMem[r].y_w);
	        putText(sourceFrame, text, Point(neighbourMem[r].x_p - cropCol + sqrt(neighbourMem[r].area_p / M_PI) + 10, neighbourMem[r].y_p), FONT_HERSHEY_PLAIN, 1, Scalar(0,tColor), 1);
	        sprintf(text,"z%5.2f", neighbourMem[r].z_w);
	        putText(sourceFrame, text, Point(neighbourMem[r].x_p - cropCol + sqrt(neighbourMem[r].area_p / M_PI) + 10, neighbourMem[r].y_p + 15), FONT_HERSHEY_PLAIN, 1, Scalar(0,tColor), 1);

	    }
	}
	for(unsigned int r=0; r < trackRes_size; r++)         // Convert angles & Write/Print output
	{
	    sprintf(text,"x%5.2f", trackRes[r].x_w);
	    putText(sourceFrame, text, Point(trackRes[r].x_p - cropCol + sqrt(trackRes[r].area_p / M_PI) + 10, trackRes[r].y_p - 15), FONT_HERSHEY_PLAIN, 1, Scalar(0,255,255), 1);
	    sprintf(text,"y%5.2f", trackRes[r].y_w);
	    putText(sourceFrame, text, Point(trackRes[r].x_p - cropCol + sqrt(trackRes[r].area_p / M_PI) + 10, trackRes[r].y_p), FONT_HERSHEY_PLAIN, 1, Scalar(0,255,255), 1);
	    sprintf(text,"z%5.2f", trackRes[r].z_w);
	    putText(sourceFrame, text, Point(trackRes[r].x_p - cropCol + sqrt(trackRes[r].area_p / M_PI) + 10, trackRes[r].y_p + 15), FONT_HERSHEY_PLAIN, 1, Scalar(0,255,255), 1);
	}
	sprintf(text,"t:%4.1f%% o:%4.1f%%", pixCount/((float) ispHeight * ispWidth) * 100, pixSucCount/((float) pixCount) * 100);
	putText(sourceFrame, text, Point(10,sourceFrame.rows-120), FONT_HERSHEY_PLAIN, 1, Scalar(0,255,255), 1);
	sprintf(text,"d:%4.1f%% n:%4.1f%% s:%4.1f%%", pixDupCount/((float) pixCount) * 100, pixNofCount/((float) pixCount) * 100, pixSrcCount/((float) pixCount) * 100);
	putText(sourceFrame, text, Point(10,sourceFrame.rows-80), FONT_HERSHEY_PLAIN, 1, Scalar(0,255,255), 1);

	//line(sourceFrame, Point(0,0), Point(0, sourceFrame.rows-1), Scalar(0,255), 2);
	//line(sourceFrame, Point(sourceFrame.cols - 1,0), Point(sourceFrame.cols - 1, sourceFrame.rows-1), Scalar(0,255), 2);
	return;
}
#endif // AR_FILTER_MOD_VIDEO

void active_random_filter_header(Mat& sourceFrame){
#if AR_FILTER_MEASURE_FPS
    clock_gettime(CLOCK_MONOTONIC, &time_now);
    curT            = sys_time_elapsed_us(&time_init, &time_now);
    uint32_t dt_us  = sys_time_elapsed_us(&time_prev, &time_now);
    AR_FILTER_FPS   = 0.975 * AR_FILTER_FPS + 0.025 * 1000000.f / dt_us;
    time_prev       = time_now;
    VERBOSE_PRINT("Measured FPS: %0.2f\n", AR_FILTER_FPS);
#endif
    trackRes_clear();
#if AR_FILTER_SAVE_FRAME
    if(runCount == 25) saveBuffer(sourceFrame, "testBuffer.txt");   // (optional) save a raw UYVY frame
#endif // AR_FILTER_SAVE_FRAME
}

void active_random_filter_footer(void){
#if AR_FILTER_SHOW_MEM
    for(unsigned int r=0; r < neighbourMem_size; r++)        // Print to file & terminal
    {
        PRINT("%i - Object %d at (%0.2f m, %0.2f m, %0.2f m)\n", runCount, neighbourMem[r].id, neighbourMem[r].x_w, neighbourMem[r].y_w, neighbourMem[r].z_w);                                                        // Print to terminal
    }
    printf("\n");
#endif // AR_FILTER_SHOW_MEM
#if AR_FILTER_CALIBRATE_CAM
    if(runCount >= (AR_FILTER_TIMEOUT + 100) && runCount < (AR_FILTER_TIMEOUT + 110)) calibrateEstimation();
#endif // AR_FILTER_CALIBRATE_CAM
    VERBOSE_PRINT("pixCount: %d  (%.2f%%), pixSucCount: %d  (%.2f%%), pixDupCount: %d  (%.2f%%), pixNofCount: %d  (%.2f%%), pixSrcCount: %d  (%.2f%%)\n", pixCount, pixCount/((float) ispHeight * ispWidth) * 100, pixSucCount, pixSucCount/((float) pixCount) * 100, pixDupCount, pixDupCount/((float) pixCount) * 100, pixNofCount, pixNofCount/((float) pixCount) * 100, pixSrcCount, pixSrcCount/((float) pixCount) * 100);
    runCount++;                                                // Increase counter
}

void addCrop(void){
    for(unsigned int r=0; r < cropAreas.size(); r++)
    {
        bool overlap = false;
        if(!overlap && (inRectangle(Point(objCrop.x, objCrop.y), cropAreas[r]) || inRectangle(Point(objCrop.x + objCrop.width, objCrop.y), cropAreas[r]) || inRectangle(Point(objCrop.x + objCrop.width, objCrop.y + objCrop.height), cropAreas[r]) || inRectangle(Point(objCrop.x, objCrop.y + objCrop.height), cropAreas[r])))
        {
            overlap = true; // One of the corner points is inside the cropAreas rectangle
        }
        if(!overlap && objCrop.x >= cropAreas[r].x && (objCrop.x + objCrop.width) <= (cropAreas[r].x + cropAreas[r].width) && objCrop.y <= cropAreas[r].y && (objCrop.y + objCrop.height) >= (cropAreas[r].y + cropAreas[r].height))
        {
            overlap = true; // less wide, yet fully overlapping in height
        }
        if(!overlap && objCrop.y >= cropAreas[r].y && (objCrop.y + objCrop.height) <= (cropAreas[r].y + cropAreas[r].height) && objCrop.x <= cropAreas[r].x && (objCrop.x + objCrop.width) >= (cropAreas[r].x + cropAreas[r].width))
        {
            overlap = true; // less high, yet fully overlapping in width
        }
        if(!overlap && (inRectangle(Point(cropAreas[r].x, cropAreas[r].y), objCrop) || inRectangle(Point(cropAreas[r].x + cropAreas[r].width, cropAreas[r].y), objCrop) || inRectangle(Point(cropAreas[r].x + cropAreas[r].width, cropAreas[r].y + cropAreas[r].height), objCrop) || inRectangle(Point(cropAreas[r].x, cropAreas[r].y + cropAreas[r].height), objCrop)))
        {
            overlap = true; // One of the corner points is inside the objCrop rectangle
        }
        if(overlap == true)
        {
            objCrop.width       = max(objCrop.x + objCrop.width, cropAreas[r].x + cropAreas[r].width) - min(objCrop.x, cropAreas[r].x);
            objCrop.height      = max(objCrop.y + objCrop.height, cropAreas[r].y + cropAreas[r].height) - min(objCrop.y, cropAreas[r].y);
            objCrop.x           = min(objCrop.x, cropAreas[r].x);
            objCrop.y           = min(objCrop.y, cropAreas[r].y);
            cropAreas[r].x      = 0;
            cropAreas[r].y      = 0;
            cropAreas[r].width  = 0;
            cropAreas[r].height = 0;
        }
    }
    if(objCrop.width * objCrop.height >= AR_FILTER_MIN_CROP_AREA * ispScalar * ispScalar)
    {
        cropAreas.push_back(objCrop);
    }
    return;
}

bool inRectangle(Point pt, Rect rectangle){
    if(pt.x >= rectangle.x && pt.x <= (rectangle.x + rectangle.width) && pt.y >= rectangle.y && pt.y <= (rectangle.y + rectangle.height))
    {
        return true;
    }else{
        return false;
    }
}

Rect enlargeRectangle(Mat& sourceFrame, Rect rectangle, double scale){
    int Hincrease       = round(scale / 2 * rectangle.width);
    int Vincrease       = round(scale / 2 * rectangle.height);
    rectangle.width     = min(sourceFrame.cols - 1, rectangle.x + rectangle.width + Hincrease) - max(0, rectangle.x - Hincrease);
    rectangle.height    = min(sourceFrame.rows - 1, rectangle.y + rectangle.height + Vincrease) - max(0, rectangle.y - Vincrease);
    rectangle.x         = max(0, rectangle.x - Hincrease);
    rectangle.y         = max(0, rectangle.y - Vincrease);
    return rectangle;
}

bool trackRes_findMax( void ){
    trackRes_maxVal = 0.0;
    trackRes_maxId  = 0;
    for(uint8_t i=0; i < trackRes_size; i++){
        if(trackRes[i].r_c >= trackRes_maxVal){
            trackRes_maxVal = trackRes[i].r_c;
            trackRes_maxId  = i;
        }
    }
    return true;
}

bool trackRes_add( trackResults newRes, uint8_t overwriteId){
    if(overwriteId < trackRes_size){
        trackRes[overwriteId]           = newRes;
        trackRes_lastId                 = overwriteId;
        if(overwriteId == trackRes_maxId){
            if(trackRes_size == AR_FILTER_MAX_OBJECTS){
                trackRes_findMax();
            }
        }
    }
    else if(trackRes_size == AR_FILTER_MAX_OBJECTS){
        if(newRes.r_c < trackRes_maxVal){
            trackRes[trackRes_maxId]    = newRes;
            trackRes_lastId             = trackRes_maxId;
            trackRes_findMax();
        }
        else{
            return false;
        }
    }
    else{
        trackRes[trackRes_size]         = newRes;
        trackRes_lastId                 = trackRes_size;
        trackRes_size++;
        if(trackRes_size == AR_FILTER_MAX_OBJECTS){
            trackRes_findMax();
        }
    }
    return true;
}

bool trackRes_clear( void ){
    trackRes_size = 0;
    return true;
}

bool neighbourMem_findMax( void ){
    neighbourMem_maxVal = 0.0;
    neighbourMem_maxId  = 0;
    for(uint8_t i=0; i<neighbourMem_size; i++){
        if(neighbourMem[i].r_c >= neighbourMem_maxVal){
            neighbourMem_maxVal = neighbourMem[i].r_c;
            neighbourMem_maxId  = i;
        }
    }
    return true;
}

bool neighbourMem_add( memoryBlock newRes, uint8_t overwriteId){
    if(overwriteId < neighbourMem_size){
        neighbourMem[overwriteId]           = newRes;
        neighbourMem_lastId                 = overwriteId;
        if(overwriteId == neighbourMem_maxId){
            if(neighbourMem_size == AR_FILTER_MAX_OBJECTS){
                neighbourMem_findMax();
            }
        }
    }
    else if(neighbourMem_size == AR_FILTER_MAX_OBJECTS){
        if(newRes.r_c < neighbourMem_maxVal){
            neighbourMem[neighbourMem_maxId]    = newRes;
            neighbourMem_lastId                 = neighbourMem_maxId;
            neighbourMem_findMax();
        }
        else{
            return false;
        }
    }
    else{
        neighbourMem[neighbourMem_size]         = newRes;
        neighbourMem_lastId                     = neighbourMem_size;
        neighbourMem_size++;
        if(neighbourMem_size == AR_FILTER_MAX_OBJECTS){
            neighbourMem_findMax();
        }
    }
    return true;
}

#if AR_FILTER_CALIBRATE_CAM
void calibrateEstimation(void){
	PRINT("Starting calibration! Found %d objects\n", trackRes_size);

	vector< vector<double> > calPositions(5, vector<double>(3));
	calPositions[0][0] 	=  1.00;
	calPositions[0][1] 	=  0.00;
	calPositions[0][2] 	=  0.0;

	calPositions[1][0] 	=  1.00;
	calPositions[1][1] 	= -1.00;
	calPositions[1][2] 	=  0.0;

	calPositions[2][0] 	=  1.00;
	calPositions[2][1] 	= -2.00;
	calPositions[2][2] 	=  0.0;

	calPositions[3][0] 	=  1.00;
	calPositions[3][1] 	=  1.00;
	calPositions[3][2] 	=  0.0;

	calPositions[4][0] 	=  1.00;
	calPositions[4][1] 	=  2.00;
	calPositions[4][2] 	=  0.0;
/*
	calPositions[5][0]  =  1.00;
	calPositions[5][1]  =  1.00;
	calPositions[5][2]  =  0.10;

	calPositions[6][0]  =  1.00;
	calPositions[6][1]  =  2.00;
	calPositions[6][2]  =  0.10;
*/
	double k_opt        = 1.0;
	double k_min 		= 1.0;
	double k_max 		= 1.22;
	double k_step 		= 0.0025;

	double perspective_zCor_opt  = 0;
	double perspective_zCor_min  = 0;
	double perspective_zCor_max  = 5;
	double perspective_zCor_step = 0.025;

	//uint16_t calArea_opt    =     0;
	//uint16_t calArea_min 	=  7000;
	//uint16_t calArea_max 	= 12000;
	//uint16_t calArea_step 	=    10;

	//int orbDiag_opt   =    0;
	//int orbDiag_min 	= 1900;
	//int orbDiag_max 	= 2500;
	//int orbDiag_step 	=   10;

	double position[3];
	double ball_err = 1000;
	uint8_t tr, ball_id = 0;
	for(tr=0; tr < trackRes_size; tr++){
#if AR_FILTER_CALIBRATE_CAM == 2
	    double cur_ball_err = pow(trackRes[tr].x_w - calPositions[0][0], 2.0) + pow(trackRes[tr].y_w - calPositions[0][1], 2.0);
#else
	    double cur_ball_err = pow(trackRes[tr].x_w - calPositions[0][0], 2.0) + pow(trackRes[tr].y_w - calPositions[0][1], 2.0) + pow(trackRes[tr].z_w - calPositions[0][2], 2.0);
#endif
	    if(cur_ball_err < ball_err){
	        ball_err = cur_ball_err;
	        ball_id  = tr;
	    }
	}
	default_calArea = (uint16_t) (trackRes[ball_id].area_p / pow(ispScalar,2.0));
	printf("Calibrated area to %d based on trackRes %d (%5.2f)\n", default_calArea, ball_id, trackRes[ball_id].r_c);
	estimatePosition(trackRes[ball_id].x_p, trackRes[ball_id].y_p, trackRes[ball_id].area_p, position);
	printf("New position for trackRes %d:              (%5.2f)\n", ball_id, position[2]);

	double err, opt_err = 1000;
	int i=0, totI = (int) ( ( 1 + ceil( (k_max - k_min) / k_step ) ) * ( 1 + ceil( (perspective_zCor_max - perspective_zCor_min) / perspective_zCor_step ) ) );
	for(default_k = k_min; default_k <= k_max; default_k += k_step)
	{
	    for(perspective_zCor = perspective_zCor_min; perspective_zCor <= perspective_zCor_max; perspective_zCor += perspective_zCor_step)
	    {
	        err = 0;
	        for(unsigned int r=0; r < trackRes_size; r++)		// Convert angles & Write/Print output
	        {
	            estimatePosition(trackRes[r].x_p, trackRes[r].y_p, trackRes[r].area_p, position);
	            trackRes[r].x_c = position[0];
	            trackRes[r].y_c = position[1];
	            trackRes[r].r_c = position[2];
	            cam2body(&trackRes[r]);							// Convert from camera angles to body angles (correct for roll)
	            body2world(&trackRes[r]); 	                    // TODO: FIX FAKE EULER Convert from body angles to world coordinates (correct yaw and pitch)
	            ball_err = 1000;
	            for(unsigned int i=0; i < calPositions.size(); i++)
	            {
#if AR_FILTER_CALIBRATE_CAM == 2
	                double cur_ball_err = pow(trackRes[r].x_w - calPositions[i][0], 2.0) + pow(trackRes[r].y_w - calPositions[i][1], 2.0);
#else
	                double cur_ball_err = pow(trackRes[r].x_w - calPositions[i][0], 2.0) + pow(trackRes[r].y_w - calPositions[i][1], 2.0) + pow(trackRes[r].z_w - calPositions[i][2], 2.0);
#endif
	                if(cur_ball_err < ball_err)
	                {
	                    ball_err = cur_ball_err;
	                }
	            }
	            err += ball_err;
	        }
	        err = err / trackRes_size;
	        if(err < opt_err)
	        {
	            opt_err 	            = err;
	            k_opt 		            = default_k;
	            perspective_zCor_opt    = perspective_zCor;
	        }
	        i++;
	        printf("\r%6.2f percent - %0.2f", 100 * i / ((double) totI), sqrt(opt_err));
	    }
	}
	printf("\n");
	default_k           = k_opt;
	perspective_zCor    = perspective_zCor_opt;
	PRINT("Calibration finished. Avg error: %0.3f.\t k=%0.3f\t perspective_zCor=%0.3f\n\n", sqrt(opt_err), k_opt, perspective_zCor_opt);
	//usleep(500000);
}
#endif

#if AR_FILTER_SAVE_FRAME
void saveBuffer(Mat sourceFrame, const char *filename)
{
	char path[100];
	sprintf(path,"/data/ftp/internal_000/%s", filename);
	FILE * iFile = fopen(path,"w");
	PRINT("Writing imagebuffer(%i x %i) to file %s  ... ", sourceFrame.rows, sourceFrame.cols, path);
	for(int row = 0; row < sourceFrame.rows; row++)
	{
		for(int col = 0; col < sourceFrame.cols; col++)
		{
			fprintf(iFile, "%i,%i", sourceFrame.at<Vec2b>(row,col)[0], sourceFrame.at<Vec2b>(row,col)[1]);
			if(col != sourceFrame.cols-1)
			{
				fprintf(iFile, ",");
			}
		}
		fprintf(iFile,"\n");
	}
	fclose(iFile);
	PRINT(" done.\n");
}
#endif
