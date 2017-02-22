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
 * @file "modules/computer_vision//cv_bebop_camera_stabilization.c"
 * @author Wilco Vlenterie (wv-tud)
 * Active random sampling colour filter
 */

#include "bebop_camera_stabilization.h"
#include <vector>
#include <ctime>

#define BOARD_CONFIG "boards/bebop.h"               ///< Define which board

extern "C" {
    #include "boards/bebop.h"                       ///< C header used for bebop specific settings
    #include <state.h>                              ///< C header used for state functions and data
    #include <sys/time.h>                           ///< C header used for system time functions and data
    #include "mcu_periph/sys_time.h"                ///< C header used for PPRZ time functions and data
}

using namespace std;
#include <opencv2/core/core.hpp>                    ///< Load openCV
#include <opencv2/imgproc/imgproc.hpp>              ///< Load openCV image processing library
using namespace cv;


#define PRINT(string,...) fprintf(stderr, "[AR-FILTER->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

#define CAM_STAB_VERBOSE FALSE
#if CAM_STAB_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

#define xSign(x) ( ( x ) >= ( 0 ) ? ( 1 ) : ( -1 ) )

#define CAM_STAB_UNIT_TEST     0   ///< Do unit tests with random points
#define CAM_STAB_MOD_VIDEO     1   ///< Modify the frame to show relevant info
#define CAM_STAB_CROSSHAIR     1   ///< Show centre of frame with crosshair
#define CAM_STAB_MEASURE_FPS   1   ///< Measure average FPS

static void             bebop_camera_stabilization_header ( void );
static void             bebop_camera_stabilization_footer ( void );
static Rect             setISPvars          ( uint16_t width, uint16_t height );
static uint16_t         horizonPos          ( double y_orig );
static void             plotHorizon         (Mat& sourceFrameCrop);
/** Fisheye correction **/
static double 			correctRadius		( double r, double f, double k );
static double           invertRadius        ( double r, double f, double k );
/** Perspective correction **/
static void             inputCoord          ( double x_out, double y_out, double *x_in, double *y_in );
static void             outputCoord         ( double x_in, double y_in, double *x_out, double *y_out );
/** Stabilization functions **/
static void             getMVP              ( double MVP[16] );
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
/** Line drawing functions **/
static void             plotHorizontalLine  (Mat& sourceFrameCrop, double yAngle, double xResDeg);
static void             plotVerticalLine    (Mat& sourceFrameCrop, double xAngle, double yResDeg);
/** Optional functions **/
#if CAM_STAB_MOD_VIDEO
static void             mod_video           (Mat& sourceFrame);
#endif
#if CAM_STAB_CALIBRATE_CAM
static void 			calibrateEstimation (void);
#endif
#if CAM_STAB_SAVE_FRAME
static void 			saveBuffer			(Mat sourceFrame, const char *filename);
#endif
/** Set up perspective and correction parameters **/
double      CAM_STAB_VIEW_R            = 0.00113;                          ///< Perspective viewing distance
double      default_k                   = 1.2247445;                        ///< Fisheye correction factor (1.22474604174 max)
double      default_6th_o               = 0.255;                            ///< First order remaing correction factor
double      default_2nd_o               = 0.155;                            ///< Second order counter remaing correction factor
uint16_t    default_calArea             = 7650;                             ///< Area of a ball at 1m resolution on full sensor resolution
double      default_orbDiag             = 2 * CFG_MT9F002_FISHEYE_RADIUS;   ///< Diagonal size of the fisheye picture in full sensor resolution
float       angleOfView                 = 179.85;                           ///< Perspective angle of view ( < 180 )
float       near                        = 0.075;                            ///< Perspective near clipping plane
float       far                         = 1.5;                              ///< Perspective far clipping plane
/** Set up Remaining parameters **/
double 	    CAM_STAB_IMAGE_CROP_FOVY 	= 30.0 * M_PI / 180.0; 		    ///< (in Radians) FOV centered around the horizon to search for contours

double      CAM_STAB_FPS               = 17.0;
/** Initialize parameters to be assigned during runtime **/
uint16_t                    ispWidth            = 0;                    ///< Maximum width of ISP after applied scaling
uint16_t                    ispHeight           = 0;                    ///< Maximum height of ISP after applied scaling
uint16_t                    initialWidth;                               ///< Initial width of ISP after applied scaling
uint16_t                    initialHeight;                              ///< Initial height of ISP after applied scaling
uint16_t                    cropCol;                                    ///< Column from which the ISP is cropped relative to MIN
int16_t                     fillHeight;
double                      ispScalar;                                  ///< Applied scalar by the ISP
static struct FloatEulers*  eulerAngles;                                ///< Euler angles the moment the image was recored (supplied externally)
static uint16_t             runCount            = 0;

#if CAM_STAB_MEASURE_FPS
    static struct timespec      time_now;                               ///< The current time
    static struct timespec      time_prev;                              ///< The time of the previous frame
    static struct timespec      time_init;                              ///< The time the processing began (after timeout)
    static uint32_t curT;                                               ///< The time in us between time_init and time_now
#endif

static double           MVP[16];                                        ///< The model view projection matrix of the current frame

void bebop_camera_stabilization_init(void){
    ispScalar                   = mt9f002.output_scaler * 2.0/((double) mt9f002.y_odd_inc + 1.0);
    ispHeight                   = round((CFG_MT9F002_X_ADDR_MAX - CFG_MT9F002_X_ADDR_MIN) * ispScalar);
    ispWidth                    = round((CFG_MT9F002_Y_ADDR_MAX - CFG_MT9F002_Y_ADDR_MIN) * ispScalar);
    initialWidth                = mt9f002.output_width;
    initialHeight               = mt9f002.output_height;
#if CAM_STAB_MEASURE_FPS || CAM_STAB_WRITE_LOG
    clock_gettime(CLOCK_MONOTONIC, &time_prev);
    time_init = time_prev;
#endif
#if CAM_STAB_WRITE_LOG
    time_t startTime    = time(0);
    tm * startTM        = localtime(&startTime);
    sprintf(arf_FileName, "/data/ftp/internal_000/ARF_result-%d-%02d-%02d_%02d-%02d-%02d.txt", startTM->tm_year + 1900, startTM->tm_mon + 1, startTM->tm_mday, startTM->tm_hour, startTM->tm_min, startTM->tm_sec);
    arf_File            = fopen(arf_FileName,"w");
    if (arf_File == NULL){
        perror("[AS-ERROR] File error");
    }
    else{
        fprintf(arf_File,"NR\tUS\tID\tMEM\tPOSX\tPOSY\tPOSZ\tPSI\tOBJX\tOBJY\tOBJZ\n");
        PRINT("Writing tracking results to: %s\n", arf_FileName);
    }
#endif
}

void bebop_camera_stabilization(char* buff, uint16_t width, uint16_t height, struct FloatEulers* curEulerAngles){
    eulerAngles = curEulerAngles;
    Mat sourceFrame (height, width, CV_8UC2, buff);                 // Initialize current frame in openCV (UYVY) 2 channel
    bebop_camera_stabilization_header();                                  // Mostly printing and storing
    Rect crop 	        = setISPvars( width, height); 	            // Calculate ISP related parameters

    Mat sourceFrameCrop = sourceFrame(crop); 				                // Crop the frame

#if CAM_STAB_MOD_VIDEO
	mod_video(sourceFrameCrop);                              // Modify the sourceframesourceFrame.cols-1
#endif // CAM_STAB_MOD_VIDEO
#if CAM_STAB_CROSSHAIR
	//circle(sourceFrame,Point(ispHeight/2 - cropCol + crop.x, ispWidth/2), CFG_MT9F002_FISHEYE_RADIUS * ispScalar, cvScalar(0,255), 1);
	plotHorizon(sourceFrameCrop);
#endif
	sourceFrameCrop.release();
	sourceFrame.release();                                          // Release Mat
	bebop_camera_stabilization_footer();
	return;
}

Rect setISPvars( uint16_t width, uint16_t height){
    double x1, y1, left[2], center[2], right[2];
    // This function computes the cropping according to the desires FOV Y and the current euler angles
    getMVP(MVP);
    uint16_t horizPos             = horizonPos(0.0);
    // Top
    angles2point(-75.0/180.0*M_PI, 0.5 * CAM_STAB_IMAGE_CROP_FOVY, &x1, &y1);
    point2pixel(x1,y1,&left[0],&left[1]);
    angles2point(0.0, 0.5 * CAM_STAB_IMAGE_CROP_FOVY, &x1, &y1);
    point2pixel(x1,y1,&center[0],&center[1]);
    angles2point(75.0/180.0*M_PI, 0.5 * CAM_STAB_IMAGE_CROP_FOVY, &x1, &y1);
    point2pixel(x1,y1,&right[0],&right[1]);
    uint16_t top        = (uint16_t) round( max( right[1], max( left[1], center[1])));
    // Bottom
    angles2point(-75.0/180.0*M_PI, -0.5 * CAM_STAB_IMAGE_CROP_FOVY, &x1, &y1);
    point2pixel(x1,y1,&left[0],&left[1]);
    angles2point(0.0, -0.5 * CAM_STAB_IMAGE_CROP_FOVY, &x1, &y1);
    point2pixel(x1,y1,&center[0],&center[1]);
    angles2point(75.0/180.0*M_PI, -0.5 * CAM_STAB_IMAGE_CROP_FOVY, &x1, &y1);
    point2pixel(x1,y1,&right[0],&right[1]);
    int16_t desOffset   = (uint16_t) round( min( right[1], min( left[1], center[1])));

    //PRINT("t: %d, h: %d, b: %d\n",top, horizPos, desOffset);

    uint16_t desHeight  = (top - desOffset);
    fillHeight          = (int16_t) round( 0.5*initialWidth - (horizPos - desOffset) );
    desOffset                  -= fillHeight;
    desHeight                  += fillHeight;
    if(desOffset < -MT9F002_INITIAL_OFFSET_X){
            desOffset = -MT9F002_INITIAL_OFFSET_X;
    }
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
    cropCol                     = desOffset + fillHeight;
    mt9f002.offset_x            = MT9F002_INITIAL_OFFSET_X + desOffset / ispScalar;
    mt9f002.output_width        = desHeight;
    mt9f002.sensor_width        = desHeight / ispScalar;
    Rect crop;
    if(fillHeight >= 0 && desHeight <= width && (desHeight - fillHeight) > 0){
        crop                   = cvRect(fillHeight, 0, desHeight - fillHeight, height);
    }
    else{
        crop                   = cvRect(0, 0, width, height);
    }
    mt9f002_update_resolution(&mt9f002);
    return crop;
}

void pixel2point(double x_in, double y_in, double *x_out, double *y_out){
    double f, r, theta, corR, maxR;
    f       = default_orbDiag * ispScalar / (4 * sin(M_PI / 4));
    x_in   -= ispWidth * 0.5;
    y_in   -= ispHeight * 0.5;
    r       = sqrt( pow(x_in, 2.0) + pow(y_in, 2.0) );
    theta   = atan2( y_in, x_in );
    // Approximation of 2nd order inversion
    double r_start, r_end;
    r_start = r;
    r_end   = r_start * 1.0 / (1.0 - pow(r, 1.0) * default_6th_o / pow(CFG_MT9F002_FISHEYE_RADIUS * ispScalar, 1.0) + pow(r, 2.0) * default_2nd_o / pow(CFG_MT9F002_FISHEYE_RADIUS * ispScalar, 2.0));
    r_end   = r_start * 1.0 / (1.0 - pow(r_end, 1.0) * default_6th_o / pow(CFG_MT9F002_FISHEYE_RADIUS * ispScalar, 1.0) + pow(r_end, 2.0) * default_2nd_o / pow(CFG_MT9F002_FISHEYE_RADIUS * ispScalar, 2.0));
    r_end   = r_start * 1.0 / (1.0 - pow(r_end, 1.0) * default_6th_o / pow(CFG_MT9F002_FISHEYE_RADIUS * ispScalar, 1.0) + pow(r_end, 2.0) * default_2nd_o / pow(CFG_MT9F002_FISHEYE_RADIUS * ispScalar, 2.0));

    corR    = correctRadius(r_end, f, default_k);
    maxR    = correctRadius(CFG_MT9F002_FISHEYE_RADIUS * ispScalar, f, default_k);
    x_in    = (corR * cos(theta)) / maxR;
    y_in    = (corR * sin(theta)) / maxR;
    outputCoord(x_in, y_in, y_out, x_out);
}

void point2pixel(double x_out, double y_out, double *x_in, double *y_in){
    double f, r, theta,corR, maxR;
    f           = default_orbDiag * ispScalar / (4 * sin(M_PI / 4));
    inputCoord(x_out, y_out, x_in, y_in);
    maxR        = correctRadius(CFG_MT9F002_FISHEYE_RADIUS * ispScalar, f, default_k);
    corR        = maxR * sqrt( pow( *x_in, 2.0 ) + pow( *y_in, 2.0 ) );
    theta       = atan2( *y_in, *x_in);
    r           = invertRadius(corR, f, default_k);

    r           = r * (1.0 - pow(r, 1.0) * default_6th_o / pow(CFG_MT9F002_FISHEYE_RADIUS * ispScalar, 1.0) + pow(r, 2.0) * default_2nd_o / pow(CFG_MT9F002_FISHEYE_RADIUS * ispScalar, 2.0));

    *x_in       = ispWidth * 0.5 + cos(theta) * (r);
    *y_in       = ispHeight * 0.5 + sin(theta) * (r);
}

void angles2point(double xAngle, double yAngle, double *x_out, double * y_out){
    *x_out      = CAM_STAB_VIEW_R * tan(xAngle);
    *y_out      = CAM_STAB_VIEW_R * tan(yAngle);
}

void point2angles(double x_out, double y_out, double *xAngle, double *yAngle){

    *xAngle     = atan(x_out / CAM_STAB_VIEW_R);
    *yAngle     = atan(y_out / CAM_STAB_VIEW_R);
}

void plotHorizon(Mat& sourceFrameCrop){
    plotHorizontalLine(sourceFrameCrop, 0.0 / 180.0 * M_PI, 5);
    //plotHorizontalLine(sourceFrameCrop, 15.0 / 180.0 * M_PI, 5);
    //plotHorizontalLine(sourceFrameCrop, -15.0 / 180.0 * M_PI, 5);
    //plotHorizontalLine(sourceFrameCrop, 30.0 / 180.0 * M_PI, 5);
    //plotHorizontalLine(sourceFrameCrop, -30.0 / 180.0 * M_PI, 5);
    plotVerticalLine(sourceFrameCrop,   0.0 / 180.0 * M_PI, 5);
    //plotVerticalLine(sourceFrameCrop,   45.0 / 180.0 * M_PI, 5);
    //plotVerticalLine(sourceFrameCrop,   -45.0 / 180.0 * M_PI, 5);
    //plotVerticalLine(sourceFrameCrop,   63.4349 / 180.0 * M_PI, 5);
    //plotVerticalLine(sourceFrameCrop,   -63.4349 / 180.0 * M_PI, 5);
    //plotPerspective(sourceFrameCrop);
}

double correctRadius(double r, double f, double k){
    // This function calculates the corrected radius for radial distortion
    // According to the article "A Generic Non-Linear Method for Fisheye Correction" by Dhane, Kutty and Bangadkar
    double frac = sin( atan( r / f ) ) * k;
    if(frac < 1.0){
        return f * tan( asin( frac ) );
    }
    else{
        return f * tan( asin( sin( atan( CFG_MT9F002_FISHEYE_RADIUS * ispScalar / f ) ) * k ) );
    }
}

double invertRadius(double r, double f, double k){
    // This function calculates the inverted radius for corrected radial distortion
    // According to the article "A Generic Non-Linear Method for Fisheye Correction" by Dhane, Kutty and Bangadkar
    double frac = sin( atan( r / f ) ) / k;
    if(frac < 1.0){
        return f * tan( asin( frac ) );
    }
    else{
        return f * tan( asin( sin( atan( CFG_MT9F002_FISHEYE_RADIUS * ispScalar / f ) ) / k ) );
    }
}

uint16_t horizonPos( double y_orig ){
    double x_in, y_in, x_pos, y_pos;
    angles2point(0.0, y_orig, &x_pos, &y_pos);
    point2pixel(x_pos, y_pos, &x_in, &y_in);
    return (uint16_t) round(y_in);
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
    double modelMat[16], viewMat[16], modelviewMat[16], projectionMat[16], tmpRes[16], postRotMat[16];
    double eye[4], forward[4], up[4], center[4];
    eye[0]      = 0.0;  eye[1]      =  0.0; eye[2]      = -CAM_STAB_VIEW_R;    eye[3]      = 1.0;
    forward[0]  = 0.0;  forward[1]  =  0.0; forward[2]  =  1.0;                 forward[3]  = 1.0;
    up[0]       = 0.0;  up[1]       =  1.0; up[2]       =  0.0;                 up[3]       = 1.0;
    // Create The model matrix
    double sensorRotation[16], eyeTranslationF[16], headingRotation[16], eyeTranslationR[16], modelMat_tmp1[16], modelMat_tmp2[16];
    setRotationMat(0.0, M_PI, 0.0, sensorRotation);
    setTranslationMat(-eye[0], -eye[1], -eye[2], eyeTranslationF);
    setRotationMat(0.0, 0.0, 0.0, headingRotation);
    setTranslationMat( eye[0],  eye[1],  eye[2], eyeTranslationR);
    matrixMultiply(eyeTranslationF, sensorRotation, modelMat_tmp1);
    matrixMultiply(headingRotation, modelMat_tmp1, modelMat_tmp2);
    matrixMultiply(eyeTranslationR, modelMat_tmp2, modelMat);
    // Create the view matrix
    rotateVector(-MT9F002_THETA_OFFSET - eulerAngles->theta, 0.0, -eulerAngles->phi, forward);
    rotateVector(-MT9F002_THETA_OFFSET - eulerAngles->theta, 0.0, -eulerAngles->phi, up);
    center[0] = eye[0] + forward[0];
    center[1] = eye[1] + forward[1];
    center[2] = eye[2] + forward[2];
    center[3] = 1.0;
    view_set_lookat(viewMat, eye,  center,  up);
    setPerspectiveMat(projectionMat);
    matrixMultiply(viewMat, modelMat, modelviewMat);
    matrixMultiply(projectionMat, modelviewMat, tmpRes);
    setRotationMat(0.0, 0.0, 0.0, postRotMat);
    matrixMultiply(postRotMat, tmpRes, MVP);
}

#if CAM_STAB_MOD_VIDEO
void mod_video(Mat& sourceFrame){
	char text[200];
#if CAM_STAB_MEASURE_FPS
	sprintf(text,"%5.2f %5.d %8.2fs", CAM_STAB_FPS,(runCount), curT / 1000000.0);
#endif // CAM_STAB_MEASURE_FPS
	putText(sourceFrame, text, Point(10,sourceFrame.rows-40), FONT_HERSHEY_PLAIN, 1, Scalar(0,255,255), 1);
	if((cropCol & 1) == 0){
	    line(sourceFrame, Point(0,0), Point(0, sourceFrame.rows-1), Scalar(0,255), 1);
	}
	else{
	    line(sourceFrame, Point(1,0), Point(1, sourceFrame.rows-1), Scalar(0,255), 1);
	}
	if(((cropCol & 1) == 0 && ((sourceFrame.cols - 1) & 1) == 0) || ((cropCol & 1) == 1 && ((sourceFrame.cols - 1) & 1) == 1)){
	    line(sourceFrame, Point(sourceFrame.cols - 1,0), Point(sourceFrame.cols - 1, sourceFrame.rows-1), Scalar(0,255), 1);
	}
	else{
	    line(sourceFrame, Point(sourceFrame.cols - 2,0), Point(sourceFrame.cols - 2, sourceFrame.rows-1), Scalar(0,255), 1);
	}
	return;
}
#endif // CAM_STAB_MOD_VIDEO

void plotHorizontalLine(Mat& sourceFrameCrop, double yAngle, double xResDeg){
    double x1, y1, x1p, y1p, x2, y2, x2p, y2p;
    // Plot horizontal line
    angles2point(-85.0/180.0*M_PI, yAngle, &x1, &y1);
    point2pixel(x1, y1, &x1p, &y1p);
    char text[50];
    for(double x_angle = -85+xResDeg; x_angle<=85; x_angle+=xResDeg){
        angles2point(x_angle / 180 * M_PI, yAngle, &x2, &y2);
        point2pixel(x2, y2, &x2p, &y2p);
        if(y1p >= cropCol && y1p < ispHeight && x1p >= 0 && x1p < ispWidth &&
                y2p >= cropCol && y2p < ispHeight && x2p >= 0 && x2p < ispWidth ){
            //PRINT("angle: %0.2f - p1 (%0.2f, %0.2f) p2(%0.2f, %0.2f)\n",x_angle, x1p, y1p, x2p, y2p);
            line(sourceFrameCrop, Point(y1p - cropCol,x1p), Point(y2p - cropCol,x2p), Scalar(0,127), 1);
        }
        x1p = x2p;
        y1p = y2p;
    }
    for(double i = -80; i <= 80; i+=10){
        angles2point(i / 180.0 * M_PI, yAngle, &x1, &y1);
        point2pixel(x1, y1, &x1p, &y1p);
        if(i != 0.0){
            sprintf(text,"- %0.0f", i);
            putText(sourceFrameCrop, text, Point(y1p - cropCol -2, x1p + 5), FONT_HERSHEY_PLAIN, 1, Scalar(0,255), 1);
        }
    }
}

void plotVerticalLine(Mat& sourceFrameCrop, double xAngle, double yResDeg){
    double x1, y1, x1p, y1p, x2, y2, x2p, y2p;
    // Plot vertical line
    angles2point(xAngle, -85.0/180.0*M_PI, &x1, &y1);
    point2pixel(x1, y1, &x1p, &y1p);
    char text[50];
    for(double y_angle = (-85.0+yResDeg)/180.0*M_PI; y_angle<= 85.0/180.0*M_PI + MT9F002_THETA_OFFSET + eulerAngles->theta; y_angle+=yResDeg / 180 * M_PI){
        angles2point(xAngle, y_angle, &x2, &y2);
        point2pixel(x2, y2, &x2p, &y2p);
        if(y1p >= cropCol && y1p < ispHeight && x1p >= 0 && x1p < ispWidth &&
                y2p >= cropCol && y2p < ispHeight && x2p >= 0 && x2p < ispWidth ){
            line(sourceFrameCrop, Point(y1p - cropCol,x1p), Point(y2p - cropCol,x2p), Scalar(0,127), 1);
        }
        x1p = x2p;
        y1p = y2p;
    }
    for(double i = -75.0/180.0*M_PI; i <= 75.0/180.0*M_PI + MT9F002_THETA_OFFSET + eulerAngles->theta; i+=15.0/180.0*M_PI){
        angles2point(xAngle, i, &x1, &y1);
        point2pixel(x1, y1, &x1p, &y1p);
        if(round(i / M_PI * 180) != 0.0){
            sprintf(text,"| %0.0f", i / M_PI * 180);
            putText(sourceFrameCrop, text, Point(y1p - cropCol - 2, x1p - 2), FONT_HERSHEY_PLAIN, 1, Scalar(0,255), 1);
        }
    }
}

void plotPerspective(Mat& sourceFrameCrop){
    double x1,y1,x1p,y1p,x2p,y2p,scale = 10000.0;
    double offsetCol = -150;
    y1 = -0.5*CAM_STAB_VIEW_R;
    x1 =  0.5*CAM_STAB_VIEW_R;
    outputCoord(x1, y1, &x1p, &y1p);
    x1p *= scale;
    y1p *= scale;
    y1 =  0.5*CAM_STAB_VIEW_R;
    x1 =  0.5*CAM_STAB_VIEW_R;
    outputCoord(x1, y1, &x2p, &y2p);
    x2p *= scale;
    y2p *= scale;
    line(sourceFrameCrop, Point((y1p + sourceFrameCrop.cols/2.0) + offsetCol, (x1p + sourceFrameCrop.rows/2.0)), Point((y2p + sourceFrameCrop.cols/2.0) + offsetCol, (x2p + sourceFrameCrop.rows/2.0)), Scalar(0,127), 1);
    y1 =  0.5*CAM_STAB_VIEW_R;
    x1 = -0.5*CAM_STAB_VIEW_R;
    outputCoord(x1, y1, &x1p, &y1p);
    x1p *= scale;
    y1p *= scale;
    line(sourceFrameCrop, Point((y2p + sourceFrameCrop.cols/2.0) + offsetCol, (x2p + sourceFrameCrop.rows/2.0)), Point((y1p + sourceFrameCrop.cols/2.0) + offsetCol, (x1p + sourceFrameCrop.rows/2.0)), Scalar(0,127), 1);
    y1 = -0.5*CAM_STAB_VIEW_R;
    x1 = -0.5*CAM_STAB_VIEW_R;
    outputCoord(x1, y1, &x2p, &y2p);
    x2p *= scale;
    y2p *= scale;
    line(sourceFrameCrop, Point((y1p + sourceFrameCrop.cols/2.0) + offsetCol, (x1p + sourceFrameCrop.rows/2.0)), Point((y2p + sourceFrameCrop.cols/2.0) + offsetCol, (x2p + sourceFrameCrop.rows/2.0)), Scalar(0,127), 1);
    y1 = -0.5*CAM_STAB_VIEW_R;
    x1 =  0.5*CAM_STAB_VIEW_R;
    outputCoord(x1, y1, &x1p, &y1p);
    x1p *= scale;
    y1p *= scale;
    line(sourceFrameCrop, Point((y2p + sourceFrameCrop.cols/2.0) + offsetCol, (x2p + sourceFrameCrop.rows/2.0)), Point((y1p + sourceFrameCrop.cols/2.0) + offsetCol, (x1p + sourceFrameCrop.rows/2.0)), Scalar(0,127), 1);
}

void bebop_camera_stabilization_header( void ){
#if CAM_STAB_MEASURE_FPS
    clock_gettime(CLOCK_MONOTONIC, &time_now);
    curT            = sys_time_elapsed_us(&time_init, &time_now);
    uint32_t dt_us  = sys_time_elapsed_us(&time_prev, &time_now);
    CAM_STAB_FPS    = 0.975 * CAM_STAB_FPS + 0.025 * 1000000.f / dt_us;
    time_prev       = time_now;
    VERBOSE_PRINT("Measured FPS: %0.2f\n", CAM_STAB_FPS);
#endif
}

void bebop_camera_stabilization_footer(void){
    runCount++;
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

/*
void showMat( double mat[16] ) {
    printf("| %6.3f  %6.3f  %6.3f  %6.3f |\n", mat[0], mat[4], mat[8], mat[12]);
    printf("| %6.3f  %6.3f  %6.3f  %6.3f |\n", mat[1], mat[5], mat[9], mat[13]);
    printf("| %6.3f  %6.3f  %6.3f  %6.3f |\n", mat[2], mat[6], mat[10], mat[14]);
    printf("| %6.3f  %6.3f  %6.3f  %6.3f |\n", mat[3], mat[7], mat[11], mat[15]);
}

void showVec( double vec[4] ) {
    printf("| %6.3f  %6.3f  %6.3f  %6.3f |\n", vec[0], vec[1], vec[2], vec[3]);
}
*/
