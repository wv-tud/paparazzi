/*
 * Copyright (C) Wilco Vlenterie
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
 * @file "modules/computer_vision/autoswarm/autoswarm.h"
 * @author Wilco Vlenterie
 * Autonomous bebop swarming module based on vision
 */

/// Header Files ///
#include "state.h"                                              ///< Used for accessing state variables
#include "navigation.h"                                         ///< Used for navigation functions
#include "generated/flight_plan.h"                              ///< Used for WP definitions
#include "autoswarm/autoswarm.h"                                ///< Include own header file
#include "computer_vision/active_random_filter.h"               ///< Include active random filter header file

#define PRINT(string,...) fprintf(stderr, "[autoswarm->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)

#ifndef AS_VERBOSE
#define AS_VERBOSE FALSE
#endif
PRINT_CONFIG_VAR(AS_VERBOSE)

#if AS_VERBOSE
#define VERBOSE_PRINT PRINT
#else
#define VERBOSE_PRINT(...)
#endif

#ifndef AS_EXTENDED
#define AS_EXTENDED 0
#endif

#ifndef AS_GLOBAL_ATTRACTOR
#define AS_GLOBAL_ATTRACTOR AS_POINT
#endif
PRINT_CONFIG_VAR(AS_GLOBAL_ATTRACTOR)

#ifndef AS_GLOBAL_STR
#define AS_GLOBAL_STR 1.0
#endif
PRINT_CONFIG_VAR(AS_GLOBAL_STR)

#ifndef AS_SEPARATION
#define AS_SEPARATION 1.3
#endif
PRINT_CONFIG_VAR(AS_SEPARATION)

#ifndef AS_CIRCLE_RADIUS
#define AS_CIRCLE_RADIUS 1.0
#endif
PRINT_CONFIG_VAR(AS_CIRCLE_RADIUS)

#ifndef AS_VMAX
#define AS_VMAX 2.5
#endif
PRINT_CONFIG_VAR(AS_VMAX)

#ifndef AS_HEADING_MODE
#define AS_HEADING_MODE AS_CAM_GLOBAL
#endif
PRINT_CONFIG_VAR(AS_CAM_GLOBAL)

#ifndef AS_E
#define AS_E 0.0025
#endif
PRINT_CONFIG_VAR(AS_E)

#ifndef AS_EPS
#define AS_EPS 0.03
#endif
PRINT_CONFIG_VAR(AS_EPS)

#ifndef AS_HOME
#define AS_HOME 0.4
#endif
PRINT_CONFIG_VAR(AS_HOME)

#ifndef AS_LOGLO
#define AS_LOGLO 1.75
#endif
PRINT_CONFIG_VAR(AS_LOGLO)

#ifndef AS_LATTICE_RATIO
#define AS_LATTICE_RATIO 2.0
#endif
PRINT_CONFIG_VAR(AS_LATTICE_RATIO)

#ifndef AS_DEADZONE
#define AS_DEADZONE 0.25
#endif
PRINT_CONFIG_VAR(AS_DEADZONE)

#ifndef AS_YAWRATEMAX
#define AS_YAWRATEMAX 4096.0
#endif
PRINT_CONFIG_VAR(AS_YAWRATEMAX)

/// Debug options ///
#ifndef AS_PRINT_WAYPOINT
#define AS_PRINT_WAYPOINT 0                                     ///< Show the updated positions of the waypoints
#endif
PRINT_CONFIG_VAR(AS_PRINT_WAYPOINT)

#ifndef AS_WRITE_RESULTS
#define AS_WRITE_RESULTS 0                                      ///< Write measurements to text file
#endif
PRINT_CONFIG_VAR(AS_WRITE_RESULTS)

#define AS_ADHERE_TO_FP 0

#ifndef AS_ADHERE_TO_FP
#define AS_ADHERE_TO_FP 1                                       ///< Don't start autoswarm until in block "Swarm" or "Swarm Home"
#endif
#if AS_ADHERE_TO_FP
PRINT_CONFIG_VAR(AS_WRITE_RESULTS)
#else
#warning [AS] Not adhering to flightplan
#endif

#ifndef AS_IGNORE_LOCAL
#define AS_IGNORE_LOCAL 0                                       ///< Ignore local component
#endif
#if AS_IGNORE_LOCAL
PRINT_CONFIG_VAR(AS_IGNORE_LOCAL)
#warning [AS] Ignoring local component
#endif

/// Static function declarations ///
static void     calculateVelocityResponse   (struct NedCoor_f *pos, struct FloatEulers* eulerAngles, double totV[3], double gi[3]);
static void     calculateGlobalVelocity     (struct NedCoor_f *pos, double gi[3], double ci[3]);
static void     calculateLocalVelocity      (struct NedCoor_f *pos, double li[3]);
static void     calculateDiffVelocity       (double totV[3], double di[3]);
static void     calculateCamPosition        (struct NedCoor_f *pos, struct FloatEulers* eulerAngles, double totV[3], double gi[3], double cPos[3]);
static void     updateWaypoints             (struct NedCoor_f *pos, double totV[3], double cPos[3]);
static void     limitNorm                   (double vec[3], double maxNorm);
static void     limitNormHeading            (struct FloatEulers* eulerAngles, double vec[3]);
static void     autoswarm_write_log         (void);
static double   calculateLocalGlobalCoeff   (struct NedCoor_f *pos, double gi[3], double ci[3]);
static void     limitYaw                    (struct NedCoor_f *pos, struct FloatEulers* eulerAngles, double cPos[3]);

double AS_WN = 0.0;
double AS_GC = 0.0;
double AS_GN = 0.0;

struct originPoint globalOrigin;

/// Set up swarm parameters ///
int     settings_as_extended        = AS_EXTENDED;              ///< Enable Velocity Template extension
int     settings_as_heading_mode    = AS_HEADING_MODE;          ///< Heading/Camera control modus
double  settings_as_separation      = AS_SEPARATION;            ///< Separation between neighbours
double  settings_as_lattice_ratio   = AS_LATTICE_RATIO;         ///< Local interactions are bounded within lattice_ratio * separation

int     settings_ignore_local       = AS_IGNORE_LOCAL;          ///< Ignore local component

double  settings_as_e               = AS_E;                     ///< Pinciroli E coefficient
double  settings_as_eps             = AS_EPS;                   ///< Differential component
double  settings_as_loglo           = AS_LOGLO;                 ///< Local-Global interaction exponent

double  settings_as_global_strength = AS_GLOBAL_STR;            ///< Global field strength (% of V_MAX)
double  settings_as_vmax            = AS_VMAX;                  ///< Maximum goal waypoint translation

/// Set up global attractor parameters ///
int     settings_as_attractor       = AS_GLOBAL_ATTRACTOR;      ///< The type of global attractor/field
double  settings_as_circle_radius   = AS_CIRCLE_RADIUS;         ///< Radius of the circle field
double  settings_as_deadzone        = AS_DEADZONE;              ///< Deadzone near the centre of the global field to prevent spinning around the centre

double  settings_as_bucket_vmin     = 0.25;                     ///< Bucket global field minimum velocity

/// Initialize parameters to be assigned during runtime ///
static uint32_t             runCount        = 0;                ///< The current frame number
static const char *         flight_blocks[] = FP_BLOCKS;        ///< Array of flight blocks
static double               prev_v_d[3]     = {0.0, 0.0, 0.0};  ///< Previous totV used for differential component

#ifdef __linux__
double                      lastTotV[3]     = {0.0, 0.0, 0.0};  ///< Used for plotting totV on frame
//pthread_mutex_t             totV_mutex;
#endif

/// Optional variables declarations ///
#if AS_WRITE_RESULTS
static time_t               startTime;                          ///< Start time of the program
static time_t               currentTime;                        ///< Current time
static int                  curT;                               ///< Current difference between start time and current time aka runtime
FILE*                       pFile;                              ///< File handle to write to
char                        resultFile [50];                    ///< File name to write to
#endif // AS_WRITE_RESULTS

/// Function definitions ///

/** initialize autoswarm
 *
 * @param[in] initial global mode (uint8_t)
 *
 * This function sets the default global mode, cam_range and log-file
 */
void autoswarm_init( void ){
#if AS_WRITE_RESULTS
    startTime = time(0);
    tm * startTM    = localtime(&startTime);
    sprintf(resultFile, "/data/ftp/internal_000/AS_result-%d-%02d-%02d_%02d-%02d-%02d.txt", startTM->tm_year + 1900, startTM->tm_mon + 1, startTM->tm_mday, startTM->tm_hour, startTM->tm_min, startTM->tm_sec);
    pFile           = fopen(resultFile,"w");
    if (pFile == NULL){
        perror("[AS-ERROR] File error");
    }
    else{
        fprintf(pFile,"ID\tmem\ti\tt\tposX\tposY\tposZ\tobjX\tobjY\tobjZ\n");
        fclose(pFile);
        printf("[AS] Writing tracking results to: %s\n", resultFile);
    }
#endif
    printf("[AS] initialized, ar_filter cam_range is %0.2f\n",ARF_CAM_RANGE);
    return;
}

/** main function of autoswarm
 *
 * This function calculates the desired waypoints and sets the correct heading
 */
void autoswarm_run( void ){
#if AS_ADHERE_TO_FP
    if (strcmp("Swarm",flight_blocks[nav_block]) && strcmp("Swarm Home",flight_blocks[nav_block])){
        ///< Don't run if we are not in flight block "Swarm" or "Swarm Home"
        return;
    }
#endif
#if AS_WRITE_RESULTS
    currentTime     = time(0);                                              ///< Get the current time
    curT            = difftime(currentTime,startTime);                      ///< Calculate time-difference between startTime and currentTime
#endif
    struct FloatEulers* eulerAngles = stateGetNedToBodyEulers_f();          ///< Get vehicle body attitude euler angles (float).
    struct NedCoor_f* pos           = stateGetPositionNed_f();              ///< Get position in local NED coordinates (float).
    double totV[3]  = {0.0, 0.0, 0.0};                                      ///< Desired velocity
    double cPos[3]  = {0.0, 0.0, 0.0};                                      ///< Desired camera position in unit distance world coordinates
    double gi[3]    = {0.0, 0.0, 0.0};                                      ///< Global interaction vector
    calculateVelocityResponse(pos, eulerAngles, totV, gi);                  ///< Calculate the velocity response of the agent and store result in totV and gi
    calculateCamPosition(pos, eulerAngles, totV, gi, cPos);                 ///< Calculate WP_CAM/heading based on pos,totV and gi and store in cPos
    updateWaypoints(pos, totV, cPos);                                       ///< Update waypoints based on velocity contribution
    autoswarm_write_log();                                                  ///< Write to log file
    runCount++;                                                             ///< Update global run-counter
    return;
}

/** Calulates the velocity response
 *
 * @param[in] Current position (struct NedCoor_f*)
 * @param[in] Array to store desired velocity (double[3])
 * @param[in] Array to store the global components (double[3])
 *
 * This function calculates velocity response based on the local, global, and differential components
 */
void calculateVelocityResponse(struct NedCoor_f *pos, struct FloatEulers* eulerAngles, double totV[3], double gi[3]){
    double li[3]    = {0.0, 0.0, 0.0};                      ///< Local contribution
    double ci[3]    = {0.0, 0.0, 0.0};                      ///< Conflict resolution contribution
    double di[3]    = {0.0, 0.0, 0.0};                      ///< Differential contribution
    if(!settings_ignore_local){
      calculateLocalVelocity(pos, li);                        ///< Get the local contribution based on our neighbours
    }
    calculateGlobalVelocity(pos, gi, ci);                   ///< Get the contribution due to the global attractor/field
    double g_coeff  = calculateLocalGlobalCoeff(pos, gi, ci);   ///< Calculate the local-global interaction coefficient
    if (settings_as_heading_mode == AS_CAM_GLOBAL){
        limitNormHeading(eulerAngles, gi);                  ///< Limit the velocity when heading change gets larger
    }
    totV[0]         = li[0] + g_coeff * (gi[0] + ci[0]);    ///< Scale the global component with the local-global interaction coefficient and add them
    totV[1]         = li[1] + g_coeff * (gi[1] + ci[1]);
    limitNorm(totV, settings_as_vmax);                      ///< Check if the desired velocity exceeds settings_as_vmax
    calculateDiffVelocity(totV, di);                        ///< Calculate the differential component based on change in desired velocity
    totV[0]         = totV[0] + di[0];                      ///< Add differential component
    totV[1]         = totV[1] + di[1];
    if (settings_as_heading_mode!=AS_CAM_GLOBAL){
        limitNormHeading(eulerAngles, totV);                ///< Limit the velocity when heading change gets larger
    }
#ifdef __linux__
  //pthread_mutex_lock(&totV_mutex);
#endif
    lastTotV[0] = cos(-eulerAngles->psi) * totV[0] - sin(-eulerAngles->psi) * totV[1];
    lastTotV[1] = sin(-eulerAngles->psi) * totV[0] + cos(-eulerAngles->psi) * totV[1];
#ifdef __linux__
  //pthread_mutex_unlock(&totV_mutex);
#endif
    return;
}

/** Calulates the local velocity response
 *
 * @param[in] Current position (struct NedCoor_f*)
 * @param[in] Array to store local interaction velocity (double[3])
 *
 * This function calculates local velocity response based on our neighbours
 */
void calculateLocalVelocity(struct NedCoor_f *pos, double li[3]){
#ifdef __linux__
  //pthread_mutex_lock(&neighbourMem_mutex);
#endif
    for (uint8_t i=0; i < neighbourMem_size; i++){
        double r    = hypot(pos->x - neighbourMem[i].x_w, pos->y - neighbourMem[i].y_w );                                                   ///< The range to this neighbour
        if(r < settings_as_lattice_ratio * settings_as_separation){
            double li_n = 12 * settings_as_e / r * ( pow( settings_as_separation / r, 12.0 ) - pow( settings_as_separation / r, 6.0 ) );    ///< Local contribution scaling factor
            li[0]      += li_n * (pos->x - neighbourMem[i].x_w) / r;                                                                        ///< Local contribution in x
            li[1]      += li_n * (pos->y - neighbourMem[i].y_w) / r;                                                                        ///< Local contribution in y
        }
    }
    if (neighbourMem_size > 0){
        li[0] = li[0] / neighbourMem_size;                                                                                                  ///< Average the local contribution (nr. neighbours independent)
        li[1] = li[1] / neighbourMem_size;
        li[2] = 0.0;
    }
    else{
        li[0] = 0.0;
        li[1] = 0.0;
        li[2] = 0.0;
    }
#ifdef __linux__
  //pthread_mutex_unlock(&neighbourMem_mutex);
#endif
    return;
}

/** Calulates the local-global interaction coefficient
 *
 * @param[in] Current position (struct NedCoor_f*)
 * @param[in] Array containing global interaction velocity (double[3])
 * @param[out] Local-global interaction coefficient (double)
 *
 * This function calculates the local-global interaction coefficient
 */
double calculateLocalGlobalCoeff(struct NedCoor_f *pos, double gi[3], double ci[3]){
#ifdef __linux__
    //pthread_mutex_lock(&neighbourMem_mutex);
#endif
    double result = 1.0;
    if(settings_as_extended){
      if (neighbourMem_size > 0){
        double w_neighbours[2]  = {0.0, 0.0};                                                                                                           ///< weighed neighbours vector
        for (unsigned int r = 0; r < neighbourMem_size; r++){
          double range            = hypot( pos->x - neighbourMem[r].x_w, pos->y - neighbourMem[r].y_w );                                                ///< Range of neighbour
          if(range < settings_as_separation * settings_as_lattice_ratio){
            double rel_dist         = ( settings_as_lattice_ratio * settings_as_separation - range)
                / ( settings_as_separation * ( settings_as_lattice_ratio - 1 ) );                                                                       ///< Relative distance coefficient
            w_neighbours[0]        += pow( rel_dist, 2.0 ) * (neighbourMem[r].x_w - pos->x) / range;                                                  ///< Calculate neighbour contribution to weighed neighbours
            w_neighbours[1]        += pow( rel_dist, 2.0 ) * (neighbourMem[r].y_w - pos->y) / range;
          }
        }
        double gi_n             = hypot( gi[0] + ci[0], gi[1] + ci[1]);                                                                                 ///< Norm of global interaction
        w_neighbours[0] *= gi_n;                                                                                                                        ///< Scale weighed neighbours vector by global contribution
        w_neighbours[1] *= gi_n;
        double c_gi_coef = ( w_neighbours[0] * (gi[0] + ci[0]) + w_neighbours[1] * (gi[1] + ci[1]) ) / gi_n;                                            ///< dot product between weighed neighbours and global interaction, divided by dot produgt of global interaction and global interaction;
        result = pow( fmin( 1.0, fmax( 0.0, (gi_n - c_gi_coef) / gi_n ) ), settings_as_loglo );                                                         ///< Return local-global interaction coefficient
        AS_GC = result;
        AS_WN = c_gi_coef;
        AS_GN = gi_n;
      }
#ifdef __linux__
      //pthread_mutex_unlock(&neighbourMem_mutex);
#endif
    }
    return result;
}

/** Calulates the global velocity component
 *
 * @param[in] Current position (struct NedCoor_f*)
 * @param[in] Array to store global interaction velocity (double[3])
 *
 * This function calculates the global velocity component
 */
void calculateGlobalVelocity(struct NedCoor_f *pos, double gi[3], double ci[3]){
    /*
    #warning Hardcoded origin location
    setGlobalOrigin(1.0, 0.0, 0.0);
    */
    double direction = 1.0;
    switch (settings_as_attractor){
    case AS_POINT :{
       double cr     = hypot( globalOrigin.cx - pos->x, globalOrigin.cy - pos->y);
        if (cr > settings_as_deadzone){
            gi[0]               = settings_as_global_strength * settings_as_vmax * ( globalOrigin.cx - pos->x ) / cr;
            gi[1]               = settings_as_global_strength * settings_as_vmax * ( globalOrigin.cy - pos->y ) / cr;
        }
        break;
    }
    case AS_BUCKET :{
        double cr     = hypot( globalOrigin.cx - pos->x, globalOrigin.cy - pos->y);
        if (cr > settings_as_deadzone){
            double gScalar      = fmin( settings_as_global_strength * settings_as_vmax, fmax( settings_as_bucket_vmin, settings_as_global_strength * settings_as_vmax * ( 1 - 1 / ( 1 + exp( 6.0 / settings_as_circle_radius * (cr - settings_as_circle_radius ) ) ) ) ) );
            gi[0]               = gScalar * ( globalOrigin.cx - pos->x ) / cr;
            gi[1]               = gScalar * ( globalOrigin.cy - pos->y ) / cr;
        }
        break;
    }
    case AS_CIRCLE_CW :
        direction = -1.0;
    case AS_CIRCLE_CC :{
        double cr     = hypot( globalOrigin.cx - pos->x, globalOrigin.cy - pos->y);
        if (cr > settings_as_deadzone){
            double band_width_gain  = 0.5;
            double spiral_gain      = 24.0;
            double circle_angle     = direction * (80.0 + fmin(90.0, fmax(-90.0, spiral_gain * pow(settings_as_circle_radius - cr, 1.0)))) / 180.0 * M_PI;
            double circle_strength  = settings_as_global_strength * settings_as_vmax * fmin(0.9, 0.5 + band_width_gain * pow(cr - settings_as_circle_radius, 2.0));
            double xContrib         = circle_strength * (globalOrigin.cx - pos->x) / cr;
            double yContrib         = circle_strength * (globalOrigin.cy - pos->y) / cr;
            gi[0]                   = cos(circle_angle) * xContrib - sin(circle_angle) * yContrib;
            gi[1]                   = sin(circle_angle) * xContrib + cos(circle_angle) * yContrib;
        }
        break;
    }
    default : break;
    }
    if(settings_as_extended){
      // Add doublets for all neighbours to enhance conflict resolution
      double angle_gi = atan2( gi[1], gi[0] );
      double gi_n     = hypot( gi[0], gi[1] );
      double u        = 0.0;
      double v        = 0.0;
#ifdef __linux__
      //pthread_mutex_lock(&neighbourMem_mutex);
#endif
      for (uint8_t i=0; i < neighbourMem_size; i++){
        double r    = hypot(pos->x - neighbourMem[i].x_w , pos->y - neighbourMem[i].y_w);  ///< The range to this neighbour
        if(r >= 0.95 * settings_as_separation){
          double dAngle   = angle_gi - atan2(neighbourMem[i].y_w - pos->y, neighbourMem[i].x_w - pos->x);
          if (dAngle >  M_PI)     dAngle =  2.0 * M_PI - dAngle;
          if (dAngle < -M_PI)     dAngle =  2.0 * M_PI + dAngle;
          u              += gi_n * pow(settings_as_separation, 2.0) * (pow(sin(dAngle), 2.0) - pow(cos(dAngle), 2.0)) / pow( r, 2.0);
          v              += (2) * gi_n * pow(settings_as_separation, 2.0) * cos(dAngle) * sin(dAngle) / pow( r, 2.0);
        }
      }
#ifdef __linux__
      //pthread_mutex_unlock(&neighbourMem_mutex);
#endif
      ci[0]           = u * cos(angle_gi) - v * sin(angle_gi);
      ci[1]           = u * sin(angle_gi) + v * cos(angle_gi);
    }else{
      ci[0] = 0.0;
      ci[1] = 0.0;
    }
    return;
}

/** Calulates the differential velocity component
 *
 * @param[in] Current position (struct NedCoor_f*)
 * @param[in] Array to store differential interaction velocity (double[3])
 *
 * This function calculates the differential velocity component
 */
void calculateDiffVelocity(double totV[3], double di[3]){
    di[0]       = - settings_as_eps * ( totV[0] - prev_v_d[0] );
    di[1]       = - settings_as_eps * ( totV[1] - prev_v_d[1] );
    prev_v_d[0] = totV[0];
    prev_v_d[1] = totV[1];
    prev_v_d[2] = totV[2];
    return;
}

/** Calulates the desired camera position / heading
 *
 * @param[in] Current position (struct NedCoor_f*)
 * @param[in] Array containing desired velocity (double[3])
 * @param[in] Array to store desired camera position as a unit vector from the current position (double[3])
 * @param[in] Array containing global velocity component (double[3])
 *
 * This function calculates the differential velocity component
 */
void calculateCamPosition(struct NedCoor_f *pos, struct FloatEulers* eulerAngles, double totV[3], double gi[3], double cPos[3]){
    if (settings_as_heading_mode == AS_CAM_FORWARD){                     ///< Point WP_CAM on unity circle towards totV
        double velR = hypot(totV[0], totV[1]);
        if (velR > 0){
            cPos[0] = pos->x + totV[0] / velR;
            cPos[1] = pos->y + totV[1] / velR;
            cPos[2] = pos->z;
        }
        else{
            cPos[0] = pos->x + cos(eulerAngles->psi) * 1;
            cPos[1] = pos->y + sin(eulerAngles->psi) * 1;
            cPos[2] = pos->z;
        }
    }
    if (settings_as_heading_mode == AS_CAM_GLOBAL){                        ///< Point WP_CAM on unity circle towards global component
        double velR = hypot(gi[0], gi[1]);
        if (velR > 0){
            cPos[0]     = pos->x + gi[0] / velR;
            cPos[1]     = pos->y + gi[1] / velR;
            cPos[2]     = pos->z;
        }
        else{
            cPos[0]     = pos->x + cos(eulerAngles->psi) * 1;
            cPos[1]     = pos->y + sin(eulerAngles->psi) * 1;
            cPos[2]     = pos->z;
        }
    }
    limitYaw(pos, eulerAngles, cPos);                                      // Limit yaw
    return;
}

void limitYaw(struct NedCoor_f *pos, struct FloatEulers* eulerAngles, double cPos[3]){
    double cameraHeading    = atan2(cPos[1] - pos->y, cPos[0] - pos->x);
    double relHeading       = cameraHeading - eulerAngles->psi; // Z axis is defined downwards for psi so * -1 for the atan2

    if (relHeading >  M_PI)     relHeading =  2.0 * M_PI - relHeading;
    if (relHeading < -M_PI)     relHeading =  2.0 * M_PI + relHeading;

    //PRINT("relHeading: %4.2f    max: %4.2f   psi: %4.2f", relHeading * 180.0 / M_PI, AS_YAWRATEMAX / 512.0, eulerAngles->psi / M_PI * 180.0);
    if (fabs(relHeading) > AS_YAWRATEMAX  / 512.0 / 180.0 * M_PI){
        double newHeading   = eulerAngles->psi + relHeading/fabs(relHeading) * (AS_YAWRATEMAX / 512.0 / 180.0 * M_PI);
        //printf("  newHeading: %4.2f", newHeading / M_PI * 180);
        cPos[0]             = pos->x + cos(newHeading) * 1.0;
        cPos[1]             = pos->y + sin(newHeading) * 1.0;
        cPos[2]             = pos->z;
    }
    //printf("\n");
    return;
}

/** Limits the norm of the desired velocity based on relative heading change
 *
 * @param[in] Array containing desired velocity (double[3])
 *
 * This function limits the velocity when increasing yaw changes are made
 */
void limitNormHeading(struct FloatEulers* eulerAngles, double vec[3]){
    double vecHeading  = atan2(vec[1], vec[0]);                 ///< Heading of vec
    double relHeading   = vecHeading - eulerAngles->psi;        ///< Relative angle between vec and heading

    if (relHeading >  M_PI)     relHeading =  2.0 * M_PI - relHeading;
    if (relHeading < -M_PI)     relHeading =  2.0 * M_PI + relHeading;

    double vTurn = fmax(0.01, cos(relHeading)) * hypot(vec[0], vec[1]);

    vec[0] = cos(vecHeading) * vTurn;
    vec[1] = sin(vecHeading) * vTurn;
    return;
}

/** Limits the norm the input vector based on the maximum given
 *
 * @param[in] Vector (double[3])
 * @param[in] maximum norm (double)
 *
 * This function limits the input vector to a maximum given
 */
void limitNorm(double vec[3], double maxNorm){
    double cor = maxNorm / hypot(vec[0], vec[1]);      ///< Find the correction factor
    if (cor < 1.0){
        vec[0] *= cor;                       ///< vector so that ||vector|| = maxNorm
        vec[1] *= cor;
        vec[2] = 0.0;
    }
    return;
}

/** Updates the waypoints and heading based on desired velocity and camera position
 *
 * @param[in] Current position (struct NedCoor_f*)
 * @param[in] Array containing desired velocity (double[3])
 * @param[in] Array containing desired camera position (double[3])
 *
 * This function updates waypoints and heading according to the given desired velocity and camera position
 */
void updateWaypoints(struct NedCoor_f *pos, double totV[3], double cPos[3]){
    /*
     *  BE CAREFUL! waypoint_set_xy_i is in ENU so N(x) and E(y) axis are changed compared to x_w and y_w
     */
    waypoint_set_xy_i((unsigned char) WP__GOAL, POS_BFP_OF_REAL(pos->y + totV[1]),  POS_BFP_OF_REAL(pos->x + totV[0]));     ///< Update WP_GOAL waypoint to add desired velocity to our position
    waypoint_set_xy_i((unsigned char) WP__CAM,  POS_BFP_OF_REAL(cPos[1]),           POS_BFP_OF_REAL(cPos[0]));              ///< Update WP_GOAL waypoint to add camera position to our position
    setGlobalOrigin(waypoint_get_y((unsigned char) WP_GLOBAL), waypoint_get_x((unsigned char) WP_GLOBAL), 1);               ///< Update global origin based on WP_GLOBAL location
    if (!strcmp("Swarm",flight_blocks[nav_block]) || !strcmp("Swarm Home",flight_blocks[nav_block])){
        nav_set_heading_towards_waypoint(WP__CAM);                                                                          ///< Currently in block "Swarm" or "Swarm Home" so update heading
    }
    if (AS_PRINT_WAYPOINT) PRINT("WP_CAM (%0.2f m, %0.2f m) \tWP_GOAL (%0.2f m, %0.2f m) \tWP_GLOBAL (%0.2f m, %0.2f m)\n", cPos[0], cPos[1], pos->x + totV[0], pos->y + totV[1], globalOrigin.cx, globalOrigin.cy);
}

/** Checks if position is close enough to waypoint _TD
 *
 * @param[in] Is our position close enough to waypoint _TD (bool)
 *
 * This function checks is our current position is close enough to waypoint _TD based on the AS_HOME distance
 */
bool amIhome(void){
    struct EnuCoor_f *pos = stateGetPositionEnu_f();  // Get current position
    if (hypot(pos->x - waypoint_get_x((unsigned char) WP__TD), pos->y - waypoint_get_y((unsigned char) WP__TD)) < AS_HOME){
        return true;
    }
    else{
        return false;
    }
}

/** Writes a log to the disk
 *
 * This function writes a log to the disk //TODO: fix
 */
void autoswarm_write_log(){
#if AS_WRITE_RESULTS                                                 // See if we want to write results
#ifdef __linux__
  //pthread_mutex_lock(&neighbourMem_mutex);
#endif
    pFile         = fopen("/data/ftp/internal_000/results.txt","a");        // Open file for appending
    if (pFile == NULL){
        perror("[AS-ERROR] File error");
    }
    for (unsigned int r=0; r < neighbourMem_size; r++){                // Print to file & terminal
        int memInt = 0;
        if (neighbourMem[r].lastSeen == runCount) memInt = 1;
        if (pFile != NULL) fprintf(pFile, "%i\t%i\t%i\t%i\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\t%0.2f\n", neighbourMem[r].id, memInt, runCount, curT, pos->x, pos->y, pos->z, neighbourMem[r].x_w, neighbourMem[r].y_w, neighbourMem[r].z_w); // if file writing is enabled, write to file
    }
    if (pFile != NULL) fclose(pFile);    // Close file
#ifdef __linux__
  //pthread_mutex_unlock(&neighbourMem_mutex);
#endif
#endif
    if (AS_PRINT_WAYPOINT) printf("\n"); // Separate terminal output by newline
}
