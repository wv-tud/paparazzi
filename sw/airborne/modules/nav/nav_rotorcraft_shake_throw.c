/*
 * Copyright (C) W. Vlenterie
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
 * @file "modules/nav/nav_rotorcraft_shake_throw.c"
 * @author W. Vlenterie
 * Shake and throw launch for rotorcraft
 */

#include "modules/nav/nav_rotorcraft_shake_throw.h"
#include "state.h"
#include "subsystems/gps.h"
#include "firmwares/rotorcraft/navigation.h"
#include "firmwares/rotorcraft/stabilization.h"
#ifdef BOARD_BEBOP
#include <stdlib.h>
#endif
#include "generated/flight_plan.h"
#include "autopilot.h"

#ifndef NAV_RC_SHAKE_THROW_DEBUG_NO_FLIGHT
#define NAV_RC_SHAKE_THROW_DEBUG_NO_FLIGHT 0
#endif
PRINT_CONFIG_VAR(NAV_RC_SHAKE_THROW_DEBUG_NO_FLIGHT)

#ifndef NAV_RC_SHAKE_THROW_GOTO_BLOCK
#define NAV_RC_SHAKE_THROW_GOTO_BLOCK Standby
#endif
PRINT_CONFIG_VAR(NAV_RC_SHAKE_THROW_GOTO_BLOCK)

#ifndef NAV_RC_SHAKE_THROW_RESET_BLOCK
#define NAV_RC_SHAKE_THROW_RESET_BLOCK Landed
#endif
PRINT_CONFIG_VAR(NAV_RC_SHAKE_THROW_RESET_BLOCK)

#ifndef NAV_RC_SHAKE_THROW_LAUNCH_REQUIREMENTS
#define NAV_RC_SHAKE_THROW_LAUNCH_REQUIREMENTS GpsFixValid() && autopilot_get_mode() == AP_MODE_NAV
#endif
PRINT_CONFIG_VAR(NAV_RC_SHAKE_THROW_LAUNCH_REQUIREMENTS)

enum nav_rotorcraft_shake_throw_status_ {
    NAV_RC_ST_UNINIT,
    NAV_RC_ST_WAITING,
    NAV_RC_ST_TIMEOUT,
    NAV_RC_ST_LAUNCHED,
};
enum nav_rotorcraft_shake_throw_status_ nav_rotorcraft_shake_throw_status;

float nav_rotorcraft_shake_throw_shake_threshold = 2.0; ///< The motors are not spinning yet so low noise on sensor
float nav_rotorcraft_shake_throw_fall_threshold =  7.0; ///< The motors are spinning so more noise on sensor

float nav_rotorcraft_shake_throw_upside_z_threshold = 7.5;
float nav_rotorcraft_shake_throw_upside_xy_threshold = 2.0;

uint16_t nav_rotorcraft_shake_throw_shake_time = 32;
uint16_t nav_rotorcraft_shake_throw_fall_time = 32;

uint16_t nav_rotorcraft_shake_throw_cancel_time = 256;
uint16_t nav_rotorcraft_shake_throw_timeout_time = 512;


uint16_t shake_timer = 0;
uint16_t timeout_timer = 0;
uint16_t cancel_timer = 0;
uint16_t fall_timer = 0;

uint16_t block_goto = 0;
uint16_t block_reset = 0;

static const char* flight_blocks[] = FP_BLOCKS;        ///< Array of flight blocks

void nav_rotorcraft_shake_throw_init( void ) {
    nav_rotorcraft_shake_throw_status = NAV_RC_ST_UNINIT;
    uint8_t i;
    for(i = 0; i < NB_BLOCK; i++){
        if(!strcmp(STRINGIFY(NAV_RC_SHAKE_THROW_RESET_BLOCK),flight_blocks[i])){
            block_reset = i;
        }
        if(!strcmp(STRINGIFY(NAV_RC_SHAKE_THROW_GOTO_BLOCK),flight_blocks[i])){
            block_goto = i;
        }
    }
}

void nav_rotorcraft_shake_throw_run( void ) {
    switch(nav_rotorcraft_shake_throw_status){
    case NAV_RC_ST_WAITING :
        ///< Waiting for the shake
        if(nav_rotorcraft_shake_throw_body_accel_norm() < nav_rotorcraft_shake_throw_shake_threshold){
            if(shake_timer >= nav_rotorcraft_shake_throw_shake_time){
                if(NAV_RC_SHAKE_THROW_LAUNCH_REQUIREMENTS){
                    nav_rotorcraft_shake_throw_status++;
#ifdef BOARD_BEBOP
                    system("BLDC_Test_Bench -n -M 1");
#endif
                }
                else{
#ifdef BOARD_BEBOP
                    system("BLDC_Test_Bench -n -M 2");
#endif
                    shake_timer = 0;
                }
            }
            else{
                shake_timer++;
            }
        }
        else{
            shake_timer = 0;
        }
        break;
    case NAV_RC_ST_TIMEOUT :
        ///< Configurable timeout for arming the engines
        if(nav_rotorcraft_shake_throw_body_accel_norm() > nav_rotorcraft_shake_throw_upside_z_threshold){
            if(timeout_timer >= nav_rotorcraft_shake_throw_timeout_time){
                timeout_timer = 0;
                stabilization_cmd[COMMAND_ROLL] = 0;
                stabilization_cmd[COMMAND_PITCH] = 0;
                stabilization_cmd[COMMAND_YAW] = 0;
#if !NAV_RC_SHAKE_THROW_DEBUG_NO_FLIGHT
                autopilot.motors_on = true;
                NavResurrect();
#else
                autopilot.motors_on = false;
                NavKillThrottle();
#endif
                fall_timer = 0;
                nav_rotorcraft_shake_throw_status++;
            }
            else{
                timeout_timer++;
            }
        }
        else{
            timeout_timer = 0;
        }
        break;
    case NAV_RC_ST_LAUNCHED : {
        ///< Waiting for the throw
        if(nav_rotorcraft_shake_throw_body_accel_norm() < nav_rotorcraft_shake_throw_fall_threshold){
            if(fall_timer >= nav_rotorcraft_shake_throw_fall_time){
#if !NAV_RC_SHAKE_THROW_DEBUG_NO_FLIGHT
                NavResurrect();
                GotoBlock(block_goto);
#else
                NavKillThrottle();
                GotoBlock(block_goto);
#ifdef BOARD_BEBOP
            system("BLDC_Test_Bench -n -M -2");
#endif
#endif
            }
            else{
                fall_timer++;
            }
        }
        else{
            fall_timer = 0;
        }
        if(nav_block == block_reset){
            nav_rotorcraft_shake_throw_status = NAV_RC_ST_UNINIT; ///< We've landed, reset the status
        }
        break;
    }
    default :
        ///< Nothing to be done
        break;
    }
    /// Check for the start/cancel flip
    if(nav_rotorcraft_shake_throw_upside_down()){
        if(cancel_timer >= nav_rotorcraft_shake_throw_cancel_time){
            NavKillThrottle();
#ifdef BOARD_BEBOP
            system("BLDC_Test_Bench -n -M 1");
#endif
            nav_rotorcraft_shake_throw_status = NAV_RC_ST_WAITING;
            shake_timer = 0;
            timeout_timer = 0;
            cancel_timer = 0;
            fall_timer = 0;
        }
        else{
            cancel_timer++;
        }
    }
    else{
        cancel_timer = 0;
    }
}

float nav_rotorcraft_shake_throw_body_accel_norm( void ){
    struct Int32Vect3* body_accel = stateGetAccelBody_i();
    float n = ACCEL_FLOAT_OF_BFP( (int32_t) round( sqrtf((float) (body_accel->x * body_accel->x + body_accel->y * body_accel->y + body_accel->z * body_accel->z)) ) );
    return n;
}

uint8_t nav_rotorcraft_shake_throw_upside_down( void ){
    struct Int32Vect3* body_accel = stateGetAccelBody_i();
    if(ACCEL_FLOAT_OF_BFP( body_accel->z ) > nav_rotorcraft_shake_throw_upside_z_threshold && ACCEL_FLOAT_OF_BFP( body_accel->x ) < nav_rotorcraft_shake_throw_upside_xy_threshold && ACCEL_FLOAT_OF_BFP( body_accel->y ) < nav_rotorcraft_shake_throw_upside_xy_threshold){
        return 1;
    }
    else{
        return 0;
    }
}
