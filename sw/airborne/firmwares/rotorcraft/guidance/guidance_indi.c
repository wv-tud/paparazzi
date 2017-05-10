/*
 * Copyright (C) 2015 Ewoud Smeur <ewoud.smeur@gmail.com>
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
 */

/**
 * @file firmwares/rotorcraft/guidance/guidance_indi.c
 *
 * A guidance mode based on Incremental Nonlinear Dynamic Inversion
 * Come to IROS2016 to learn more!
 *
 */

#include "generated/airframe.h"
#include "firmwares/rotorcraft/guidance/guidance_indi.h"
#include "subsystems/ins/ins_int.h"
#include "subsystems/radio_control.h"
#include "state.h"
#include "subsystems/imu.h"
#include "firmwares/rotorcraft/guidance/guidance_h.h"
#include "firmwares/rotorcraft/guidance/guidance_v.h"
#include "firmwares/rotorcraft/stabilization/stabilization_attitude.h"
#include "firmwares/rotorcraft/autopilot_rc_helpers.h"
#include "mcu_periph/sys_time.h"
#include "autopilot.h"
#include "stabilization/stabilization_attitude_ref_quat_int.h"
#include "firmwares/rotorcraft/stabilization.h"
#include "stdio.h"
#include "subsystems/abi.h"

// The acceleration reference is calculated with these gains. If you use GPS,
// they are probably limited by the update rate of your GPS. The default
// values are tuned for 4 Hz GPS updates. If you have high speed position updates, the
// gains can be higher, depending on the speed of the inner loop.
#ifdef GUIDANCE_INDI_POS_GAIN
float guidance_indi_pos_gain = GUIDANCE_INDI_POS_GAIN;
#else
float guidance_indi_pos_gain = 1.0;
#endif

#ifdef GUIDANCE_INDI_SPEED_GAIN
float guidance_indi_speed_gain = GUIDANCE_INDI_SPEED_GAIN;
#else
float guidance_indi_speed_gain = 2.7;
#endif

struct FloatVect3 sp_accel = {0.0,0.0,0.0};
#ifdef GUIDANCE_INDI_SPECIFIC_FORCE_GAIN
float thrust_in_specific_force_gain = GUIDANCE_INDI_SPECIFIC_FORCE_GAIN;
static void guidance_indi_filter_thrust(void);

#ifndef GUIDANCE_INDI_THRUST_DYNAMICS
#ifndef STABILIZATION_INDI_ACT_DYN_P
#error "You need to define GUIDANCE_INDI_THRUST_DYNAMICS to be able to use indi vertical control"
#else // assume that the same actuators are used for thrust as for roll (e.g. quadrotor)
#define GUIDANCE_INDI_THRUST_DYNAMICS STABILIZATION_INDI_ACT_DYN_P
#endif
#endif //GUIDANCE_INDI_THRUST_DYNAMICS

#endif //GUIDANCE_INDI_SPECIFIC_FORCE_GAIN

#ifndef GUIDANCE_INDI_FILTER_CUTOFF
#ifdef STABILIZATION_INDI_FILT_CUTOFF
#define GUIDANCE_INDI_FILTER_CUTOFF STABILIZATION_INDI_FILT_CUTOFF
#else
#define GUIDANCE_INDI_FILTER_CUTOFF 3.0
#endif
#endif

#ifndef GUIDANCE_INDI_FILTER_CUTOFF_P
#ifndef STABILIZATION_INDI_FILT_CUTOFF_P
#define GUIDANCE_INDI_FILTER_CUTOFF_P GUIDANCE_INDI_FILTER_CUTOFF
#else
#define GUIDANCE_INDI_FILTER_CUTOFF_P STABILIZATION_INDI_FILT_CUTOFF_P
#endif
#endif

#ifndef GUIDANCE_INDI_FILTER_CUTOFF_Q
#ifndef STABILIZATION_INDI_FILT_CUTOFF_Q
#define GUIDANCE_INDI_FILTER_CUTOFF_Q GUIDANCE_INDI_FILTER_CUTOFF
#else
#define GUIDANCE_INDI_FILTER_CUTOFF_Q STABILIZATION_INDI_FILT_CUTOFF_Q
#endif
#endif

#ifndef GUIDANCE_INDI_FILTER_CUTOFF_R
#ifndef STABILIZATION_INDI_FILT_CUTOFF_R
#define GUIDANCE_INDI_FILTER_CUTOFF_R GUIDANCE_INDI_FILTER_CUTOFF
#else
#define GUIDANCE_INDI_FILTER_CUTOFF_R STABILIZATION_INDI_FILT_CUTOFF_R
#endif
#endif

float thrust_act = 0;

#ifndef GUIDANCE_INDI_FILTER_ORDER
#ifndef STABILIZATION_INDI_FILTER_ORDER
#define GUIDANCE_INDI_FILTER_ORDER 2
#else
#define GUIDANCE_INDI_FILTER_ORDER STABILIZATION_INDI_FILTER_ORDER
#endif
#endif

#if GUIDANCE_INDI_FILTER_ORDER == 2
Butterworth2LowPass filt_accel_ned[3];
Butterworth2LowPass roll_filt;
Butterworth2LowPass pitch_filt;
Butterworth2LowPass thrust_filt;
#else
Butterworth4LowPass filt_accel_ned[3];
Butterworth4LowPass roll_filt;
Butterworth4LowPass pitch_filt;
Butterworth4LowPass thrust_filt;
#endif

struct FloatMat33 Ga;
struct FloatMat33 Ga_inv;
struct FloatVect3 euler_cmd;

float filter_cutoff = GUIDANCE_INDI_FILTER_CUTOFF;

struct FloatEulers guidance_euler_cmd;
float thrust_in;

static void guidance_indi_propagate_filters(void);
static void guidance_indi_calcG(struct FloatMat33 *Gmat);

/**
 *
 * Call upon entering indi guidance
 */
void guidance_indi_enter(void) {
  thrust_in = 0.0;
  thrust_act = 0;

  float tau = 1.0/(2.0*M_PI*filter_cutoff);

  float tau_p = 1.0/(2.0*M_PI*GUIDANCE_INDI_FILTER_CUTOFF_P);
  float tau_q = 1.0/(2.0*M_PI*GUIDANCE_INDI_FILTER_CUTOFF_Q);
  float tau_r = 1.0/(2.0*M_PI*GUIDANCE_INDI_FILTER_CUTOFF_R);

  float sample_time = 1.0/PERIODIC_FREQUENCY;

#if GUIDANCE_INDI_FILTER_ORDER == 2
    init_butterworth_2_low_pass(&filt_accel_ned[0], tau_p, sample_time, 0.0);
    init_butterworth_2_low_pass(&filt_accel_ned[1], tau_q, sample_time, 0.0);
    init_butterworth_2_low_pass(&filt_accel_ned[2], tau_r, sample_time, 0.0);
#else
    init_butterworth_4_low_pass(&filt_accel_ned[0], tau_p, sample_time, 0.0);
    init_butterworth_2_low_pass(&filt_accel_ned[1], tau_q, sample_time, 0.0);
    init_butterworth_2_low_pass(&filt_accel_ned[2], tau_r, sample_time, 0.0);
#endif

#if GUIDANCE_INDI_FILTER_ORDER == 2
  init_butterworth_2_low_pass(&roll_filt,   tau_p, sample_time, 0.0);
  init_butterworth_2_low_pass(&pitch_filt,  tau_q, sample_time, 0.0);
  init_butterworth_2_low_pass(&thrust_filt, tau_r, sample_time, 0.0);
#else
  init_butterworth_4_low_pass(&roll_filt,   tau_p, sample_time, 0.0);
  init_butterworth_4_low_pass(&pitch_filt,  tau_q, sample_time, 0.0);
  init_butterworth_4_low_pass(&thrust_filt, tau_r, sample_time, 0.0);
#endif
}

/**
 * @param in_flight in flight boolean
 * @param heading_sp the desired heading [rad]
 *
 * main indi guidance function
 */
void guidance_indi_run(bool in_flight, float heading_sp) {

  //filter accel to get rid of noise and filter attitude to synchronize with accel
  guidance_indi_propagate_filters();

  //Linear controller to find the acceleration setpoint from position and velocity
  float pos_x_err = POS_FLOAT_OF_BFP(guidance_h.ref.pos.x - stateGetPositionNed_i()->x);
  float pos_y_err = POS_FLOAT_OF_BFP(guidance_h.ref.pos.y - stateGetPositionNed_i()->y);
  float pos_z_err = POS_FLOAT_OF_BFP(guidance_v_z_ref - stateGetPositionNed_i()->z);

  float speed_sp_x = pos_x_err * guidance_indi_pos_gain;
  float speed_sp_y = pos_y_err * guidance_indi_pos_gain;
  float speed_sp_z = pos_z_err * guidance_indi_pos_gain;

  sp_accel.x = (speed_sp_x - stateGetSpeedNed_f()->x) * guidance_indi_speed_gain;
  sp_accel.y = (speed_sp_y - stateGetSpeedNed_f()->y) * guidance_indi_speed_gain;
  sp_accel.z = (speed_sp_z - stateGetSpeedNed_f()->z) * guidance_indi_speed_gain;

#if GUIDANCE_INDI_RC_DEBUG
#warning "GUIDANCE_INDI_RC_DEBUG lets you control the accelerations via RC, but disables autonomous flight!"
  //for rc control horizontal, rotate from body axes to NED
  float psi = stateGetNedToBodyEulers_f()->psi;
  float rc_x = -(radio_control.values[RADIO_PITCH]/9600.0)*8.0;
  float rc_y = (radio_control.values[RADIO_ROLL]/9600.0)*8.0;
  sp_accel.x = cosf(psi) * rc_x - sinf(psi) * rc_y;
  sp_accel.y = sinf(psi) * rc_x + cosf(psi) * rc_y;

  //for rc vertical control
  sp_accel.z = -(radio_control.values[RADIO_THROTTLE]-4500)*8.0/9600.0;
#endif

  //Calculate matrix of partial derivatives
  guidance_indi_calcG(&Ga);
  //Invert this matrix
  MAT33_INV(Ga_inv, Ga);

#if GUIDANCE_INDI_FILTER_ORDER == 2
  struct FloatVect3 a_diff = { sp_accel.x - filt_accel_ned[0].o[0], sp_accel.y -filt_accel_ned[1].o[0], sp_accel.z -filt_accel_ned[2].o[0]};
#else
  struct FloatVect3 a_diff = { sp_accel.x - filt_accel_ned[0].lp2.o[0], sp_accel.y -filt_accel_ned[1].lp2.o[0], sp_accel.z -filt_accel_ned[2].lp2.o[0]};
#endif

  //Bound the acceleration error so that the linearization still holds
  Bound(a_diff.x, -6.0, 6.0);
  Bound(a_diff.y, -6.0, 6.0);
  Bound(a_diff.z, -9.0, 9.0);

  //If the thrust to specific force ratio has been defined, include vertical control
  //else ignore the vertical acceleration error
#ifndef GUIDANCE_INDI_SPECIFIC_FORCE_GAIN
#ifndef STABILIZATION_ATTITUDE_INDI_FULL
  a_diff.z = 0.0;
#endif
#endif

  //Calculate roll,pitch and thrust command
  MAT33_VECT3_MUL(euler_cmd, Ga_inv, a_diff);

  AbiSendMsgTHRUST(THRUST_INCREMENT_ID, euler_cmd.z);
#if GUIDANCE_INDI_FILTER_ORDER == 2
  guidance_euler_cmd.phi = roll_filt.o[0] + euler_cmd.x;
  guidance_euler_cmd.theta = pitch_filt.o[0] + euler_cmd.y;
#else
  guidance_euler_cmd.phi = roll_filt.lp2.o[0] + euler_cmd.x;
  guidance_euler_cmd.theta = pitch_filt.lp2.o[0] + euler_cmd.y;
#endif
  //zero psi command, because a roll/pitch quat will be constructed later
  guidance_euler_cmd.psi = 0;

#ifdef GUIDANCE_INDI_SPECIFIC_FORCE_GAIN
  guidance_indi_filter_thrust();

  //Add the increment in specific force * specific_force_to_thrust_gain to the filtered thrust
#if GUIDANCE_INDI_FILTER_ORDER == 2
  thrust_in = thrust_filt.o[0] + euler_cmd.z*thrust_in_specific_force_gain;
#else
  thrust_in = thrust_filt.lp2.o[0] + euler_cmd.z*thrust_in_specific_force_gain;
#endif
  Bound(thrust_in, 0, 9600);

#if GUIDANCE_INDI_RC_DEBUG
  if(radio_control.values[RADIO_THROTTLE]<300) {
    thrust_in = 0;
  }
#endif

  //Overwrite the thrust command from guidance_v
  stabilization_cmd[COMMAND_THRUST] = thrust_in;
#endif

  //Bound euler angles to prevent flipping
  Bound(guidance_euler_cmd.phi, -GUIDANCE_H_MAX_BANK, GUIDANCE_H_MAX_BANK);
  Bound(guidance_euler_cmd.theta, -GUIDANCE_H_MAX_BANK, GUIDANCE_H_MAX_BANK);

  //set the quat setpoint with the calculated roll and pitch
  stabilization_attitude_set_setpoint_rp_quat_f(&guidance_euler_cmd, in_flight, heading_sp);
}

#ifdef GUIDANCE_INDI_SPECIFIC_FORCE_GAIN
/**
 * Filter the thrust, such that it corresponds to the filtered acceleration
 */
void guidance_indi_filter_thrust(void)
{
  // Actuator dynamics
  thrust_act = thrust_act + GUIDANCE_INDI_THRUST_DYNAMICS * (thrust_in - thrust_act);

  // same filter as for the acceleration
#if GUIDANCE_INDI_FILTER_ORDER == 2
  update_butterworth_2_low_pass(&thrust_filt, thrust_act);
#else
  update_butterworth_4_low_pass(&thrust_filt, thrust_act);
#endif
}
#endif

/**
 * Low pass the accelerometer measurements to remove noise from vibrations.
 * The roll and pitch also need to be filtered to synchronize them with the
 * acceleration
 */
void guidance_indi_propagate_filters(void) {
  struct NedCoor_f *accel = stateGetAccelNed_f();
#if GUIDANCE_INDI_FILTER_ORDER == 2
  update_butterworth_2_low_pass(&filt_accel_ned[0], accel->x);
  update_butterworth_2_low_pass(&filt_accel_ned[1], accel->y);
  update_butterworth_2_low_pass(&filt_accel_ned[2], accel->z);

  update_butterworth_2_low_pass(&roll_filt, stateGetNedToBodyEulers_f()->phi);
  update_butterworth_2_low_pass(&pitch_filt, stateGetNedToBodyEulers_f()->theta);
#else
  update_butterworth_4_low_pass(&filt_accel_ned[0], accel->x);
  update_butterworth_4_low_pass(&filt_accel_ned[1], accel->y);
  update_butterworth_4_low_pass(&filt_accel_ned[2], accel->z);

  update_butterworth_4_low_pass(&roll_filt, stateGetNedToBodyEulers_f()->phi);
  update_butterworth_4_low_pass(&pitch_filt, stateGetNedToBodyEulers_f()->theta);
#endif
}

/**
 * @param Gmat array to write the matrix to [3x3]
 *
 * Calculate the matrix of partial derivatives of the roll, pitch and thrust
 * w.r.t. the NED accelerations
 */
void guidance_indi_calcG(struct FloatMat33 *Gmat) {

  struct FloatEulers *euler = stateGetNedToBodyEulers_f();

  float sphi = sinf(euler->phi);
  float cphi = cosf(euler->phi);
  float stheta = sinf(euler->theta);
  float ctheta = cosf(euler->theta);
  float spsi = sinf(euler->psi);
  float cpsi = cosf(euler->psi);
  //minus gravity is a guesstimate of the thrust force, thrust measurement would be better
  float T = -9.81;

  RMAT_ELMT(*Gmat, 0, 0) = (cphi*spsi - sphi*cpsi*stheta)*T;
  RMAT_ELMT(*Gmat, 1, 0) = (-sphi*spsi*stheta - cpsi*cphi)*T;
  RMAT_ELMT(*Gmat, 2, 0) = -ctheta*sphi*T;
  RMAT_ELMT(*Gmat, 0, 1) = (cphi*cpsi*ctheta)*T;
  RMAT_ELMT(*Gmat, 1, 1) = (cphi*spsi*ctheta)*T;
  RMAT_ELMT(*Gmat, 2, 1) = -stheta*cphi*T;
  RMAT_ELMT(*Gmat, 0, 2) = sphi*spsi + cphi*cpsi*stheta;
  RMAT_ELMT(*Gmat, 1, 2) = cphi*spsi*stheta - cpsi*sphi;
  RMAT_ELMT(*Gmat, 2, 2) = cphi*ctheta;
}

/**
 * @param indi_rp_cmd roll/pitch command from indi guidance [rad] (float)
 * @param in_flight in flight boolean
 * @param heading the desired heading [rad] in BFP with INT32_ANGLE_FRAC
 *
 * function that creates a quaternion from a roll, pitch and yaw setpoint
 */
void stabilization_attitude_set_setpoint_rp_quat_f(struct FloatEulers* indi_rp_cmd, bool in_flight, float heading)
{
  struct FloatQuat q_rp_cmd;
  //this is a quaternion without yaw! add the desired yaw before you use it!
  float_quat_of_eulers(&q_rp_cmd, indi_rp_cmd);

  /* get current heading */
  const struct FloatVect3 zaxis = {0., 0., 1.};
  struct FloatQuat q_yaw;

  float_quat_of_axis_angle(&q_yaw, &zaxis, stateGetNedToBodyEulers_f()->psi);

  /* roll/pitch commands applied to to current heading */
  struct FloatQuat q_rp_sp;
  float_quat_comp(&q_rp_sp, &q_yaw, &q_rp_cmd);
  float_quat_normalize(&q_rp_sp);

  struct FloatQuat q_sp;

  if (in_flight) {
    /* get current heading setpoint */
    struct FloatQuat q_yaw_sp;
    float_quat_of_axis_angle(&q_yaw_sp, &zaxis, heading);


    /* rotation between current yaw and yaw setpoint */
    struct FloatQuat q_yaw_diff;
    float_quat_comp_inv(&q_yaw_diff, &q_yaw_sp, &q_yaw);

    /* compute final setpoint with yaw */
    float_quat_comp_norm_shortest(&q_sp, &q_rp_sp, &q_yaw_diff);
  } else {
    QUAT_COPY(q_sp, q_rp_sp);
  }

  QUAT_BFP_OF_REAL(stab_att_sp_quat,q_sp);
}
