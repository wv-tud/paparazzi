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
 * @file "modules/nav/nav_rotorcraft_shake_throw.h"
 * @author W. Vlenterie
 * Shake and throw launch for rotorcraft
 */

#ifndef NAV_ROTORCRAFT_SHAKE_THROW_H
#define NAV_ROTORCRAFT_SHAKE_THROW_H

#include "std.h"

void nav_rotorcraft_shake_throw_init( void );
void nav_rotorcraft_shake_throw_run( void );
float nav_rotorcraft_shake_throw_body_accel_norm( void );
uint8_t nav_rotorcraft_shake_throw_upside_down( void );

#endif

