/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * Parameters for vtol attitude controller.
 *
 * @author Roman Bapst <bapstr@ethz.ch>
 */
#include <px4_defines.h>
#include <vtol_pos_control/param_define.h>
#include <systemlib/param/param.h>

 /**
 * Vtol position error gain
 *
 *
 * @min 0.0
 * @group VTOL Position Control
 */
PX4_PARAM_DEFINE_FLOAT(VTP_POS_XY_P);

 /**
 * Vtol position error gain
 *
 *
 * @min 0.0
 * @group VTOL Position Control
 */
PX4_PARAM_DEFINE_FLOAT(VTP_POS_Z_P);

 /**
 * Vtol position error gain
 *
 *
 * @min 0.0
 * @group VTOL Position Control
 */
PX4_PARAM_DEFINE_FLOAT(VTP_VEL_XY_P);

 /**
 * Vtol position error gain
 *
 *
 * @min 0.0
 * @group VTOL Position Control
 */
PX4_PARAM_DEFINE_FLOAT(VTP_VEL_Z_P);

 /**
 * Vtol position error gain
 *
 *
 * @min 0.0
 * @group VTOL Position Control
 */
PX4_PARAM_DEFINE_FLOAT(VTP_VEL_XY_D);

 /**
 * Vtol position error gain
 *
 *
 * @min 0.0
 * @group VTOL Position Control
 */
PX4_PARAM_DEFINE_FLOAT(VTP_VEL_Z_D);

/**
 * Vtol position error gain
 *
 *
 * @min 0.0
 * @group VTOL Position Control
 */
PX4_PARAM_DEFINE_FLOAT(VTP_VEL_XY_I);

/**
 * Vtol position error gain
 *
 *
 * @min 0.0
 * @group VTOL Position Control
 */
PX4_PARAM_DEFINE_FLOAT(VTP_VEL_Z_I);

/**
 * Vtol xy velocity feedforward gain
 *
 *
 * @min 0.0
 * @group VTOL Position Control
 */
PX4_PARAM_DEFINE_FLOAT(VTP_VEL_XY_FF);

/**
 * Vtol xy velocity feedforward gain
 *
 *
 * @min 0.0
 * @group VTOL Position Control
 */
PX4_PARAM_DEFINE_FLOAT(VTP_VEL_Z_FF);

/**
 * Vtol position error gain
 *
 *
 * @min 0.0
 * @group VTOL Position Control
 */
PX4_PARAM_DEFINE_FLOAT(VTP_VZ_MAX);

/**
 * Vtol acceleration during transition
 *
 *
 * @min 0.0
 * @group VTOL Position Control
 */
PX4_PARAM_DEFINE_FLOAT(VTP_ACC_TRANS);

/**
 * Vtol angle at which ok to switch to fw mode
 *
 *
 * @min 0.0
 * @group VTOL Position Control
 */
PX4_PARAM_DEFINE_FLOAT(VTP_PTCH_TRANSIT);

/**
 * Vtol airspeed at which ok to switch to fw mode
 *
 *
 * @min 0.0
 * @group VTOL Position Control
 */
PX4_PARAM_DEFINE_FLOAT(VTP_SPD_TRANSIT);

/**
 * Vtol angular acceleration during backtransition
 *
 *
 * @min 0.0
 * @group VTOL Position Control
 */
PX4_PARAM_DEFINE_FLOAT(VTP_ACC_ANG);

/**
 * Vtol scale from desired thrust acceleration to thrust signal
 *
 *
 * @min 0.0
 * @group VTOL Position Control
 */
PX4_PARAM_DEFINE_FLOAT(VTP_THRUST_SCALE);

/**
 * Vtol scale from desired thrust acceleration to thrust signal
 *
 *
 * @min 0.0
 * @group VTOL Position Control
 */
PX4_PARAM_DEFINE_FLOAT(VTP_MAN_R_MAX);

/**
 * Vtol scale from desired thrust acceleration to thrust signal
 *
 *
 * @min 0.0
 * @group VTOL Position Control
 */
PX4_PARAM_DEFINE_FLOAT(VTP_MAN_P_MAX);

/**
 * Vtol scale from desired thrust acceleration to thrust signal
 *
 *
 * @min 0.0
 * @group VTOL Position Control
 */
PX4_PARAM_DEFINE_FLOAT(VTP_MAN_Y_MAX);

/**
 * Pitch trim for fw flight
 *
 *
 * @min -2.0
 * @max 2.0
 * @group VTOL Position Control
 */
PX4_PARAM_DEFINE_FLOAT(VTP_PITCH_TRIM);

/**
 * Pitch sensitivity to control stick for fw flight
 *
 *
 * @group VTOL Position Control
 */
PX4_PARAM_DEFINE_FLOAT(VTP_PITCH_SENS);

/**
 * Target cruise speed in forward flight
 *
 *
 * @group VTOL Position Control
 */
PX4_PARAM_DEFINE_FLOAT(VTP_CRUISE_SPD);

/**
 * Yaw P gain
 *
 * Yaw proportional gain, i.e. desired angular speed in rad/s for error 1 rad.
 *
 * @unit 1/s
 * @min 0.0
 * @group Multicopter Attitude Control
 */
PX4_PARAM_DEFINE_FLOAT(MC_YAW_P);
