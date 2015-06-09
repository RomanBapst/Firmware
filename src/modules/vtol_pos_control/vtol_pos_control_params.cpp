/****************************************************************************
 *
 *   Copyright (c) 2012, 2013 PX4 Development Team. All rights reserved.
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

#include <vtol_pos_control/vtol_pos_control_params.h>


//**********************************************************************
//*** Constructors / Destructors
//**********************************************************************
parameters::parameters()
{
}

parameters::~parameters()
{
}

//**********************************************************************
//*** Functions
//**********************************************************************

int parameters::update()
{
    /* position error gain */
    _params.pos_p(0) = _params_handles.pos_xy_p.update();
    _params.pos_p(1) = _params_handles.pos_xy_p.update();
    _params.pos_p(2) = _params_handles.pos_z_p.update();

    /* xy velocity error gain */
    _params.vel_p(0) = _params_handles.vel_xy_p.update();
    _params.vel_p(1) = _params_handles.vel_xy_p.update();
    _params.vel_p(2) = _params_handles.vel_z_p.update();

    /* xy velocity error d gain */
    _params.vel_d(0) = _params_handles.vel_xy_d.update();
    _params.vel_d(1) = _params_handles.vel_xy_d.update();
    _params.vel_d(2) = _params_handles.vel_z_d.update();

    /* xy velocity error i gain */
    _params.vel_i(0) = _params_handles.vel_xy_i.update();
    _params.vel_i(1) = _params_handles.vel_xy_i.update();
    _params.vel_i(2) = _params_handles.vel_z_i.update();

    /* xy velocity feedforward gain */
    _params.vel_ff(0) = _params_handles.vel_xy_ff.update();
    _params.vel_ff(1) = _params_handles.vel_xy_ff.update();

    /* z velocity feedforward gain */
    _params.vel_ff(2) = _params_handles.vel_z_ff.update();

    /* max maginitude of z velocity */
    _params.vz_max = _params_handles.vz_max.update();

    /* attitude yaw error p gain */
    _params.mc_yaw_p = _params_handles.mc_yaw_p.update();

    _params.man_roll_max = _params_handles.man_roll_max.update();
    _params.man_pitch_max = _params_handles.man_pitch_max.update();
    _params.man_yaw_max = _params_handles.man_yaw_max.update();
    _params.man_roll_max = math::radians(_params.man_roll_max);
    _params.man_pitch_max = math::radians(_params.man_pitch_max);
    _params.man_yaw_max = math::radians(_params.man_yaw_max);

    _params.acc_trans = _params_handles.acc_trans.update();
    _params.pitch_transit = _params_handles.pitch_transit.update();
    _params.pitch_transit = math::radians(_params.pitch_transit);
    _params.airspeed_transit = _params_handles.airspeed_transit.update();
    _params.angle_acc_trans = _params_handles.angle_acc_trans.update();
    _params.angle_acc_trans = math::radians(_params.angle_acc_trans);
    _params.thrust_scaling = _params_handles.thrust_scaling.update();
    _params.pitch_trim = _params_handles.pitch_trim.update();
    _params.pitch_sensitivity = _params_handles.pitch_sensitivity.update();
    _params.fw_cruise_speed = _params_handles.fw_cruise_speed.update();

    return 0;
}