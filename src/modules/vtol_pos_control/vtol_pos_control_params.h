/****************************************************************************
 *
 *   Copyright (c) 2013 - 2015 PX4 Development Team. All rights reserved.
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
#pragma once

#include <math.h>
#include <lib/mathlib/mathlib.h>
#include <px4.h>
#include <vtol_pos_control/param_define.h>

using namespace px4;

// **********************************************************************
// *** Main parameter class and functionality
// **********************************************************************

class parameters
{
public:
    parameters();
    ~parameters(void);

    int update();

public:
    struct {
        math::Vector<3> pos_p;
        math::Vector<3> vel_p;
        math::Vector<3> vel_d;
        math::Vector<3> vel_i;
        math::Vector<3> vel_ff;
        float vz_max;
        float mc_yaw_p;
        float man_roll_max;
        float man_pitch_max;
        float man_yaw_max;
        float acc_trans;
        float pitch_transit;
        float airspeed_transit;
        float angle_acc_trans;
        float thrust_scaling;
        float pitch_trim;
        float pitch_sensitivity;
        float fw_cruise_speed;
    } _params;
private:
    struct {
        ParameterFloat pos_xy_p = ParameterFloat("VTP_POS_XY_P", PARAM_VTP_POS_XY_P_DEFAULT);
        ParameterFloat pos_z_p = ParameterFloat("VTP_POS_Z_P", PARAM_VTP_POS_Z_P_DEFAULT);
        ParameterFloat vel_xy_p = ParameterFloat("VTP_VEL_XY_P", PARAM_VTP_VEL_XY_P_DEFAULT);
        ParameterFloat vel_z_p = ParameterFloat("VTP_VEL_Z_P", PARAM_VTP_VEL_Z_P_DEFAULT);
        ParameterFloat vel_xy_d = ParameterFloat("VTP_VEL_XY_D", PARAM_VTP_VEL_XY_D_DEFAULT);
        ParameterFloat vel_z_d = ParameterFloat("VTP_VEL_Z_D", PARAM_VTP_VEL_Z_D_DEFAULT);
        ParameterFloat vel_xy_i = ParameterFloat("VTP_VEL_XY_I", PARAM_VTP_VEL_XY_I_DEFAULT);
        ParameterFloat vel_z_i = ParameterFloat("VTP_VEL_Z_I", PARAM_VTP_VEL_Z_I_DEFAULT);
        ParameterFloat vel_xy_ff = ParameterFloat("VTP_VEL_XY_FF", PARAM_VTP_VEL_XY_FF_DEFAULT);
        ParameterFloat vel_z_ff = ParameterFloat("VTP_VEL_Z_FF", PARAM_VTP_VEL_Z_FF_DEFAULT);
        ParameterFloat vz_max = ParameterFloat("VTP_VZ_MAX", PARAM_VTP_VZ_MAX_DEFAULT);

        ParameterFloat man_roll_max = ParameterFloat("VTP_MAN_R_MAX", PARAM_VTP_MAN_R_MAX_DEFAULT);
        ParameterFloat man_pitch_max = ParameterFloat("VTP_MAN_P_MAX", PARAM_VTP_MAN_P_MAX_DEFAULT);
        ParameterFloat man_yaw_max = ParameterFloat("VTP_MAN_Y_MAX", PARAM_VTP_MAN_Y_MAX_DEFAULT);

        ParameterFloat acc_trans = ParameterFloat("VTP_ACC_TRANS", PARAM_VTP_ACC_TRANS_DEFAULT);
        ParameterFloat pitch_transit = ParameterFloat("VTP_PTCH_TRANSIT", PARAM_VTP_PTCH_TRANSIT_DEFAULT);
        ParameterFloat airspeed_transit = ParameterFloat("VTP_SPD_TRANSIT", PARAM_VTP_SPD_TRANSIT_DEFAULT);
        ParameterFloat angle_acc_trans = ParameterFloat("VTP_ACC_ANG", PARAM_VTP_ACC_ANG_DEFAULT);
        ParameterFloat thrust_scaling = ParameterFloat("VTP_THRUST_SCALE", PARAM_VTP_THRUST_SCALE_DEFAULT);
        ParameterFloat pitch_trim = ParameterFloat("VTP_PITCH_TRIM", PARAM_VTP_PITCH_TRIM_DEFAULT);
        ParameterFloat pitch_sensitivity = ParameterFloat("VTP_PITCH_SENS", PARAM_VTP_PITCH_SENS_DEFAULT);
        ParameterFloat fw_cruise_speed = ParameterFloat("VTP_CRUISE_SPD", PARAM_VTP_CRUISE_SPD_DEFAULT);
        ParameterFloat mc_yaw_p = ParameterFloat("MC_YAW_P", PARAM_MC_YAW_P_DEFAULT); //TODO 
    } _params_handles;
};
