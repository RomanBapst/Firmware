/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
 * @file VTOL_pos_main.cpp
 *
 * @author Roman Bapst 	<bapstr@ethz.ch>
 *
 */

#include <nuttx/config.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/circuit_breaker.h>
#include <lib/mathlib/mathlib.h>
#include <lib/geo/geo.h>

#include <vtol_pos_control/vtol_pos_control.h>

#define GRAVITY 9.81f

/**
* Constructor
*/
VtolPositionControl::VtolPositionControl() :
	_n(_appState),

	_v_att_sp(),
	_local_pos_sp(),

	// init publication handlers
	_local_pos_sp_pub(nullptr),
	_att_sp_pub(nullptr),
	//_loop_perf(perf_alloc(PC_ELAPSED, "vtol_pos_control")),
	//_nonfinite_input_perf(perf_alloc(PC_COUNT, "vtol pos control nonfinite input")),
	_old_time_stamp(px4::get_time_micros())
{
	// Subscriptions
	_v_att = _n.subscribe<px4_vehicle_attitude>(&VtolPositionControl::handle_vehicle_attitude, this, 0);
	_v_control_mode = _n.subscribe<px4_vehicle_control_mode>(0);
	_manual_control_sp = _n.subscribe<px4_manual_control_setpoint>(0);
	_local_pos = _n.subscribe<px4_vehicle_local_position>(0);
	_actuator_status = _n.subscribe<px4_actuator_armed>(0);
	_param_update = _n.subscribe<px4_parameter_update>(&VtolPositionControl::handle_parameter_update, this, 1000);

	// Advertizement
	_att_sp_pub = _n.advertise<px4_vehicle_attitude_setpoint>();
	_local_pos_sp_pub = _n.advertise<px4_vehicle_local_position_setpoint>();
	

	_vtol.status = MC_MODE;
	_vtol.status_prev = MC_MODE;
	_vtol.q_trans.from_euler(0,0,0);
	_vtol.vel_trans_abs = 0.0f;
	_vtol.pitch_trans = 0.0f;
	
	_pos.zero();
	_pos_sp.zero();
	_vel.zero();
	_vel_sp.zero();
	_q_sp.from_euler(0,0,0);
	_R.set(_v_att->data().R);
	_R_sp.from_euler(0,0,0);
	_euler.zero();
	_f_aero.zero();
	_thrust_sp = 0.0f;
	_a_sp.zero();
	_a_sp_prev = {0,0,-1.0f};
	_vel_error_integ.zero();
	_vel_error_prev.zero();

	_dt = 0.01f;
	_yaw_sp = 0.0f;
	_gravity_factor = 1.0f;
	_reset_vel_xy_int = true;
	_reset_vel_z_int = true;
	_reset_pos = true;
	_reset_yaw_sp = true;
	_reset_vel_error = true;
	_a_sp_prev_valid = false;

	// fetch initial parameter values
	_p.update();
}

/**
* Destructor
*/
VtolPositionControl::~VtolPositionControl()
{
}

void VtolPositionControl::handle_parameter_update(const px4_parameter_update &msg)
{
    _p.update();
}

// velocity setpoint for fw control
void VtolPositionControl::calc_des_pos_and_vel() {
	// user can change yaw with roll stick
	float turn_angle = _manual_control_sp->data().y*0.5f;
	float vel_abs = _p._params.fw_cruise_speed + _manual_control_sp->data().z * 10.0f;
	_vel_sp(0) = vel_abs * cosf(_yaw_sp + turn_angle);
	_vel_sp(1) = vel_abs * sinf(_yaw_sp + turn_angle);
	_vel_sp(2) = _p._params.pitch_sensitivity*_manual_control_sp->data().x + _p._params.pitch_trim;

	// do not use integral for fw flight
	_reset_vel_xy_int = true;
	_reset_vel_z_int = true;
	_reset_vel_error = false;
}

// only used for mc control
void VtolPositionControl::get_desired_velocity() {
	if(_reset_pos) {
		_pos_sp = _pos;
		_reset_pos = false;
	}

	// commanded move rate by user
	math::Vector<3> move_rate = {0,0,0};

	if(_v_control_mode->data().flag_control_altitude_enabled && !_v_control_mode->data().flag_control_position_enabled) {
		_pos_sp(0) = _pos(0);
		_pos_sp(1) = _pos(1);
		_pos_sp(2) = _pos(2);
		_vel_sp(2) = -2*(_manual_control_sp->data().z - 0.5f)*_p._params.vz_max;
		_vel_sp(0) = 0.0f;
		_vel_sp(1) = 0.0f;
	}
	else if(_v_control_mode->data().flag_control_position_enabled) {
		move_rate(0) = _manual_control_sp->data().x;
		move_rate(1) = _manual_control_sp->data().y;
		move_rate(2) = _manual_control_sp->data().z;
		// rotate move rates with yaw
		math::Matrix<3,3> R_yaw;
		float yaw = get_feasible_yaw();
		R_yaw.from_euler(0.0f,0.0f,yaw);
		move_rate = R_yaw*move_rate;
		// user gives move rate with sticks
		_pos_sp(0) += move_rate(0)*_dt;
		_pos_sp(1) +=  move_rate(1)*_dt;

		if(_manual_control_sp->data().z < 0.35f || _manual_control_sp->data().z > 0.65f) {
			_pos_sp(2) -= 1.0f*(_manual_control_sp->data().z - 0.5f)*_dt;
		}

		math::Vector<3> pos_err = _pos_sp - _pos;
		_vel_sp = pos_err.emult(_p._params.pos_p) + move_rate.emult(_p._params.vel_ff);
	}
	// create velocity setpoint from position error
	// prevent throttle loss during high climb rate phase
	if(_vel_sp(2) - _vel(2) > 0.7f) {
		_vel_sp(2) = _vel(2) + 0.7f;
	}
	_gravity_factor = 1.0f;	// assume not aerodyanmic lift, need to compensate for gravity
	_a_sp_prev_valid = false;
}

// caluclate desired acceleration from desired velocity
void VtolPositionControl::get_desired_acceleration() {
	// provisional feedforward, not used yet
	math::Vector<3> acc_feedforward;
	memset(&acc_feedforward,0,sizeof(acc_feedforward));

	// calculate velocity error
	math::Vector<3> vel_error;
	// deadband for fixed wing mode
	if(_manual_control_sp->data().aux2 > 0.0f && fabsf(_vel(2)) < 1.0f) {
		math::Vector<3> helper;
		helper = _vel;
		helper(2) *= helper(2)*helper(2);
		vel_error = _vel_sp - helper;
	}
	else {
		vel_error = _vel_sp - _vel;	// in global frame
	}

	
	_vel_error_integ += vel_error*_dt;	// update integral

	if(_reset_vel_xy_int) {
		_vel_error_integ(0) = 0.0f;
		_vel_error_integ(1) = 0.0f;
		_reset_vel_xy_int = false;
	}
	if(_reset_vel_z_int) {
		_vel_error_integ(2) = 0.0f;
		_reset_vel_z_int = false;
	}

	// initialize D controller
	if(_reset_vel_error) {
		_vel_error_prev = vel_error;
		_reset_vel_error = false;
	}

	// limit integration error to prevent windup
	_vel_error_integ(0) = math::constrain(_vel_error_integ(0),-8.0f,8.0f);
	_vel_error_integ(1) = math::constrain(_vel_error_integ(1),-8.0f,8.0f);
	_vel_error_integ(2) = math::constrain(_vel_error_integ(2),-8.0f,8.0f);

	_a_sp = acc_feedforward + _p._params.vel_p.emult(vel_error) + _p._params.vel_d.emult(vel_error - _vel_error_prev)/_dt
		+ _p._params.vel_i.emult(_vel_error_integ);
	_vel_error_prev = vel_error;

	if(_manual_control_sp->data().aux2 > 0.0f) {
		float a_corr = _a_sp.length();
		_a_sp = a_corr > GRAVITY*0.6f ? _a_sp*GRAVITY*0.6f/a_corr : _a_sp;
		math::Vector<3> thrust_line = {0.0f,0.0f,-1.0f};
		_a_sp += _R*thrust_line*GRAVITY;
	}
	else {
		_a_sp(2) -= GRAVITY;
	}

		// reset previous acceleration setpoint if not valid
		/*
		if(!_a_sp_prev_valid) {
			_a_sp_prev = _a_sp;
			_a_sp_prev_valid = true;
		}
		float acc_norm = math::constrain(_a_sp.length(),GRAVITY,1.0f/_p._params.thrust_scaling);
		// limit rate of change of a_sp
		math::Vector<3> a_sp_norm = _a_sp.normalized();
		math::Vector<3> a_sp_prev_norm = _a_sp_prev.normalized();
		float inner_prod = math::constrain(a_sp_norm*a_sp_prev_norm,-1.0f,1.0f);
		float beta = acosf(inner_prod);
		if(fabsf(beta) > M_PI_F*_dt) {
			beta = M_PI_F*_dt;
			math::Vector<3> axis = a_sp_prev_norm % a_sp_norm;
			math::Quaternion q_limiter = {cosf(beta/2), sinf(beta/2)*axis(0),sinf(beta/2)*axis(1),sinf(beta/2)*axis(2)};
			a_sp_norm = q_limiter.rotate(a_sp_prev_norm);
		}
		*/

	//_a_sp = a_sp_norm*acc_norm;
	//_a_sp_prev = _a_sp;	// update
}

// not used for now
void VtolPositionControl::get_aerodynamic_force() {
	// for now pretend there are no aerodynamic forces
	memset(&_f_aero,0,sizeof(_f_aero));
}

// main controller, which generates quaternion setpoint
void VtolPositionControl::run_controller() {
	// get desired acceleration
	get_desired_acceleration();

	// substract estimated aerodynamic acceleration
	//get_aerodynamic_force();

	// rotate desired acceleration such that nose of vehicle is in flight direction
	math::Quaternion q_yaw;
	q_yaw.from_euler(0.0f,0.0f,_yaw_sp);
	math::Matrix<3,3> R_yaw = q_yaw.to_dcm();
	R_yaw = R_yaw.transposed();
	math::Vector<3> a_des_yaw = R_yaw*_a_sp;

	// compute desired rotation angle
	math::Vector<3> thrust_des_body = a_des_yaw.normalized();
	math::Vector<3> zi = {0.0f,0.0f,-1.0f};
	float inner_prod = zi*thrust_des_body;
	inner_prod = math::constrain(inner_prod,-1.0f,1.0f);
	float alpha = acosf(inner_prod);

	// compute axis of desired rotation
	math::Vector<3> rot_axis;
	if(fabsf(alpha) < 0.01f) {
		rot_axis.zero();
		rot_axis(1) = 1.0f;
	}
	else {
		rot_axis = zi%thrust_des_body;
	}

	rot_axis.normalize();

	// compute desired quaternion
	math::Quaternion q_xy;
	q_xy(0) = cosf(0.5f*alpha);
	rot_axis*= sinf(0.5f*alpha);
	q_xy(1) = rot_axis(0);
	q_xy(2) = rot_axis(1);
	q_xy(3) = rot_axis(2);

	_q_sp = q_yaw*q_xy;	// combine the two rotations
	_R_sp = _q_sp.to_dcm();

	// rotate body z-axis to global frame
	math::Vector<3> zb_des = thrust_des_body;
	zb_des = R_yaw.transposed()*zb_des;
	thrust_des_body = zb_des;

	// compute desired thrust (project desired to actual axis)
	math::Vector<3> zb = _R*zi;
	_thrust_sp = zb*zb_des;
	_thrust_sp *= _a_sp.length(); // this is not a force yet!
	// limit thrust, min value is imortant to avoid loss of pitch control
	_thrust_sp = math::constrain(_thrust_sp*_p._params.thrust_scaling,0.0f,1.0f);
}

float VtolPositionControl::get_feasible_yaw() {
	if(_vel(0)*_vel(0) + _vel(1)*_vel(1) > 16.0f) {
		return atan2f(_vel(1),_vel(0));
	} else {
		return _euler(2);
	}
}

// for mc mode only
void VtolPositionControl::control_manual() {
	math::Vector<3> euler_des;
	euler_des(0) = _manual_control_sp->data().y * _p._params.man_roll_max;
	euler_des(1) = -_manual_control_sp->data().x * _p._params.man_pitch_max;
	euler_des(2) = _yaw_sp;
	_thrust_sp = _manual_control_sp->data().z;
	_R_sp.from_euler(euler_des(0),euler_des(1),euler_des(2));
	_q_sp.from_euler(euler_des(0),euler_des(1),euler_des(2));
}

// for mc mode only
void VtolPositionControl::control_attitude() {
	get_desired_velocity();
	run_controller();

	// write attitude and thrust setpoint
	math::Vector<3> euler_des;
	euler_des(0) = _manual_control_sp->data().y * _p._params.man_roll_max;
	euler_des(1) = -_manual_control_sp->data().x * _p._params.man_pitch_max;
	euler_des(2) = _yaw_sp;
	_R_sp.from_euler(euler_des(0),euler_des(1),euler_des(2));
	_q_sp.from_euler(euler_des(0),euler_des(1),euler_des(2));
}

// for mc mode only
void VtolPositionControl::control_position() {
	get_desired_velocity();
	run_controller();
}

// publish the attitude setpoint
void VtolPositionControl::publish_att_sp() {
	// write desired attitude & thrust setpoint
	math::Vector<3> euler_des = _R_sp.to_euler();
	_v_att_sp.data().timestamp = px4::get_time_micros();
	memcpy(&_v_att_sp.data().R_body,_R_sp.data,sizeof(_v_att_sp.data().R_body));
	memcpy(&_v_att_sp.data().q_d[0],&_q_sp.data[0],sizeof(_v_att_sp.data().q_d));
	
	// compute quaternion error
	math::Quaternion q_sp_inv = {_q_sp(0),-_q_sp(1),-_q_sp(2),-_q_sp(3)};
	math::Quaternion q_error;
	math::Quaternion q;
	q.from_dcm(_R);
	q_error = q_sp_inv*q;
	memcpy(&_v_att_sp.data().q_e[0],&q_error.data[0],sizeof(_v_att_sp.data().q_e));
	_v_att_sp.data().R_valid = true;
	_v_att_sp.data().roll_body = euler_des(0);
	_v_att_sp.data().pitch_body = euler_des(1);
	_v_att_sp.data().yaw_body = _yaw_sp;
	_v_att_sp.data().thrust = _thrust_sp;

	// publish desired attitude and thrust
	_att_sp_pub->publish(_v_att_sp);
}

// do some special logging for developing purposes
void VtolPositionControl::send_to_log() {
	_local_pos_sp.data().timestamp = px4::get_time_micros();

	// fill local position setpoint
	_local_pos_sp.data().x = _pos_sp(0);
	_local_pos_sp.data().y = _pos_sp(1);
	_local_pos_sp.data().z = _pos_sp(2);
	_local_pos_sp.data().vx = _vel_sp(0);
	_local_pos_sp.data().vy = _vel_sp(1);
	_local_pos_sp.data().vz = _vel_sp(2);
	_local_pos_sp.data().acc_x = _a_sp(0);
	_local_pos_sp.data().acc_y = _a_sp(1);
	_local_pos_sp.data().acc_z = _a_sp(2);

	// Publish
	_local_pos_sp_pub->publish(_local_pos_sp);

}

// control plane in mc mode for close to hover situations
void VtolPositionControl::control_mc() {
	// manual control
	if(!_v_control_mode->data().flag_control_altitude_enabled) {
		// full manual control
		control_manual();
		_reset_vel_z_int = true;
		_reset_vel_xy_int = true;
		_reset_pos = true;
		_reset_vel_error = true;
	}
	else if(_v_control_mode->data().flag_control_altitude_enabled && ! _v_control_mode->data().flag_control_position_enabled) {
		_reset_vel_xy_int = true;
		control_attitude();
	}
	else if(_v_control_mode->data().flag_control_altitude_enabled && _v_control_mode->data().flag_control_position_enabled) {
		control_position();
	}
}

// control plane in fw mode, thrust is done manually
void VtolPositionControl::control_fw() {
	// do normal roll, pitch control to tune low level controller
	calc_des_pos_and_vel();
	run_controller();
	/*
	math::Quaternion q_fw;
	float roll_des = _manual_control_sp->data().y;
	float pitch_des = -_manual_control_sp->data().x;
	math::Quaternion q_offset;
	q_offset.from_euler(0,-M_PI_2_F,0);
	math::Quaternion q;
	q.from_dcm(_R);
	q_fw = q*q_fw;
	math::Quaternion q_manual;
	q_manual.from_euler(roll_des,pitch_des,_yaw_sp);
	_q_sp = q_manual*q_offset;
	_thrust_sp = _manual_control_sp->data().z;
	*/
}

// not importatant for now
void VtolPositionControl::control_trans() {
	if(_vtol.status_prev == MC_MODE) {
		// do a front transition
		math::Quaternion q_pitch;
		q_pitch.from_euler(0,-_vtol.pitch_trans,0);
		_q_sp = _vtol.q_trans*q_pitch;
		_R_sp = _q_sp.to_dcm();
		_thrust_sp = _manual_control_sp->data().z;
		_vtol.pitch_trans += 0.2f*_dt;
		_vtol.pitch_trans = math::constrain(_vtol.pitch_trans,0.0f,1.0f);
	}
	else {
		// do a backtransition
		math::Quaternion q_pitch;
		q_pitch.from_euler(0,_vtol.pitch_trans,0);
		_q_sp = _vtol.q_trans*q_pitch;
		_R_sp = _q_sp.to_dcm();
		_thrust_sp = _manual_control_sp->data().z;
		_vtol.pitch_trans += 0.2f*_dt;
		_vtol.pitch_trans = math::constrain(_vtol.pitch_trans,0.0f,M_PI_2_F);
	}
}

void VtolPositionControl::control_position_step() {
	/* set target position to the position located 40 meters in front of the plane, 10 meters heigher up*/

}

void VtolPositionControl::compute_des_yaw() {
	// compute desired yaw for all modes
	if(_vtol.status == FW_MODE) {
		if(_vel(0)*_vel(0) + _vel(1)*_vel(1) > 16.0f) {
			_yaw_sp = atan2f(_vel(1),_vel(0));
		}
		else if(_vel(0)*_vel(0) + _vel(1)*_vel(1) < 16.0f) {
			_yaw_sp = _euler(2);
		}
	}
	else if(_vtol.status == TRANS_MODE) {
		
	}
	else {	// we are in multicopter mode, let user change yaw
		float yaw_sp_move_rate = _manual_control_sp->data().r * _p._params.man_yaw_max;
		_v_att_sp.data().yaw_sp_move_rate = yaw_sp_move_rate;
		_yaw_sp = _wrap_pi(_yaw_sp + yaw_sp_move_rate * _dt);
		float yaw_offs_max = _p._params.man_yaw_max / _p._params.mc_yaw_p;
		float yaw_offs = _wrap_pi(_yaw_sp - _v_att->data().yaw);
		if (yaw_offs < - yaw_offs_max) {
			_yaw_sp = _wrap_pi(_v_att->data().yaw - yaw_offs_max);
			} else if (yaw_offs > yaw_offs_max) {
			_yaw_sp = _wrap_pi(_v_att->data().yaw + yaw_offs_max);
		}
	}

	// reset yaw setpoint if necessary
	if(_reset_yaw_sp) {
		_yaw_sp = _euler(2);
	}
}

// for now we just want to test fw mode when we are in air, no other logic
void VtolPositionControl::update_vtol_state() {
	if(_manual_control_sp->data().aux2 > 0.0f) {
		_vtol.status = FW_MODE;
	}
	else {
		_vtol.status = MC_MODE;
	}
	// check if user wants to do transition
	/*
	if(_manual_control_sp->data().aux2 > 0.0f && _vtol.status == MC_MODE) {
		// check if possible to do front transtion
		bool allow = false;
		// only allow starting from position control mode
		allow = _v_control_mode->data().flag_control_position_enabled || _v_control_mode->data().flag_control_altitude_enabled;
		// only allow when vehicle is not moving too fast in xy_plane
		allow = (allow && fabsf(_vel(0)*_vel(0) + _vel(1)*_vel(1)) < 4.0f);
		// only allow if vehicle is more or less facing up
		float z_comp = _R(2,2);
		allow = (allow && z_comp > 0.9f);
		// set current yaw to desired yaw for transition
		if(allow) {
			_vtol.status_prev = _vtol.status;
			_vtol.status = TRANS_MODE;
			_vtol.q_trans.from_dcm(_R);
			_vtol.vel_trans_abs = 0.0f;
			_vtol.pitch_trans = 0.0f;
			_vel_sp.zero();
			_yaw_sp = _euler(2);	// reset yaw setpoint
		}
	}
	else if(_manual_control_sp->data().aux2 < 0 && _vtol.status == FW_MODE) {
		// do a backtransition
		_vtol.status_prev = _vtol.status;
		_vtol.status = TRANS_MODE;
		_vtol.pitch_trans = 0.0f;
		_vtol.q_trans.from_dcm(_R);
	}
	else if(_vtol.status == TRANS_MODE && _vtol.status_prev == MC_MODE) {
		if(_vtol.pitch_trans > 0.9f || fabsf(_vel(0)*_vel(0) + _vel(1)*_vel(1)) > 15.0f) {
			_vtol.status_prev = _vtol.status;
			_vtol.status = FW_MODE;
		}
	}
	else if(_vtol.status == TRANS_MODE && _vtol.status_prev == FW_MODE) {
		// get current angle of nose vector and earth z vector
		float angle = acosf(_R(2,2));
		if(fabsf(angle < 0.1f) || fabsf(_vel(0)*_vel(0) + _vel(1)*_vel(1)) < 3.0f) {
			_vtol.status = MC_MODE;
			_vtol.status_prev = MC_MODE;
			_vtol.vel_trans_abs = 0.0f;
			_reset_pos = true;
			_reset_vel_xy_int = true;
			_reset_vel_z_int = true;
		}
	}
	*/
}


void VtolPositionControl::handle_vehicle_attitude(const px4_vehicle_attitude &msg)
{
	// update data
	_pos(0) = _local_pos->data().x;
	_pos(1) = _local_pos->data().y;
	_pos(2) = _local_pos->data().z;

	_vel(0) = _local_pos->data().vx;
	_vel(1) = _local_pos->data().vy;
	_vel(2) = _local_pos->data().vz;

	_R.set(_v_att->data().R);
	_euler = _R.to_euler();
	// get usual dt estimate
	_dt = _old_time_stamp != 0 ? (px4::get_time_micros() - _old_time_stamp) * 0.000001f : 0.01f;
	_old_time_stamp = px4::get_time_micros();

	// reset yaw setpoint if not armed
	_reset_yaw_sp = false;

	compute_des_yaw();

	// failsave: user can switch to the fw controller, everything will be reset
	if(_manual_control_sp->data().aux1 <= 0.0f) {
		update_vtol_state();
	}
	else {
		// user has brutally switched to fw save mode, reset everything
		_vtol.status = MC_MODE;
		_vtol.status_prev = MC_MODE;
		_vtol.vel_trans_abs = 0.0f;
		_vtol.pitch_trans = 0.0f;
		_reset_pos = true;
		_reset_vel_xy_int = true;
		_reset_vel_z_int = true;
		_reset_vel_error = true;
	}

	if(_vtol.status == MC_MODE) {
		// fly as multicopter
		control_mc();
	}
	else if(_vtol.status == FW_MODE)
	{
		// fly as fw
		control_fw();
	}
	else {
		// do transition
		control_trans();
	}

	// send data to attitude controller, prevent double publishing with fw attitude controller(failsave)
	if(_manual_control_sp->data().aux1 < 0) {
		publish_att_sp();
	}
	// do some special logging for developing purposes
	send_to_log();
}
