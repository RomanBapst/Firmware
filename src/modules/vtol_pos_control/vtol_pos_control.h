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

/**
 * @file vtol_pos_control.h
 * Vtol position controller.
 *
 */


#ifndef PX4_SRC_MODULES_VTOL_POS_CONTROL_H_
#define PX4_SRC_MODULES_VTOL_POS_CONTROL_H_

#include <px4.h>
#include <cstdio>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>

#include <vtol_pos_control/vtol_pos_control_params.h>

using namespace px4;

class VtolPositionControl
{
public:
	/**
	 * Constructor
	 */
	VtolPositionControl();

	/**
	 * Destructor, also kills task.
	 */
	~VtolPositionControl();

	/* Callbacks for topics */
	void handle_vehicle_attitude(const px4_vehicle_attitude &msg);
	void handle_parameter_update(const px4_parameter_update &msg);

	void spin() { _n.spin(); }

protected:
	px4::NodeHandle _n;
	px4::AppState _appState;
	

	bool		_task_should_exit;  					/**< if true, task should exit */
	int		_control_task;  					/**< task handle for task */
	int		_mavlink_fd;  						/**< mavlink fd */

	Subscriber<px4_vehicle_attitude> *_v_att;				    	/**< vehicle attitude */
	Subscriber<px4_vehicle_control_mode> *_v_control_mode;  		/**< vehicle control mode subscription */
	Subscriber<px4_parameter_update> *_param_update;  					/**< parameter updates subscription */
	Subscriber<px4_manual_control_setpoint> *_manual_control_sp;  		/**< manual control setpoint subscription */
	Subscriber<px4_vehicle_local_position> *_local_pos;  			/**< sensor subscription */
	Subscriber<px4_actuator_armed> *_actuator_status;

	px4_vehicle_attitude_setpoint _v_att_sp;
	px4_vehicle_local_position_setpoint _local_pos_sp;

	Publisher<px4_vehicle_local_position_setpoint>	*_local_pos_sp_pub;
	Publisher<px4_vehicle_attitude_setpoint>	*_att_sp_pub;  		/**< attitude setpoint publication */

	uint64_t _old_time_stamp;

	parameters _p;

	//perf_counter_t	_loop_perf;			// loop performance counter
	//perf_counter_t	_nonfinite_input_perf;		// performance counter for non finite input

	typedef enum {
		MC_MODE=0,
		TRANS_MODE,
		FW_MODE
	}vtol_mode;

	struct
	{
		vtol_mode status;		// current mode
		vtol_mode status_prev;	// previous mode
		math::Quaternion q_trans;
		float vel_trans_abs;	// absolute xy velocity during transition
		float pitch_trans;
	}_vtol;

	math::Vector<3> _pos;
	math::Vector<3> _pos_sp;
	math::Vector<3> _vel;
	math::Vector<3> _vel_sp;
	math::Quaternion _q_sp;
	math::Matrix<3,3> _R;
	math::Matrix<3,3> _R_sp;
	math::Vector<3> _euler;
	math::Vector<3> _f_aero;
	math::Vector<3> _a_sp;
	math::Vector<3> _a_sp_prev;

	math::Vector<3> _vel_error_prev;
	math::Vector<3> _vel_error_integ;

	float _thrust_sp;
	float _yaw_sp;
	float _dt;
	bool _reset_pos;
	bool _reset_vel_xy_int;
	bool _reset_vel_z_int;
	bool _reset_yaw_sp;
	bool _reset_vel_error;
	bool _a_sp_prev_valid;
	float _gravity_factor;


//***************** Member functions ***********************************************************************

	void 		update_vtol_state();

	void 		control_mc();
	void 		control_manual();
	void 		control_attitude();
	void 		control_position();
	void 		get_desired_velocity();

	void 		control_fw();
	void 		calc_des_pos_and_vel();


	void 		control_trans();
	void 		control_position_step();

	void 		compute_des_yaw();
	void 		get_desired_acceleration();
	void 		get_aerodynamic_force();
	void 		run_controller();
	void 		publish_att_sp();
	void 		send_to_log();
	float 		get_feasible_yaw();
};
#endif  // PX4_SRC_MODULES_VTOL_POS_CONTROL_H_
