/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

#include <drivers/drv_hrt.h>
#include <ecl/airdata/WindEstimator.hpp>
#include <matrix/math.hpp>
#include <parameters/param.h>
#include <perf/perf_counter.h>
#include <px4_module.h>
#include <px4_module_params.h>
#include <px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/airspeed/airspeed.h>
#include <AirspeedValidator.hpp>
#include <systemlib/mavlink_log.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/airspeed.h>
#include <uORB/topics/airspeed_validated.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/mavlink_log.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_bias.h>
#include <uORB/topics/vehicle_air_data.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vtol_vehicle_status.h>
#include <uORB/topics/wind_estimate.h>

using namespace time_literals;

static constexpr uint32_t SCHEDULE_INTERVAL{100_ms};	/**< The schedule interval in usec (10 Hz) */

using matrix::Dcmf;
using matrix::Quatf;
using matrix::Vector2f;
using matrix::Vector3f;

class AirspeedModule : public ModuleBase<AirspeedModule>, public ModuleParams,
	public px4::ScheduledWorkItem
{
public:

	AirspeedModule();

	~AirspeedModule();

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/* run the main loop */
	void Run() override;

	// int print_status() override;

private:
	orb_advert_t	_airspeed_validated_pub {nullptr};			/**< airspeed validated topic*/
	orb_advert_t 	_wind_est_pub[4] {nullptr};							/**< wind estimate topic (for each airspeed validator + purely sideslip fusion) */
	orb_advert_t 	_mavlink_log_pub {nullptr}; 						/**< mavlink log topic*/

	uORB::Subscription _estimator_status_sub{ORB_ID(estimator_status)};
	uORB::Subscription _param_sub{ORB_ID(parameter_update)};
	uORB::Subscription _sensor_bias_sub{ORB_ID(sensor_bias)};
	uORB::Subscription _vehicle_air_data_sub{ORB_ID(vehicle_air_data)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vtol_vehicle_status_sub{ORB_ID(vtol_vehicle_status)};

	estimator_status_s 	_estimator_status {};
	parameter_update_s _parameter_update {};
	sensor_bias_s _sensor_bias {};
	vehicle_air_data_s _vehicle_air_data {};
	vehicle_attitude_s _vehicle_attitude {};
	vehicle_land_detected_s _vehicle_land_detected {};
	vehicle_local_position_s _vehicle_local_position {};
	vehicle_status_s _vehicle_status {};
	vtol_vehicle_status_s _vtol_vehicle_status {};

	WindEstimator	_wind_estimator_sideslip; /**< wind estimator instance only fusing sideslip */
	wind_estimate_s _wind_estimate_sideslip {}; /**< wind estimate message for wind estimator instance only fusing sideslip */

	int _airspeed_sub[3] {0}; /**< raw airspeed topics subscriptions. Max 3 airspeeds sensors. */
	int _number_of_airspeed_sensors{0}; /**<  number of airspeed sensors in use (detected during initialization)*/
	AirspeedValidator *_airspeed_validator{nullptr}; /**< airspeedValidator instances (one for each sensor, assigned dynamically during startup) */

	int _valid_airspeed_index{-1}; /**< index of currently chosen (valid) airspeed sensor */
	int _prev_airspeed_index{-1}; /**< previously chosen airspeed sensor index */
	bool _initialized{false}; /**< module initialized*/
	bool _vehicle_local_position_valid{false}; /**< local position (from GPS) valid */
	bool _in_takeoff_situation{true}; /**< in takeoff situation (defined as not yet stall speed reached) */
	float _ground_minus_wind_TAS{0.0f}; /**< true airspeed from groundspeed minus windspeed */
	float _ground_minus_wind_EAS{0.0f}; /**< true airspeed from groundspeed minus windspeed */

	perf_counter_t _perf_elapsed{};
	perf_counter_t _perf_interval{};


	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::WEST_W_P_NOISE>) _param_west_w_p_noise, /**<  */
		(ParamFloat<px4::params::WEST_SC_P_NOISE>) _param_west_sc_p_noise, /**<  */
		(ParamFloat<px4::params::WEST_TAS_NOISE>) _param_west_tas_noise, /**<  */
		(ParamFloat<px4::params::WEST_BETA_NOISE>) _param_west_beta_noise, /**<  */
		(ParamInt<px4::params::WEST_TAS_GATE>) _param_west_tas_gate, /**<  */
		(ParamInt<px4::params::WEST_BETA_GATE>) _param_west_beta_gate, /**<  */

		(ParamFloat<px4::params::COM_TAS_FS_INNOV>) _tas_innov_threshold, /**< innovation check threshold */
		(ParamFloat<px4::params::COM_TAS_FS_INTEG>) _tas_innov_integ_threshold, /**< innovation check integrator threshold */
		(ParamInt<px4::params::COM_TAS_FS_T1>) _checks_fail_delay, /**< delay to declare airspeed invalid */
		(ParamInt<px4::params::COM_TAS_FS_T2>) _checks_clear_delay, /**<  delay to declare airspeed valid again */
		(ParamFloat<px4::params::COM_ASPD_STALL>) _airspeed_stall /**<  stall speed*/
	)

	int 		start();
	void		update_params(); /**< update parameters */
	void 		poll_topics(); /**< poll all topics required beside airspeed (e.g. current temperature) */
	void 		update_wind_estimator_sideslip(); /**< update the wind estimator instance only fusing sideslip */
	void		update_ground_minus_wind_airspeed(); /**< update airspeed estimate based on groundspeed minus windspeed */
	void 		select_airspeed_and_publish(); /**< select airspeed sensor (or groundspeed-windspeed) */
	void 		publish_wind_estimates(); /**< publish wind estimator states (from all wind estimators running) */

};

AirspeedModule::AirspeedModule():
	ModuleParams(nullptr),
	ScheduledWorkItem(px4::wq_configurations::lp_default)
{
	// initialise parameters
	update_params();

	_perf_elapsed = perf_alloc_once(PC_ELAPSED, "wind_estimator elapsed");
	_perf_interval = perf_alloc_once(PC_INTERVAL, "wind_estimator interval");
}

AirspeedModule::~AirspeedModule()
{
	ScheduleClear();

	orb_unadvertise(_wind_est_pub);
	orb_unadvertise(_airspeed_validated_pub);

	// delete
	perf_free(_perf_elapsed);
	perf_free(_perf_interval);

	delete _airspeed_validator;
}

int
AirspeedModule::task_spawn(int argc, char *argv[])
{
	/* schedule a cycle to start things */
	AirspeedModule *dev = new AirspeedModule();

	// check if the trampoline is called for the first time
	if (!dev) {
		PX4_ERR("alloc failed");
		return PX4_ERROR;
	}

	_object.store(dev);

	if (dev) {
		dev->ScheduleOnInterval(SCHEDULE_INTERVAL, 10000);
		_task_id = task_id_is_work_queue;
		return PX4_OK;
	}

	return PX4_ERROR;
}

void
AirspeedModule::Run()
{
	perf_count(_perf_interval);
	perf_begin(_perf_elapsed);

	/* the first time we run through here, initialize N airspeedValidator
	 * instances (N = number of airspeed sensors detected)
	 */
	if (!_initialized) {
		int max_airspeed_sensors = 3;

		for (int i = 0; i < max_airspeed_sensors; i++) {
			if (orb_exists(ORB_ID(airspeed), i) != 0) {
				continue;
			}

			_number_of_airspeed_sensors = i + 1;
		}

		_airspeed_validator = new AirspeedValidator[_number_of_airspeed_sensors];

		if (_number_of_airspeed_sensors > 0) {
			for (int i = 0; i < _number_of_airspeed_sensors; i++) {
				_airspeed_sub[i] = orb_subscribe_multi(ORB_ID(airspeed), i);

			}
		}

		_initialized = true;
	}


	if (_param_sub.update(&_parameter_update)) {
		update_params();
	}

	poll_topics();
	update_wind_estimator_sideslip();
	update_ground_minus_wind_airspeed();

	if (_airspeed_validator != nullptr) {

		bool armed = (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
		bool fixed_wing = !_vtol_vehicle_status.vtol_in_rw_mode;
		bool in_air = !_vehicle_land_detected.landed;

		/* Prepare data for airspeed_validator */
		struct airspeed_validator_update_data input_data = {};
		input_data.timestamp = hrt_absolute_time();
		input_data.lpos_vx = _vehicle_local_position.vx;
		input_data.lpos_vy = _vehicle_local_position.vy;
		input_data.lpos_vz = _vehicle_local_position.vz;
		input_data.lpos_valid = _vehicle_local_position_valid;
		input_data.lpos_evh = _vehicle_local_position.evh;
		input_data.lpos_evv = _vehicle_local_position.evv;
		input_data.att_q[0] = _vehicle_attitude.q[0];
		input_data.att_q[1] = _vehicle_attitude.q[1];
		input_data.att_q[2] = _vehicle_attitude.q[2];
		input_data.att_q[3] = _vehicle_attitude.q[3];
		input_data.air_pressure_pa = _vehicle_air_data.baro_pressure_pa;
		input_data.air_temperature_celsius = _vehicle_air_data.baro_temp_celcius;
		input_data.accel_z = _sensor_bias.accel_z;
		input_data.vel_test_ratio = _estimator_status.vel_test_ratio;
		input_data.mag_test_ratio = _estimator_status.mag_test_ratio;

		/* iterate through all airspeed sensors, poll new data from them and update their validators */
		for (int i = 0; i < _number_of_airspeed_sensors; i++) {

			/* poll airspeed data */
			airspeed_s airspeed_raw = {};
			orb_copy(ORB_ID(airspeed), _airspeed_sub[i], &airspeed_raw); // poll raw airspeed topic of the i-th sensor
			input_data.airspeed_indicated_raw = airspeed_raw.indicated_airspeed_m_s;
			input_data.airspeed_true_raw = airspeed_raw.true_airspeed_m_s;
			input_data.airspeed_timestamp = airspeed_raw.timestamp;

			/* update in_fixed_wing_flight for the current airspeed sensor validator */
			/* takeoff situation is active from start till one of the sensors' IAS or groundspeed_EAS is above stall speed */
			if (airspeed_raw.indicated_airspeed_m_s > _airspeed_stall.get() || _ground_minus_wind_EAS > _airspeed_stall.get()) {
				_in_takeoff_situation = false;
			}

			/* reset takeoff_situation to true when not in air or not in fixed-wing mode */
			if (!in_air || !fixed_wing) {
				_in_takeoff_situation = true;
			}

			input_data.in_fixed_wing_flight = (armed && fixed_wing && in_air && !_in_takeoff_situation);

			/* push input data into airspeed validator */
			_airspeed_validator[i].update_airspeed_validator(input_data);

		}
	}




	select_airspeed_and_publish();

	perf_end(_perf_elapsed);

	if (should_exit()) {
		exit_and_cleanup();
	}
}


void AirspeedModule::update_params()
{
	updateParams();

	/* update airspeedValidator parameters */
	for (int i = 0; i < _number_of_airspeed_sensors; i++) {
		_airspeed_validator[i].set_wind_estimator_wind_p_noise(_param_west_w_p_noise.get());
		_airspeed_validator[i].set_wind_estimator_tas_scale_p_noise(_param_west_sc_p_noise.get());
		_airspeed_validator[i].set_wind_estimator_tas_noise(_param_west_tas_noise.get());
		_airspeed_validator[i].set_wind_estimator_beta_noise(_param_west_beta_noise.get());
		_airspeed_validator[i].set_wind_estimator_tas_gate(_param_west_tas_gate.get());
		_airspeed_validator[i].set_wind_estimator_beta_gate(_param_west_beta_gate.get());

		_airspeed_validator[i].set_tas_innov_threshold(_tas_innov_threshold.get());
		_airspeed_validator[i].set_tas_innov_integ_threshold(_tas_innov_integ_threshold.get());
		_airspeed_validator[i].set_checks_fail_delay(_checks_fail_delay.get());
		_airspeed_validator[i].set_checks_clear_delay(_checks_clear_delay.get());
		_airspeed_validator[i].set_airspeed_stall(_airspeed_stall.get());
	}
}

void AirspeedModule::poll_topics()
{
	const hrt_abstime time_now_usec = hrt_absolute_time();

	if (_estimator_status_sub.update(&_estimator_status)) {
		// _vehicle_attitude_valid = (_time_now_usec - sensor_bias.timestamp < 1_s) && (sensor_bias.timestamp > 0);
	}

	if (_sensor_bias_sub.update(&_sensor_bias)) {
		// _vehicle_attitude_valid = (_time_now_usec - sensor_bias.timestamp < 1_s) && (sensor_bias.timestamp > 0);
	}

	if (_vehicle_air_data_sub.update(&_vehicle_air_data)) {
		// vehicle_air_data_valid = (_time_now_usec - vehicle_air_data.timestamp < 1_s) && (vehicle_air_data.timestamp > 0);
	}

	if (_vehicle_attitude_sub.update(&_vehicle_attitude)) {
		// _vehicle_attitude_valid = (_time_now_usec - vehicle_attitude.timestamp < 1_s) && (vehicle_attitude.timestamp > 0);
	}

	if (_vehicle_land_detected_sub.update(&_vehicle_land_detected)) {
		// _vehicle_attitude_valid = (_time_now_usec - vehicle_attitude.timestamp < 1_s) && (vehicle_attitude.timestamp > 0);
	}

	if (_vehicle_local_position_sub.update(&_vehicle_local_position)) {
		_vehicle_local_position_valid = (time_now_usec - _vehicle_local_position.timestamp < 1_s)
						&& (_vehicle_local_position.timestamp > 0) && _vehicle_local_position.v_xy_valid;
	}

	if (_vehicle_status_sub.update(&_vehicle_status)) {
		// _vehicle_attitude_valid = (_time_now_usec - vehicle_attitude.timestamp < 1_s) && (vehicle_attitude.timestamp > 0);
	}

	if (_vtol_vehicle_status_sub.update(&_vtol_vehicle_status)) {
		// _vehicle_attitude_valid = (_time_now_usec - vehicle_attitude.timestamp < 1_s) && (vehicle_attitude.timestamp > 0);
	}
}

void AirspeedModule::update_wind_estimator_sideslip()
{
	bool att_valid = true;
	const hrt_abstime time_now_usec = hrt_absolute_time();

	/* update wind and airspeed estimator */
	_wind_estimator_sideslip.update(time_now_usec);

	if (_vehicle_local_position_valid && att_valid) {
		Vector3f vI(_vehicle_local_position.vx, _vehicle_local_position.vy, _vehicle_local_position.vz);
		Quatf q(_vehicle_attitude.q);

		/* sideslip fusion */
		_wind_estimator_sideslip.fuse_beta(time_now_usec, vI, q);
	}

	/* fill message for publishing later */
	_wind_estimate_sideslip.timestamp = time_now_usec;
	float wind[2];
	_wind_estimator_sideslip.get_wind(wind);
	_wind_estimate_sideslip.windspeed_north = wind[0];
	_wind_estimate_sideslip.windspeed_east = wind[1];
	float wind_cov[2];
	_wind_estimator_sideslip.get_wind_var(wind_cov);
	_wind_estimate_sideslip.variance_north = wind_cov[0];
	_wind_estimate_sideslip.variance_east = wind_cov[1];
	_wind_estimate_sideslip.tas_innov = _wind_estimator_sideslip.get_tas_innov();
	_wind_estimate_sideslip.tas_innov_var = _wind_estimator_sideslip.get_tas_innov_var();
	_wind_estimate_sideslip.beta_innov = _wind_estimator_sideslip.get_beta_innov();
	_wind_estimate_sideslip.beta_innov_var = _wind_estimator_sideslip.get_beta_innov_var();
	_wind_estimate_sideslip.tas_scale = _wind_estimator_sideslip.get_tas_scale();
}

void AirspeedModule::update_ground_minus_wind_airspeed()
{
	/* calculate airspeed estimate based on groundspeed-windspeed to use as fallback */
	float TAS_north = _vehicle_local_position.vx - _wind_estimate_sideslip.windspeed_north;
	float TAS_east = _vehicle_local_position.vy - _wind_estimate_sideslip.windspeed_east;
	float TAS_down = _vehicle_local_position.vz; // no wind estimate in z
	_ground_minus_wind_TAS = sqrtf(TAS_north * TAS_north + TAS_east * TAS_east + TAS_down * TAS_down);
	_ground_minus_wind_EAS = calc_EAS_from_TAS(_ground_minus_wind_TAS, _vehicle_air_data.baro_pressure_pa,
				 _vehicle_air_data.baro_temp_celcius);
}


void AirspeedModule::select_airspeed_and_publish()
{
	/* airspeed index:
	/  0: first airspeed sensor valid
	/  1: second airspeed sensor valid
	/ -1: airspeed sensor(s) invalid, but groundspeed-windspeed estimate valid
	/ -2: airspeed invalid (sensors and groundspeed-windspeed estimate invalid)
	*/
	bool find_new_valid_index = false;

	/* find new valid index if airspeed currently invalid (but we have sensors) */
	if ((_number_of_airspeed_sensors > 0 && _prev_airspeed_index < 0) ||
	    (_prev_airspeed_index >= 0 && !_airspeed_validator[_prev_airspeed_index].get_airspeed_valid()) ||
	    _prev_airspeed_index == -2) {

		find_new_valid_index = true;
	}

	if (find_new_valid_index) {
		_valid_airspeed_index = -1;

		for (int i = 0; i < _number_of_airspeed_sensors; i++) {
			if (_airspeed_validator[i].get_airspeed_valid()) {
				_valid_airspeed_index = i;
				break;
			}
		}
	}

	if (_valid_airspeed_index < 0 && !_vehicle_local_position_valid) {
		_valid_airspeed_index = -2;
	}

	/* publish critical message (and log) in index has changed */
	if (_valid_airspeed_index != _prev_airspeed_index) {
		mavlink_log_critical(&_mavlink_log_pub, "Airspeed estimation: Switched from sensor %i to %i", _prev_airspeed_index,
				     _valid_airspeed_index);
	}

	_prev_airspeed_index = _valid_airspeed_index;

	/* fill out airspeed_validated message for publishing it */
	airspeed_validated_s airspeed_validated = {};
	airspeed_validated.timestamp = hrt_absolute_time();
	airspeed_validated.ground_minus_wind = _ground_minus_wind_TAS;
	airspeed_validated.selected_airspeed_index = _valid_airspeed_index;

	switch (_valid_airspeed_index) {
	case -2:
		airspeed_validated.airspeed_sensor_measurement_valid = false;
		airspeed_validated.airspeed_valid = false;
		airspeed_validated.indicated_airspeed_m_s = 0.0f;
		airspeed_validated.equivalent_airspeed_m_s = 0.0f;
		airspeed_validated.true_airspeed_m_s = 0.0f;
		break;

	case -1:
		airspeed_validated.airspeed_sensor_measurement_valid = false;
		airspeed_validated.airspeed_valid = true;
		airspeed_validated.indicated_airspeed_m_s = _ground_minus_wind_EAS;
		airspeed_validated.equivalent_airspeed_m_s = _ground_minus_wind_EAS;
		airspeed_validated.true_airspeed_m_s = _ground_minus_wind_TAS;
		break;

	default:
		airspeed_validated.airspeed_sensor_measurement_valid = true;
		airspeed_validated.airspeed_valid = true; // whats a good check for the validity of ground-wind speed?
		airspeed_validated.indicated_airspeed_m_s = _airspeed_validator[_valid_airspeed_index].get_IAS();
		airspeed_validated.equivalent_airspeed_m_s = _airspeed_validator[_valid_airspeed_index].get_EAS();
		airspeed_validated.true_airspeed_m_s = _airspeed_validator[_valid_airspeed_index].get_TAS();
		break;
	}

	/* publish airspeed validated topic */
	int instance;
	orb_publish_auto(ORB_ID(airspeed_validated), &_airspeed_validated_pub, &airspeed_validated, &instance,
			 ORB_PRIO_DEFAULT);

	/* publish sideslip-only-fusion wind topic */
	orb_publish_auto(ORB_ID(wind_estimate), &_wind_est_pub[0], &_wind_estimate_sideslip, &instance, ORB_PRIO_DEFAULT);

	/* publish the wind estimator states from all airspeed validators */
	for (int i = 0; i < _number_of_airspeed_sensors; i++) {
		wind_estimate_s wind_est = _airspeed_validator[i].get_wind_estimator_states(hrt_abstime());
		orb_publish_auto(ORB_ID(wind_estimate), &_wind_est_pub[i + 1], &wind_est, &instance, ORB_PRIO_DEFAULT);
	}

}

int AirspeedModule::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		int ret = AirspeedModule::task_spawn(argc, argv);

		if (ret) {
			return ret;
		}
	}

	return print_usage("unknown command");
}

int AirspeedModule::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module provides a single airspeed_validated topic, containing an indicated (IAS),
equivalend (EAS), true airspeed (TAS) and the information if the estimation currently
is invalid or if it is based on groundspeed minus windspeed. Supporting the input of multiple
"raw" airspeed inputs, this module automatically switches to a valid sensor in case
of failure detection. For failure detection as well as for the estimation of a scale factor
from IAS to EAS, it runns several wind estimators and also publishes those.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("airspeed_estimator", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

// int AirspeedModule::print_status()
// {
// 	perf_print_counter(_perf_elapsed);
// 	perf_print_counter(_perf_interval);
//
// 	if (_instance > -1) {
// 		uORB::SubscriptionData<wind_estimate_s> est{ORB_ID(wind_estimate), (uint8_t)_instance};
// 		est.update();
//
// 		print_message(est.get());
// 	} else {
// 		PX4_INFO("Running, but never published");
// 	}
//
// 	return 0;
// }

extern "C" __EXPORT int wind_estimator_main(int argc, char *argv[]);

int
wind_estimator_main(int argc, char *argv[])
{
	return AirspeedModule::main(argc, argv);
}
