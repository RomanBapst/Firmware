#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>

#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/terrain_estimate.h>
#include <uORB/topics/parameter_update.h>

#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/circuit_breaker.h>
#include <lib/mathlib/mathlib.h>
#include <lib/geo/geo.h>


extern "C" __EXPORT int terrain_estimator_main(int argc, char *argv[]);

class TerrainEstimator
{
public:
	TerrainEstimator();
	~TerrainEstimator();

	int start();

	void show_values();

private:
	enum {n_x=3};
	enum {n_u=1};

	bool	_task_should_exit;
	int		_control_task;

	int _sensor_sub;
	int _gps_sub;
	int _attitude_sub;
	int _distance_sub;
	int _armed_sub;
	int _status_sub;
	int _params_sub;

	struct sensor_combined_s _sensors;
	struct vehicle_gps_position_s _gps;
	struct vehicle_attitude_s _att;
	struct distance_sensor_s _distance;
	struct actuator_armed_s _armed;
	struct vehicle_status_s _status;

	struct terrain_estimate_s _terrain;

	orb_advert_t _terrain_alt_pub;

	float _distance_filtered;
	float _distance_last;

	// kalman filter variables
	math::Vector<3> _x;		// state: ground distance, velocity
	float  _u_z;			// acceleration in earth z direction
	math::Matrix<3, 3> _P;	// covariance matrix

	// timestamps
	uint64_t _time_last_lidar;
	uint64_t _time_last_gps;

	struct {
		float var_acc_z;
		float var_p_z;
		float var_p_vz;
		float var_lidar;
		float var_gps_vz;
	} _params;

	struct {
		param_t var_acc_z;
		param_t var_p_z;
		param_t var_p_vz;
		param_t var_lidar;
		param_t var_gps_vz;
	} _param_handle;

	void task_main();
	static void task_main_trampoline(int argc, char *argv[]);
	void update_subscriptions();
	void predict(float dt);
	void measurement_update();
	void publish();
	void update_parameters();
	void parameter_update_poll();

};

namespace terrain_estimator
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
static const int ERROR = -1;

TerrainEstimator	*g_control;
}

TerrainEstimator::TerrainEstimator() :
	_task_should_exit(false),
	_control_task(-1),
	_sensor_sub(0),
	_gps_sub(0),
	_attitude_sub(0),
	_distance_sub(0),
	_armed_sub(0),
	_status_sub(0),
	_params_sub(0),
	_sensors{},
	_gps{},
	_att{},
	_distance{},
	_armed{},
	_status{},
	_terrain{},
	_terrain_alt_pub(nullptr),
	_distance_filtered(0.0f),
	_distance_last(0.0f),
	_time_last_lidar(0),
	_time_last_gps(0)
{
	_x.zero();
	_u_z = 0.0f;
	_P.identity();

	_param_handle.var_acc_z = param_find("VAR_ACC_Z");
	_param_handle.var_p_z = param_find("VAR_P_Z");
	_param_handle.var_p_vz = param_find("VAR_P_VZ");
	_param_handle.var_lidar = param_find("VAR_LIDAR");
	_param_handle.var_gps_vz = param_find("VAR_GPS_VZ");
}

TerrainEstimator::~TerrainEstimator()
{
	if (_control_task != -1) {
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	terrain_estimator::g_control = nullptr;
}

void TerrainEstimator::show_values() {
	for(int i = 0; i< 40;i++) {
		warnx("dist: %.5f", (double)_terrain.dist_to_ground);
		warnx("off: %.5f", (double)_x(2));
		usleep(500000);
	}
}

void TerrainEstimator::update_parameters()
{
	param_get(_param_handle.var_acc_z, &_params.var_acc_z);
	param_get(_param_handle.var_p_z, &_params.var_p_z);
	param_get(_param_handle.var_p_vz, &_params.var_p_vz);
	param_get(_param_handle.var_lidar, &_params.var_lidar);
	param_get(_param_handle.var_gps_vz, &_params.var_gps_vz);
}

void
TerrainEstimator::parameter_update_poll()
{
	bool updated;

	/* Check if parameters have changed */
	orb_check(_params_sub, &updated);

	if (updated) {
		struct parameter_update_s param_update;
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);
		update_parameters();
	}
}

void TerrainEstimator::publish()
{
	if (_terrain_alt_pub != nullptr) {
		orb_publish(ORB_ID(terrain_estimate), _terrain_alt_pub, &_terrain);
	} else {
		_terrain_alt_pub = orb_advertise(ORB_ID(terrain_estimate), &_terrain);
	}
}

void TerrainEstimator::update_subscriptions()
{
	bool updated;
	orb_check(_gps_sub, &updated);

	if (updated) {orb_copy(ORB_ID(vehicle_gps_position), _gps_sub, &_gps);}

	orb_check(_attitude_sub, &updated);

	if (updated) {orb_copy(ORB_ID(vehicle_attitude), _attitude_sub, &_att);}

	orb_check(_distance_sub, &updated);

	if (updated) {orb_copy(ORB_ID(distance_sensor), _distance_sub, &_distance);}

	orb_check(_armed_sub, &updated);

	if (updated) {orb_copy(ORB_ID(actuator_armed), _armed_sub, &_armed);}

	orb_check(_status_sub, &updated);

	if (updated) {orb_copy(ORB_ID(vehicle_status), _status_sub, &_status);}
}

void TerrainEstimator::predict(float dt)
{
	// check if should predict
	if (_distance.current_distance > 35.0f || _distance.current_distance < 0.00001f) {
		return;
	}

	if (_att.R_valid) {
		math::Matrix<3, 3> R_att(&_att.R[0]);
		math::Vector<3> a(&_sensors.accelerometer_m_s2[0]);
		math::Vector<3> u;
		u = R_att * a;
		_u_z = u(2) + 9.81f - _x(2); // compensate for gravity and offset

	} else {
		_u_z = -_x(2);	// only compensate for offset
	}

	// dynamics matrix
	math::Matrix<n_x, n_x> A;
	A.zero();
	A(0,1) = 1;
	A(1,2) = 1;

	// input matrix
	math::Matrix<n_x,1>  B;
	B.zero();
	B(1,0) = 1;

	// input noise variance
	float R = 0.135f;

	// process noise convariance
	math::Matrix<n_x, n_x>  Q;
	Q(0,0) = 0;
	Q(1,1) = 0;

	// do prediction
	math::Vector<n_x>  dx = (A * _x) * dt;
	dx(1) += B(1,0) * _u_z * dt;

	// propagate state and covariance matrix
	_x += dx;
	_P += (A * _P + _P * A.transposed() +
	       B * R * B.transposed() + Q) * dt;
}

void TerrainEstimator::measurement_update()
{
	_terrain.valid = false;
	if (_distance.timestamp > _time_last_lidar) {
		// check if measured value is sane
		if (_distance.current_distance < 35.0f || _distance.current_distance > 0.00001f) {
			_terrain.valid = true;
			// low pass filter distance measurement
			_distance_filtered = _distance.current_distance;
			float d = _distance_filtered;

			math::Matrix<1, n_x> C;
			C(0, 0) = -1; // measured altitude,

			float R = 0.009f;

			math::Vector<1> y;
			y(0) = d * cosf(_att.roll) * cosf(_att.pitch);

			// residual
			math::Matrix<1, 1> S_I = (C * _P * C.transposed());
			S_I(0,0) += R;
			S_I = S_I.inversed();
			math::Vector<1> r = y - C * _x;

			math::Matrix<n_x, 1> K = _P * C.transposed() * S_I;

			// some sort of outlayer rejection
			if (fabsf(_distance.current_distance - _distance_last) < 1.0f) {
				_x += K * r;
				_P -= K * C * _P;
			}
		}
	}
	_time_last_lidar = _distance.timestamp;
	_distance_last = _distance.current_distance;

	if (_gps.timestamp_position > _time_last_gps && _gps.satellites_used > 6) {
		math::Matrix<1, n_x> C;
		C(0,1) = 1;

		float R = 0.056f;

		math::Vector<1> y;
		y(0) = _gps.vel_d_m_s;

		// residual
		math::Matrix<1, 1> S_I = (C * _P * C.transposed());
		S_I(0,0) += R;
		S_I = S_I.inversed();
		math::Vector<1> r = y - C * _x;

		math::Matrix<n_x, 1> K = _P * C.transposed() * S_I;
		_x += K * r;
		_P -= K * C * _P;

		_time_last_gps = _gps.timestamp_position;
	}

}

void
TerrainEstimator::task_main()
{
	// subscribe to topics
	_sensor_sub = orb_subscribe(ORB_ID(sensor_combined));
	_gps_sub = orb_subscribe(ORB_ID(vehicle_gps_position));
	_attitude_sub = orb_subscribe(ORB_ID(vehicle_attitude));
	_distance_sub = orb_subscribe(ORB_ID(distance_sensor));
	_armed_sub = orb_subscribe(ORB_ID(actuator_armed));
	_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));

	// intialise parameters
	update_parameters();

	// run estimator on acceleration updates
	px4_pollfd_struct_t fds[1];

	fds[0].fd = _sensor_sub;
	fds[0].events = POLLIN;

	while (!_task_should_exit) {
		static uint64_t last_run = 0;
		float dt = (hrt_absolute_time() - last_run) / 1000000.0f;
		last_run = hrt_absolute_time();

		// guard against too small (< 2ms) and too large (> 20ms) dt's
		if (dt < 0.002f) {
			dt = 0.002f;
		} else if (dt > 0.02f) {
			dt = 0.02f;
		}

		// wait for up to 100ms for data
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		// timed out - periodic check for _task_should_exit
		if (pret == 0)
			continue;

		// this is undesirable but not much we can do - might want to flag unhappy status
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			// sleep a bit before next try
			usleep(100000);
			continue;
		}

		if (fds[0].revents & POLLIN) {
			orb_copy(ORB_ID(sensor_combined), _sensor_sub, &_sensors);
			
			// update the other topics
			update_subscriptions();

			// poll parameters
			parameter_update_poll();

			// do state prediciton
			predict(dt);

			// do measurement update
			measurement_update();

			_terrain.dist_to_ground = -_x(0);

			// publish terrain estimate
			publish();
		}
	}
 
}

void
TerrainEstimator::task_main_trampoline(int argc, char *argv[])
{
	terrain_estimator::g_control->task_main();
}

int
TerrainEstimator::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("terrain_estimator",
				       SCHED_DEFAULT,
				       SCHED_PRIORITY_MAX - 5,
				       1500,
				       (px4_main_t)&TerrainEstimator::task_main_trampoline,
				       nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}



int terrain_estimator_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: terrain_estimator {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (terrain_estimator::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

		terrain_estimator::g_control = new TerrainEstimator;

		if (terrain_estimator::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != terrain_estimator::g_control->start()) {
			delete terrain_estimator::g_control;
			terrain_estimator::g_control = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (terrain_estimator::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		delete terrain_estimator::g_control;
		terrain_estimator::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (terrain_estimator::g_control) {
			terrain_estimator::g_control->show_values();
			return 0;

		} else {
			warnx("not running");
			return 1;
		}
	}

	warnx("unrecognized command");
	return 1;
}
