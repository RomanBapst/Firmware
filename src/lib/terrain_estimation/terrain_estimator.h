#include <lib/mathlib/mathlib.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/distance_sensor.h>

class TerrainEstimator
{
public:
	TerrainEstimator();
	~TerrainEstimator();

	bool is_valid() {return _terrain_valid;}
	float get_distance_to_ground() {return _x(0);}

private:
	enum {n_x=3};
	enum {n_u=1};


	float _distance_filtered;
	float _distance_last;
	bool _terrain_valid;

	// pointers to relevant data
	struct vehicle_attitude_s *_attitude;
	struct sensor_combined_s *_sensor;
	struct distance_sensor_s *_distance;
	struct vehicle_gps_position_s *_gps;

	// kalman filter variables
	math::Vector<n_x> _x;		// state: ground distance, velocity, accel bias in z direction
	float  _u_z;			// acceleration in earth z direction
	math::Matrix<3, 3> _P;	// covariance matrix

	// timestamps
	uint64_t _time_last_distance;
	uint64_t _time_last_gps;

	struct {
		float var_acc_z;
		float var_p_z;
		float var_p_vz;
		float var_lidar;
		float var_gps_vz;
	} _params;


	void predict(float dt, const struct vehicle_attitude_s *attitude, const struct sensor_combined_s *sensor);
	void measurement_update(const struct vehicle_gps_position_s *gps, const struct distance_sensor_s *distance,
				const struct vehicle_attitude_s *attitude);

};