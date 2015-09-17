#include "terrain_estimator.h"

TerrainEstimator::TerrainEstimator() :
	_distance_filtered(0.0f),
	_distance_last(0.0f),
	_terrain_valid(false),
	_time_last_distance(0),
	_time_last_gps(0)
{
	_x.zero();
	_u_z = 0.0f;
	_P.identity();
}

void TerrainEstimator::update_state(const struct vehicle_attitude_s *attitude, const struct sensor_combined_s *sensor,
		const struct distance_sensor_s *distance, const struct vehicle_gps_position_s *gps)
{
	// update data pointers
	_attitude = attitude;
	_sensor = sensor;
	_distance = distance;
	_gps = gps;

	// reinitialise filter if we find bad data
	bool reinit = false;
	for (int i = 0; i < n_x; i++) {
		if (!PX4_ISFINITE(_x(i))) {
			reinit = true;
		}
	}

	for (int i = 0; i < n_x; i++) {
		for (int j = 0; j < n_x; j++) {
			if (!PX4_ISFINITE(_P(i)(j))) {
				reinit = true;
			}
		}
	}

	if (reinit) {
		_x.zero();
		_P.zero();
		_P(0,0) = _P(1,1) = P(2,2) = 0.1f;
	}
}


void TerrainEstimator::predict(float dt)
{
	// check if should predict
	if (_distance->current_distance > 35.0f || _distance->current_distance < 0.00001f) {
		return;
	}

	if (attitude->R_valid) {
		math::Matrix<3, 3> R_att(attitude->R);
		math::Vector<3> a(&sensor->accelerometer_m_s2[0]);
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
	_terrain_valid = false;
	if (distance->timestamp > _time_last_distance) {
		// check if measured value is sane
		if (distance->current_distance < 35.0f || distance->current_distance > 0.00001f) {
			_terrain_valid = true;
			// low pass filter distance measurement
			_distance_filtered = distance->current_distance;
			float d = _distance_filtered;

			math::Matrix<1, n_x> C;
			C(0, 0) = -1; // measured altitude,

			float R = 0.009f;

			math::Vector<1> y;
			y(0) = d * cosf(attitude->roll) * cosf(attitude->pitch);

			// residual
			math::Matrix<1, 1> S_I = (C * _P * C.transposed());
			S_I(0,0) += R;
			S_I = S_I.inversed();
			math::Vector<1> r = y - C * _x;

			math::Matrix<n_x, 1> K = _P * C.transposed() * S_I;

			// some sort of outlayer rejection
			if (fabsf(distance->current_distance - _distance_last) < 1.0f) {
				_x += K * r;
				_P -= K * C * _P;
			}
		}
	}
	_time_last_distance = distance->timestamp;
	_distance_last = distance->current_distance;

	if (gps->timestamp_position > _time_last_gps && gps->satellites_used > 6) {
		math::Matrix<1, n_x> C;
		C(0,1) = 1;

		float R = 0.056f;

		math::Vector<1> y;
		y(0) = gps->vel_d_m_s;

		// residual
		math::Matrix<1, 1> S_I = (C * _P * C.transposed());
		S_I(0,0) += R;
		S_I = S_I.inversed();
		math::Vector<1> r = y - C * _x;

		math::Matrix<n_x, 1> K = _P * C.transposed() * S_I;
		_x += K * r;
		_P -= K * C * _P;

		_time_last_gps = gps->timestamp_position;
	}

}
