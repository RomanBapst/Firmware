#include <systemlib/param/param.h>

/**
 * Variance of acceleration measurement in z direction
 *
 * @min 0.0
 * @max 1.0
 */
PARAM_DEFINE_FLOAT(VAR_ACC_Z, 0.135f);

/**
 * Process noise variance of estimated distance to ground (first state)
 *
 * @min 0.0
 * @max 1.0
 */
PARAM_DEFINE_FLOAT(VAR_P_Z, 0.0f);

/**
 * Process noise variance of estimated down velocity (second state)
 *
 * @min 0.0
 * @max 1.0
 */
PARAM_DEFINE_FLOAT(VAR_P_VZ, 0.0f);

/**
 * Variance of lidar range measurement
 *
 * @min 0.0
 * @max 1.0
 */
PARAM_DEFINE_FLOAT(VAR_LIDAR, 0.009f);

/**
 * Variance of GPS down velocity measurement
 *
 * @min 0.0
 * @max 1.0
 */
PARAM_DEFINE_FLOAT(VAR_GPS_VZ, 0.056f);