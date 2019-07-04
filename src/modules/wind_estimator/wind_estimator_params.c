/**
 * Enable Airspeed Validator
 *
 * @boolean
 * @reboot_required true
 * @group Airspeed Validator
 */
PARAM_DEFINE_INT32(WEST_EN, 1);

/**
 * Airspeed Validator: Wind estimator wind process noise.
 *
 * @min 0
 * @max 1
 * @unit m/s/s
 * @group Airspeed Validator
 */
PARAM_DEFINE_FLOAT(ARSPV_W_P_NOISE, 0.1f);

/**
 * Airspeed Validator: Wind estimator true airspeed scale process noise.
 *
 * @min 0
 * @max 0.1
 * @group Airspeed Validator
 */
PARAM_DEFINE_FLOAT(ARSPV_SC_P_NOISE, 0.0001);

/**
 * Airspeed Validator: Wind estimator true airspeed measurement noise.
 *
 * @min 0
 * @max 4
 * @unit m/s
 * @group Airspeed Validator
 */
PARAM_DEFINE_FLOAT(ARSPV_TAS_NOISE, 1.4);

/**
 * Airspeed Validator: Wind estimator sideslip measurement noise.
 *
 * @min 0
 * @max 1
 * @unit rad
 * @group Airspeed Validator
 */
PARAM_DEFINE_FLOAT(ARSPV_BETA_NOISE, 0.3);

/**
 * Airspeed Validator: Gate size for true airspeed fusion.
 *
 * Sets the number of standard deviations used by the innovation consistency test.
 *
 * @min 1
 * @max 5
 * @unit SD
 * @group Airspeed Validator
 */
PARAM_DEFINE_INT32(ARSPV_TAS_GATE, 3);

/**
 * Airspeed Validator: Gate size for true sideslip fusion.
 *
 * Sets the number of standard deviations used by the innovation consistency test.
 *
 * @min 1
 * @max 5
 * @unit SD
 * @group Airspeed Validator
 */
PARAM_DEFINE_INT32(ARSPV_BETA_GATE, 1);

/**
 * Automatic airspeed scale estimation on
 *
 * Turns the automatic airspeed scale (scale from IAS to CAS/EAS) on or off.
 *
 * @boolean
 * @group Airspeed Validator
 */
PARAM_DEFINE_INT32(ARSPV_SCALE_EST, 0);

/**
 * Airspeed scale (scale from IAS to CAS/EAS)
 *
 * Scale can either be entered manually, or estimated in-flight by setting ARSPV_SCALE_EST to 1. It is recommended to only enable this when loitering (with little altitude changes, not high bank angles).
 *
 * @min 0.5
 * @max 1.5
 * @group Airspeed Validator
 */
PARAM_DEFINE_FLOAT(ARSPV_ARSP_SCALE, 1.0f);
