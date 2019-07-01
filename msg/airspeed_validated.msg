uint64 timestamp			# time since system start (microseconds)

float32 indicated_airspeed_m_s		# indicated airspeed in m/s (IAS)

float32 equivalent_airspeed_m_s     # equivalent airspeed in m/s (accounts for instrumentation errors) (EAS)

float32 true_airspeed_m_s		# true filtered airspeed in m/s (TAS)

float32 ground_minus_wind # TAS calculated from groundspeed - windspeed

bool airspeed_sensor_measurement_valid # true if airspeed measurement from (one or multiple) pitot is valid

bool airspeed_valid # true if airspeed_sensor_measurement_valid or groundspeed-windspeed is valid

int64 selected_airspeed_index # 0-2: airspeed sensor index, -1: groundspeed-windspeed, -2: airspeed invalid
