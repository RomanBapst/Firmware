/****************************************************************************
 *
 *   Copyright (c) 2014, 2015 PX4 Development Team. All rights reserved.
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
 * @file rangefinder.cpp
 * @author Johan Jansen <jnsn.johan@gmail.com>
 * @author Ban Siesta <bansiesta@gmail.com>
 *
 * Driver for the PulsedLight Lidar-Lite range finders connected via PWM.
 *
 * This driver accesses the pwm_input published by the pwm_input driver.
 */


#include <px4_workqueue.h>
#include <drivers/device/device.h>

#include <drivers/device/ringbuffer.h>
#include <systemlib/perf_counter.h>

#include <uORB/uORB.h>
#include <uORB/topics/pwm_input.h>
#include <uORB/topics/distance_sensor.h>

#include <simulator/simulator.h>

/* Device limits */
#define LL40LS_MIN_DISTANCE (0.00f)
#define LL40LS_MAX_DISTANCE (60.00f)

// normal conversion wait time
#define LL40LS_CONVERSION_INTERVAL 50*1000UL /* 50ms */

// maximum time to wait for a conversion to complete.
#define LL40LS_CONVERSION_TIMEOUT 100*1000UL /* 100ms */


class RangeFinder : public device::VDev
{
public:
	RangeFinder(const char *path);
	virtual ~RangeFinder();

	int init() override;

	ssize_t read(struct file *filp, char *buffer, size_t buflen);
	int	ioctl(struct file *filp, int cmd, unsigned long arg);

	void start();

	void stop();

	void cycle();

	/**
	* @brief
	*   Diagnostics - print some basic information about the driver.
	*/
	void print_info();

	/**
	 * @brief
	 *   print registers to console
	 */
	void print_registers();

	/**
	* Static trampoline from the workq context; because we don't have a
	* generic workq wrapper yet.
	*
	* @param arg        Instance pointer for the driver that is polling.
	*/
	static void     cycle_trampoline(void *arg);

protected:

	int measure();

	int collect();

	int reset_sensor();

	void task_main_trampoline(int argc, char *argv[]);

	void set_minimum_distance(const float min);

	void set_maximum_distance(const float max);


	float get_minimum_distance() const;

	float get_maximum_distance() const;


	uint32_t getMeasureTicks() const;

private:
	work_s			_work;
	ringbuffer::RingBuffer	*_reports;
	int			_class_instance;
	int			_orb_class_instance;
	int			_pwmSub;
	float 		_distance;
	orb_advert_t	        _distance_sensor_topic;
	struct distance_sensor_s _range;

	float               _min_distance;
	float               _max_distance;
	uint32_t            _measure_ticks;

	perf_counter_t	        _sample_perf;
	perf_counter_t	        _read_errors;
	perf_counter_t	        _buffer_overflows;
	perf_counter_t	        _sensor_zero_resets;
};