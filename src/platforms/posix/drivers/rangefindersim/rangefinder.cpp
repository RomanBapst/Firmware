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

#include "rangefinder.h"

#include <drivers/drv_range_finder.h>

RangeFinder::RangeFinder(const char *path) :
	VDev("RangeFinder", path),
	_work{},
	_reports(nullptr),
	_class_instance(-1),
	_orb_class_instance(-1),
	_pwmSub(-1),
	_distance(0.0f),
	_distance_sensor_topic(nullptr),
	_range{},
	_min_distance(LL40LS_MIN_DISTANCE),
	_max_distance(LL40LS_MAX_DISTANCE),
	_measure_ticks(0),
	_sample_perf(perf_alloc(PC_ELAPSED, "ll40ls_pwm_read")),
	_read_errors(perf_alloc(PC_COUNT, "ll40ls_pwm_read_errors")),
	_buffer_overflows(perf_alloc(PC_COUNT, "ll40ls_pwm_buffer_overflows")),
	_sensor_zero_resets(perf_alloc(PC_COUNT, "ll40ls_pwm_zero_resets"))
{
}

RangeFinder::~RangeFinder()
{
	stop();

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}

	if (_class_instance != -1) {
		unregister_class_devname(RANGE_FINDER_BASE_DEVICE_PATH, _class_instance);
	}

	/* free perf counters */
	perf_free(_sample_perf);
	perf_free(_buffer_overflows);
	perf_free(_sensor_zero_resets);
}


int RangeFinder::init()
{
	/* do regular cdev init */
	int ret = VDev::init();

	if (ret != OK) {
		return ERROR;
	}

	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(struct distance_sensor_s));

	if (_reports == nullptr) {
		return ERROR;
	}

	_class_instance = register_class_devname(RANGE_FINDER_BASE_DEVICE_PATH);

	if (_class_instance == CLASS_DEVICE_PRIMARY) {
		/* get a publish handle on the distance_sensor topic */
		struct distance_sensor_s ds_report;
		measure();
		_reports->get(&ds_report);
		_distance_sensor_topic = orb_advertise_multi(ORB_ID(distance_sensor), &ds_report,
							     &_orb_class_instance, ORB_PRIO_LOW);

		if (_distance_sensor_topic == nullptr) {
			DEVICE_DEBUG("failed to create distance_sensor object. Did you start uOrb?");
		}
	}

	return OK;
}

void RangeFinder::set_minimum_distance(const float min)
{
	_min_distance = min;
}

void RangeFinder::set_maximum_distance(const float max)
{
	_max_distance = max;
}

float RangeFinder::get_minimum_distance() const
{
	return _min_distance;
}

float RangeFinder::get_maximum_distance() const
{
	return _max_distance;
}

uint32_t RangeFinder::getMeasureTicks() const
{
	return _measure_ticks;
}

void RangeFinder::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_read_errors);
	perf_print_counter(_buffer_overflows);
	perf_print_counter(_sensor_zero_resets);
	warnx("poll interval:  %u ticks", getMeasureTicks());
	warnx("distance: %.3fm", (double)_range.current_distance);
}

void RangeFinder::print_registers()
{
	printf("Not supported in PWM mode\n");
}

void RangeFinder::start()
{
	/* schedule a cycle to start things */
	work_queue(HPWORK, &_work, (worker_t)&RangeFinder::cycle_trampoline, this, 1);
}

void RangeFinder::stop()
{
	work_cancel(HPWORK, &_work);
}

void RangeFinder::cycle_trampoline(void *arg)
{
	RangeFinder *dev = (RangeFinder *)arg;

	dev->cycle();
}

void RangeFinder::cycle()
{
	measure();

	/* schedule a fresh cycle call when the measurement is done */
	work_queue(HPWORK,
		   &_work,
		   (worker_t)&RangeFinder::cycle_trampoline,
		   this,
		   getMeasureTicks());
	return;
}

int RangeFinder::measure()
{
	perf_begin(_sample_perf);

	if (OK != collect()) {
		DEVICE_DEBUG("collection error");
		perf_count(_read_errors);
		perf_end(_sample_perf);
		return ERROR;
	}

	_range.timestamp = hrt_absolute_time();
	_range.type = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;
	_range.max_distance = get_maximum_distance();
	_range.min_distance = get_minimum_distance();
	_range.current_distance = _distance;
	_range.covariance = 0.0f;
	_range.orientation = 8;
	/* TODO: set proper ID */
	_range.id = 0;

	/* Due to a bug in older versions of the RangeFinder firmware, we have to reset sensor on (distance == 0) */
	if (_range.current_distance <= 0.0f) {
		perf_count(_sensor_zero_resets);
		perf_end(_sample_perf);
		return reset_sensor();
	}

	if (_distance_sensor_topic != nullptr) {
		orb_publish(ORB_ID(distance_sensor), _distance_sensor_topic, &_range);
	}

	if (_reports->force(&_range)) {
		perf_count(_buffer_overflows);
	}

	poll_notify(POLLIN);
	perf_end(_sample_perf);
	return OK;
}

ssize_t RangeFinder::read(struct file *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct distance_sensor_s);
	struct distance_sensor_s *rbuf = reinterpret_cast<struct distance_sensor_s *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (getMeasureTicks() > 0) {
		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the workq thread while we are doing this;
		 * we are careful to avoid racing with them.
		 */
		while (count--) {
			if (_reports->get(rbuf)) {
				ret += sizeof(*rbuf);
				rbuf++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;

	} else {

		_reports->flush();
		measure();

		if (_reports->get(rbuf)) {
			ret = sizeof(*rbuf);
		}
	}

	return ret;
}

int
RangeFinder::ioctl(struct file *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_measure_ticks = 0;
				return OK;

			/* external signalling (DRDY) not supported */
			case SENSOR_POLLRATE_EXTERNAL:

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_ticks = USEC2TICK(LL40LS_CONVERSION_INTERVAL);

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* convert hz to tick interval via microseconds */
					unsigned ticks = USEC2TICK(1000000 / arg);

					/* check against maximum rate */
					if (ticks < USEC2TICK(LL40LS_CONVERSION_INTERVAL)) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					_measure_ticks = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_measure_ticks == 0) {
			return SENSOR_POLLRATE_MANUAL;
		}

		return (1000 / _measure_ticks);

	case SENSORIOCRESET:
		reset_sensor();
		return OK;

	// case RANGEFINDERIOCSETMINIUMDISTANCE: {
	// 		set_minimum_distance(*(float *)arg);
	// 		return OK;
	// 	}
	// 	break;

	// case RANGEFINDERIOCSETMAXIUMDISTANCE: {
	// 		set_maximum_distance(*(float *)arg);
	// 		return OK;
	// 	}
	// 	break;

	default:
		return -EINVAL;
	}
}

int RangeFinder::collect()
{
	// Simulator *sim = Simulator::getInstance();
	// if (sim == NULL) {
	// 	PX4_ERR("Error BARO_SIM::transfer no simulator");
	// 	return -ENODEV;
	// }
	// PX4_DEBUG("BARO_SIM::transfer getting sample");
	// sim->getDistanceSample(&_distance, sizeof(_distance));
	_distance = 5.0f;
	return OK;
}

int RangeFinder::reset_sensor()
{
	return OK;
}
