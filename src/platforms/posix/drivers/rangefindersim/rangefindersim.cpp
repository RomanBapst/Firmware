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
 * @file rangefindersim.cpp
 * @author Allyson Kreft
 * @author Johan Jansen <jnsn.johan@gmail.com>
 * @author Ban Siesta <bansiesta@gmail.com>
 * @author James Goppert <james.goppert@gmail.com>
 *
 * Interface for the PulsedLight Lidar-Lite range finders.
 */
#include <px4_config.h>
#include <systemlib/err.h>
#include <fcntl.h>
#include <cstdlib>
#include <string.h>
#include <stdio.h>
#include "rangefinder.h"

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

#define RANGEFINDER_SIM_PATH "/dev/rangefindersim_sim"


/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int rangefindersim_main(int argc, char *argv[]);


/**
 * Local functions in support of the shell command.
 */
namespace rangefindersim
{

/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
const int ERROR = -1;

RangeFinder	*g_dev;

int	start();
int	stop();
int	test();
int	reset();
int	info();
int regdump();
int	usage();

/**
 * Start the driver.
 */
int start()
{
	if (g_dev != nullptr) {
		PX4_ERR("driver already started");
	}


	g_dev = new RangeFinder(RANGEFINDER_SIM_PATH);

	if (g_dev != nullptr && OK != g_dev->init()) {
		delete g_dev;
		g_dev = nullptr;
	}

	/* set the poll rate to default, starts automatic data collection */
	if (g_dev != nullptr) {
		int fd = px4_open(RANGEFINDER_SIM_PATH, O_RDONLY);

		if (fd == -1) {
			goto fail;
		}

		int ret = px4_ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT);
		close(fd);

		if (ret < 0) {
			goto fail;
		}
	}

	return 0;

fail:
	PX4_ERR("driver start failed");
	return 1;
}

/**
 * Stop the driver
 */
int stop()
{
	
	if (g_dev != nullptr)  {
		delete g_dev;
		g_dev = nullptr;
	}

	return 0;
}

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */
int
test()
{
	// struct distance_sensor_s report;
	// ssize_t sz;
	// int ret;

	// const char *path;

	// path = rangefindersim_SIM_PATH;

	// int fd = px4_open(path, O_RDONLY);

	// if (fd < 0) {
	// 	err(1, "%s open failed, is the driver running?", path);
	// }

	// /* do a simple demand read */
	// sz = read(fd, &report, sizeof(report));

	// if (sz != sizeof(report)) {
	// 	err(1, "immediate read failed");
	// }

	// warnx("single read");
	// warnx("measurement: %0.2f m", (double)report.current_distance);
	// warnx("time:        %lld", report.timestamp);

	// /* start the sensor polling at 2Hz */
	// if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2)) {
	// 	errx(1, "failed to set 2Hz poll rate");
	// }

	// /* read the sensor 5 times and report each value */
	// for (unsigned i = 0; i < 5; i++) {
	// 	struct pollfd fds;

	// 	/* wait for data to be ready */
	// 	fds.fd = fd;
	// 	fds.events = POLLIN;
	// 	ret = poll(&fds, 1, 2000);

	// 	if (ret != 1) {
	// 		errx(1, "timed out waiting for sensor data");
	// 	}

	// 	/* now go get it */
	// 	sz = read(fd, &report, sizeof(report));

	// 	if (sz != sizeof(report)) {
	// 		err(1, "periodic read failed");
	// 	}

	// 	warnx("periodic read %u", i);
	// 	warnx("valid %u", (float)report.current_distance > report.min_distance
	// 		&& (float)report.current_distance < report.max_distance ? 1 : 0);
	// 	warnx("measurement: %0.3f m", (double)report.current_distance);
	// 	warnx("time:        %lld", report.timestamp);
	// }

	// /* reset the sensor polling to default rate */
	// if (OK != ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT)) {
	// 	errx(1, "failed to set default poll rate");
	// }

	//errx(0, "PASS");
	return 0;
}

/**
 * Reset the driver.
 */
int
reset()
{

	// const char *path;

	// if (use_i2c) {
	// 	path = ((bus == PX4_I2C_BUS_ONBOARD) ? rangefindersim_DEVICE_PATH_INT : rangefindersim_DEVICE_PATH_EXT);

	// } else {
	// 	path = rangefindersim_DEVICE_PATH_PWM;
	// }

	// int fd = open(path, O_RDONLY);

	// if (fd < 0) {
	// 	err(1, "failed ");
	// }

	// if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
	// 	err(1, "driver reset failed");
	// }

	// if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
	// 	err(1, "driver poll restart failed");
	// }

	// exit(0);
	return 0;
}

/**
 * Print a little info about the driver.
 */
int
info()
{
	// LidarLite * g_dev = get_dev(use_i2c, bus);
	// printf("state @ %p\n", g_dev);
	// g_dev->print_info();
	// exit(0);
	return 0;
}

/**
 * Dump registers
 */
int
regdump()
{
	// LidarLite * g_dev = get_dev(use_i2c, bus);
	// printf("regdump @ %p\n", g_dev);
	// g_dev->print_registers();
	// exit(0);
	return 0;
}

int
usage()
{
	warnx("missing command: try 'start', 'stop', 'info', 'test', 'reset', 'info' or 'regdump' [i2c|pwm]");
	return 0;
}

} // namespace

int
rangefindersim_main(int argc, char *argv[])
{

	const char *verb = argv[1];
	int ret;

	/* Start/load the driver. */
	if (!strcmp(verb, "start")) {
		ret = rangefindersim::start();
	}

	/* Stop the driver */
	if (!strcmp(verb, "stop")) {
		ret = rangefindersim::stop();
	}

	/* Test the driver/device. */
	else if (!strcmp(verb, "test")) {
		ret = rangefindersim::test();
	}

	/* Reset the driver. */
	else if (!strcmp(verb, "reset")) {
		ret = rangefindersim::reset();
	}

	/* dump registers */
	else if (!strcmp(verb, "regdump")) {
		ret = rangefindersim::regdump();
	}

	/* Print driver information. */
	else if (!strcmp(verb, "info") || !strcmp(verb, "status")) {
		ret = rangefindersim::info();
	}

	else {
		ret = rangefindersim::usage();
	}

	warnx("unrecognized command, try 'start', 'test', 'reset', 'info' or 'regdump'");
	rangefindersim::usage();
	return ret;
}
