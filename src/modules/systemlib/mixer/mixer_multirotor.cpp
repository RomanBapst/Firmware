/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file mixer_multirotor.cpp
 *
 * Multi-rotor mixers.
 */
#include <uORB/uORB.h>
#include <uORB/topics/multirotor_motor_limits.h>
#include <nuttx/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <math.h>

#include "mixer.h"

// This file is generated by the multi_tables script which is invoked during the build process
#include "mixer_multirotor.generated.h"

#define debug(fmt, args...)	do { } while(0)
//#define debug(fmt, args...)	do { printf("[mixer] " fmt "\n", ##args); } while(0)
//#include <debug.h>
//#define debug(fmt, args...)	lowsyslog(fmt "\n", ##args)

/*
 * Clockwise: 1
 * Counter-clockwise: -1
 */

namespace
{

float constrain(float val, float min, float max)
{
	return (val < min) ? min : ((val > max) ? max : val);
}

} // anonymous namespace

MultirotorMixer::MultirotorMixer(ControlCallback control_cb,
				 uintptr_t cb_handle,
				 MultirotorGeometry geometry,
				 float roll_scale,
				 float pitch_scale,
				 float yaw_scale,
				 float idle_speed) :
	Mixer(control_cb, cb_handle),
	_roll_scale(roll_scale),
	_pitch_scale(pitch_scale),
	_yaw_scale(yaw_scale),
	_idle_speed(-1.0f + idle_speed * 2.0f),	/* shift to output range here to avoid runtime calculation */
	_limits_pub(),
	_rotor_count(_config_rotor_count[(MultirotorGeometryUnderlyingType)geometry]),
	_rotors(_config_index[(MultirotorGeometryUnderlyingType)geometry])
{
}

MultirotorMixer::~MultirotorMixer()
{
}

MultirotorMixer *
MultirotorMixer::from_text(Mixer::ControlCallback control_cb, uintptr_t cb_handle, const char *buf, unsigned &buflen)
{
	MultirotorGeometry geometry;
	char geomname[8];
	int s[4];
	int used;

	/* enforce that the mixer ends with space or a new line */
	for (int i = buflen - 1; i >= 0; i--) {
		if (buf[i] == '\0')
			continue;

		/* require a space or newline at the end of the buffer, fail on printable chars */
		if (buf[i] == ' ' || buf[i] == '\n' || buf[i] == '\r') {
			/* found a line ending or space, so no split symbols / numbers. good. */
			break;
		} else {
			debug("simple parser rejected: No newline / space at end of buf. (#%d/%d: 0x%02x)", i, buflen-1, buf[i]);
			return nullptr;
		}

	}

	if (sscanf(buf, "R: %s %d %d %d %d%n", geomname, &s[0], &s[1], &s[2], &s[3], &used) != 5) {
		debug("multirotor parse failed on '%s'", buf);
		return nullptr;
	}

	if (used > (int)buflen) {
		debug("OVERFLOW: multirotor spec used %d of %u", used, buflen);
		return nullptr;
	}

	buf = skipline(buf, buflen);
	if (buf == nullptr) {
		debug("no line ending, line is incomplete");
		return nullptr;
	}

	debug("remaining in buf: %d, first char: %c", buflen, buf[0]);

	if (!strcmp(geomname, "4+")) {
		geometry = MultirotorGeometry::QUAD_PLUS;

	} else if (!strcmp(geomname, "4x")) {
		geometry = MultirotorGeometry::QUAD_X;

	} else if (!strcmp(geomname, "4v")) {
		geometry = MultirotorGeometry::QUAD_V;

	} else if (!strcmp(geomname, "4w")) {
		geometry = MultirotorGeometry::QUAD_WIDE;

	} else if (!strcmp(geomname, "4dc")) {
		geometry = MultirotorGeometry::QUAD_DEADCAT;

	} else if (!strcmp(geomname, "6+")) {
		geometry = MultirotorGeometry::HEX_PLUS;

	} else if (!strcmp(geomname, "6x")) {
		geometry = MultirotorGeometry::HEX_X;

	} else if (!strcmp(geomname, "6c")) {
		geometry = MultirotorGeometry::HEX_COX;

	} else if (!strcmp(geomname, "8+")) {
		geometry = MultirotorGeometry::OCTA_PLUS;

	} else if (!strcmp(geomname, "8x")) {
		geometry = MultirotorGeometry::OCTA_X;
		
	} else if (!strcmp(geomname, "8c")) {
		geometry = MultirotorGeometry::OCTA_COX;

	} else if (!strcmp(geomname, "2-")) {
		geometry = MultirotorGeometry::TWIN_ENGINE;

	} else if (!strcmp(geomname, "3y")) {
		geometry = MultirotorGeometry::TRI_Y;

	} else {
		debug("unrecognised geometry '%s'", geomname);
		return nullptr;
	}

	debug("adding multirotor mixer '%s'", geomname);

	return new MultirotorMixer(
		       control_cb,
		       cb_handle,
		       geometry,
		       s[0] / 10000.0f,
		       s[1] / 10000.0f,
		       s[2] / 10000.0f,
		       s[3] / 10000.0f);
}

unsigned
MultirotorMixer::mix(float *outputs, unsigned space)
{
	float		roll    = constrain(get_control(0, 0) * _roll_scale, -1.0f, 1.0f);
	//lowsyslog("roll: %d, get_control0: %d, %d\n", (int)(roll), (int)(get_control(0, 0)), (int)(_roll_scale));
	float		pitch   = constrain(get_control(0, 1) * _pitch_scale, -1.0f, 1.0f);
	float		yaw     = constrain(get_control(0, 2) * _yaw_scale, -1.0f, 1.0f);
	float		thrust  = constrain(get_control(0, 3), 0.0f, 1.0f);
	//lowsyslog("thrust: %d, get_control3: %d\n", (int)(thrust), (int)(get_control(0, 3)));
	float		min_out = 0.0f;
	float		max_out = 0.0f;

	_limits.roll_pitch = false;
	_limits.yaw = false;
	_limits.throttle_upper = false;
	_limits.throttle_lower = false;

	/* perform initial mix pass yielding unbounded outputs, ignore yaw */
	for (unsigned i = 0; i < _rotor_count; i++) {
		float out = roll * _rotors[i].roll_scale +
			    pitch * _rotors[i].pitch_scale +
			    thrust;

		out *= _rotors[i].out_scale;

		/* calculate min and max output values */
		if (out < min_out) {
			min_out = out;
		}
		if (out > max_out) {
			max_out = out;
		}

		outputs[i] = out;
	}

	/* scale down roll/pitch controls if some outputs are negative, don't add yaw, keep total thrust */
	if (min_out < 0.0f) {
		float scale_in = thrust / (thrust - min_out);

		max_out = 0.0f;

		/* mix again with adjusted controls */
		for (unsigned i = 0; i < _rotor_count; i++) {
			float out = scale_in * (roll * _rotors[i].roll_scale + pitch * _rotors[i].pitch_scale) + thrust;

			/*  adjust yaw if it will lead to negative output values */
			if (out + yaw * _rotors[i].yaw_scale < 0.0f) {
				yaw = -out / _rotors[i].yaw_scale;
			}

			outputs[i] = out;
		}
		_limits.roll_pitch = true;
	} else {
		/* scale yaw if it caused output clipping */
		for (unsigned i = 0; i < _rotor_count; i++) {
			float out = roll * _rotors[i].roll_scale + pitch * _rotors[i].pitch_scale +
				+ yaw * _rotors[i].yaw_scale + thrust;

			if(out < 0.0f) {
				yaw = -out / _rotors[i].yaw_scale;
			}
		}
	}

	/* now add yaw */
	for (unsigned i = 0; i < _rotor_count; i++) {
		outputs[i] += yaw * _rotors[i].yaw_scale;

		/* update max output value */
		if (outputs[i] > max_out) {
			max_out = out;
		}
	}

	/* scale down all outputs if some outputs are too large, reduce total thrust */
	float scale_out;
	if (max_out > 1.0f) {
		scale_out = 1.0f / max_out;
		_limits.throttle_upper = true;

	} else {
		scale_out = 1.0f;
	}

	/* scale outputs to range _idle_speed..1, and do final limiting */
	for (unsigned i = 0; i < _rotor_count; i++) {
		if (outputs[i] < _idle_speed) {
			_limits.throttle_lower = true;
		}
		outputs[i] = constrain(_idle_speed + (outputs[i] * (1.0f - _idle_speed) * scale_out), _idle_speed, 1.0f);
	}

#if defined(CONFIG_ARCH_BOARD_PX4FMU_V1) || defined(CONFIG_ARCH_BOARD_PX4FMU_V2)
        /* publish/advertise motor limits if running on FMU */
        if (_limits_pub > 0) {
            orb_publish(ORB_ID(multirotor_motor_limits), _limits_pub, &_limits);
        } else {
            _limits_pub = orb_advertise(ORB_ID(multirotor_motor_limits), &_limits);
        }
#endif
	return _rotor_count;
}

void
MultirotorMixer::groups_required(uint32_t &groups)
{
	/* XXX for now, hardcoded to indexes 0-3 in control group zero */
	groups |= (1 << 0);
}

