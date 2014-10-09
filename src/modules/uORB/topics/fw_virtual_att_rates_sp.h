/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
 * @file fw_virtual_att_rates_sp.h
 *
 * Virtual topic for fixed wing attitude rates setpoint
 *
 */

#ifndef TOPIC_FW_VIRTUAL_ATT_RATES_SP_H
#define TOPIC_FW_VIRTUAL_ATT_RATES_SP_H

#include <stdint.h>
#include "../uORB.h"

/**
 * @addtogroup topics
 * @{
 */

/** Attitude rates setpoint computed by fixed wing attitude controller */
struct vehicle_rates_setpoint_s {

	uint64_t timestamp; /**< in microseconds since system start */

	float roll;	/**< body angular rates in NED frame		*/
	float pitch;	/**< body angular rates in NED frame		*/
	float yaw;	/**< body angular rates in NED frame		*/
	float thrust;	/**< thrust normalized to 0..1			*/

	
};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(fw_virtual_att_rates_sp);

#endif
