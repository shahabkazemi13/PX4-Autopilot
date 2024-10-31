/****************************************************************************
 *
 * Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * NAU7802 force sensor enabled (external I2C)
 *
 * @reboot_required true
 * @group Sensors
 * @boolean
 */
PARAM_DEFINE_INT32(SENS_EN_NAU7802, 1);

/**
 * NAU7802 Force Sensor Tare on start
 *
 * @value 0 Tare on start
 * @value 1 Preserve offset between restarts
 *
 * @reboot_required true
 * @group Sensors
 * @boolean
 */
PARAM_DEFINE_INT32(SENS_NAU_TARE, 0);


/**
 * NAU7802 Force Sensor Gain
 *
 * @decimal 5
 * @min 0.00000000001
 * @max 10000000000.0
 * @group Sensors
 */
PARAM_DEFINE_FLOAT(SENS_NAU_GAIN, 1.0f);

/**
 * NAU7802 Force Sensor Offset
 *
 * @decimal 5
 * @min -10000000000
 * @max 10000000000
 * @group Sensors
 */
PARAM_DEFINE_FLOAT(SENS_NAU_ZERO, 0.0f);

/**
 * NAU7802 Force Sensor Low Pass Cut Off Frequency
 *
 * @decimal 5
 * @min 1
 * @max 200
 * @group Sensors
 * @unit Hz
 */
PARAM_DEFINE_FLOAT(SENS_NAU_CTFREQ, 10.0f);


/**
 * NAU7802 Force Sensor Sample Rate
 *
 * @decimal 5
 * @min 1
 * @max 500
 * @group Sensors
 * @unit Hz
 */
PARAM_DEFINE_FLOAT(SENS_NAU_RATE, 200.0f);
