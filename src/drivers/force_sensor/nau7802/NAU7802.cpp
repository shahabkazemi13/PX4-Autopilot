/****************************************************************************
 *
 *
 * Copyright (c) 2023 PX4 Development Team. All rights reserved.
 *
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
 * @file NAU7802.cpp
 * @brief Driver for the NAU7802 load cell amplifier with I2C communication.
 *
 * This driver configures, initializes, and manages NAU7802 settings and readings,
 * periodically publishing force measurements via PX4 uORB messaging.
 *
 * @author Agustin Soto and Oliver Vannoort
 */

#include "NAU7802.hpp"
#include <parameters/param.h>

using namespace time_literals;

// PX4 Specific ****************************************************

NAU7802::NAU7802(I2CSPIBusOption bus_option, const int bus, int bus_frequency, int address) :
	I2C(0, "NAU7802", bus, address, bus_frequency),
	I2CSPIDriver(MODULE_NAME, px4::device_bus_to_wq(get_device_id()), bus_option, bus, address),
	ModuleParams(nullptr)
{
	updateParams(); // Initialize parameters
    	_retries = 10;  // Set retry count for I2C transfers
}

/**
 * @brief Update parameters from PX4 parameters.
 */
void NAU7802::updateParams() {
	ModuleParams::updateParams();
	gainAdj = _param_gain.get();
	zeroOffset = _param_offset.get();
	measurement_rate_hz = _param_meas_rate.get();
	_lpf.set_cutoff_frequency(measurement_rate_hz, _param_cutoff_freq.get());

}

/**
 * @brief Probe function to check sensor connectivity.
 *
 * @return PX4_OK if probe is successful, otherwise an error code.
 */
int NAU7802::probe() {
	uint8_t code;
	int status = getRevisionCode(&code);
	if (status != PX4_OK) return status;

	return PX4_OK;
}

/**
 * @brief Initialize the sensor and configure it.
 *
 * @return PX4_OK if initialization succeeds, otherwise an error code.
 */
int NAU7802::init() {
	int status = I2C::init();
	if (status != PX4_OK) return status;
	status = begin();
	if (status != PX4_OK) return status;

	if (tare_on_restart) {

	}

	ScheduleNow();
	return PX4_OK;
}

/**
 * @brief Print status of the I2C connection.
 */
void NAU7802::print_status() {
	I2CSPIDriverBase::print_status();
}

/**
 * @brief Periodic function to schedule sensor readings and parameter updates.
 */
void NAU7802::RunImpl() {

	// Take and publish sensor reading
	PublishMessage();

	// Check if parameters have changed
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams(); // update module parameters (in DEFINE_PARAMETERS)
	}

	// schedule a fresh cycle call when the measurement is done
	ScheduleDelayed((1000000.0f / measurement_rate_hz));
}

/**
 * @brief Publish a force sensor message to uORB.
 */
void NAU7802::PublishMessage() {
	// Gets the time and takes a reading
	const hrt_abstime timestamp = hrt_absolute_time();
	int32_t reading = 0;
	int error = getReading(&reading);

	// Applies the gain, offset and low-pass filter
	float force_n = reading/128.0f*gainAdj+zeroOffset;
	float force_n_filtered = _lpf.apply(force_n);

	// Stores the values in a force message and publishes it
	force_sensor_s force_msg{};
	force_msg.timestamp = timestamp;
	force_msg.error_status = error;
	force_msg.force_measurement_n = force_n;
	force_msg.force_filtered_n = force_n_filtered;
	_force_sensor_pub.publish(force_msg);

	// Publishes the force measurement to the mavlink debug vector
	mav_debug_msg.value = force_n;
	orb_publish(ORB_ID(debug_key_value), pub_mav, &mav_debug_msg);

}


// Sensor Specific ****************************************************

/**
 * @brief Initialise the sensor with appropriate configurations.
 *
 * @return PX4_OK if initialisation succeeds, otherwise an error code.
 */
int NAU7802::begin() {
	int status = PX4_OK;
	status = reset(); //Reset all register;
		if (status != PX4_OK) return status;
	status = powerUp(); //Power on analog and digital sections of the scal;
		if (status != PX4_OK) return status;
	status = setLDO(NAU7802_LDO_3V3); //Set LDO to 3.3;
		if (status != PX4_OK) return status;
	status = setGain(NAU7802_GAIN_64); //Set gain to 32;
		if (status != PX4_OK) return status;
	status = setSampleRate(NAU7802_SPS_80); //Set samples per second to 8;
		if (status != PX4_OK) return status;
	uint8_t adc;
	status = getRegister(NAU7802_ADC,&adc); //Turn off CLK_CHP. From 9.1 power on sequencing.
		if (status != PX4_OK) return status;
	adc |= 0x30;
	status = setRegister(NAU7802_ADC, adc);
		if (status != PX4_OK) return status;
	status = setBit(NAU7802_PGA_PWR_PGA_CAP_EN, NAU7802_PGA_PWR); //Enable 330pF decoupling cap on chan 2. From 9.14 application circuit note;
		if (status != PX4_OK) return status;
	status = clearBit(NAU7802_PGA_LDOMODE, NAU7802_PGA); //Ensure LDOMODE bit is clear - improved accuracy and higher DC gain, with ESR < 1 oh;
		if (status != PX4_OK) return status;

	usleep(_ldoRampDelay * 1000); // Wait for LDO to stabilize - takes about 200ms

	status = calibrateAFE(); //Re-cal analog front end when we change gain, sample rate, or channe;
		if (status != PX4_OK) return status;

	return status;
}

/**
 * @brief Reset sensor by toggling reset bits.
 *
 * @return PX4_OK if reset succeeds, otherwise an error code.
 */
int NAU7802::reset() {
	int status = setBit(NAU7802_PU_CTRL_RR, NAU7802_PU_CTRL);  //Set Reset bit
		if (status != PX4_OK) return status;

	px4_usleep(1000);

	status = clearBit(NAU7802_PU_CTRL_RR, NAU7802_PU_CTRL);  //Clear Reset bit to leave reset state
		if (status != PX4_OK) return status;

  	return PX4_OK;
}


int NAU7802::tare() {

	float sum_readings = 0;
	int status;

	// Gets the time and takes a reading
	for (size_t i = 0; i < NUM_TARE_READINGS; i++) {
		int32_t reading = 0;
		status = getReading(&reading);

		// Applies the gain, offset and low-pass filter
		sum_readings += reading/128.0f*gainAdj;
		if (status != PX4_OK) return status;
	}

	zeroOffset = -(sum_readings / NUM_TARE_READINGS);

	return PX4_OK;
}

/**
 * @brief Power up sensor by setting control bits.
 *
 * @return PX4_OK if power-up succeeds, otherwise an error code.
 */
int NAU7802::powerUp() {
	int status = setBit(NAU7802_PU_CTRL_PUD, NAU7802_PU_CTRL); // Enable digital power
	if (status != PX4_OK) return status;
	status = setBit(NAU7802_PU_CTRL_PUA, NAU7802_PU_CTRL); // Enable analog power
	if (status != PX4_OK) return status;

	//Wait for Power Up bit to be set - takes approximately 200us
	int counter = 0;
	bool bit = false;
	while (bit == false) {
		status = getBit(NAU7802_PU_CTRL_PUR, NAU7802_PU_CTRL, &bit); // Check power-up readiness
		if (status != PX4_OK) return status;
		px4_usleep(1000);
		if (counter++ > 100)
			return PX4_ERROR; //Error
	}
	status = setBit(NAU7802_PU_CTRL_CS, NAU7802_PU_CTRL);  // Set Cycle Start bit. See 9.1 point ;
	if (status != PX4_OK) return status;

	return PX4_OK;
}

/**
 * @brief Retrieves device revision code.
 *
 * @param code - Output pointer to store the device code.
 * @return PX4_OK if retrieval succeeds, otherwise an error code.
 */
int NAU7802::getRevisionCode(uint8_t *code) {
	int status = getRegister(NAU7802_DEVICE_REV, code);
	if (status != PX4_OK) return status;

	// *code = (*code & 0x0F);

  	return PX4_OK;
}

/**
 * @brief Configure gain value for amplifier.
 *
 * @param gainValue - Gain setting for the sensor.
 * @return PX4_OK if gain setting succeeds, otherwise an error code.
 */
int NAU7802::setGain(uint8_t gainValue) {
	if (gainValue > 0b111)
    		gainValue = 0b111; //Error check

	uint8_t reg;
	int status = getRegister(NAU7802_CTRL1, &reg);
	if (status != PX4_OK) return status;

	reg &= 0b11111000; //Clear gain bits
	reg |= gainValue;  //Mask in new bits

	status = setRegister(NAU7802_CTRL1, reg);
	if (status != PX4_OK) return status;

	return PX4_OK;
}

/**
 * @brief Set Low Dropout (LDO) voltage regulator value.
 *
 * @param ldoValue - Desired LDO voltage setting (0b000 to 0b111).
 * @return PX4_OK if LDO setting succeeds, otherwise an error code.
 */
int NAU7802::setLDO(uint8_t ldoValue) {
	if (ldoValue > 0b111)
    		ldoValue = 0b111;  // Error check to limit max value

	uint8_t reg;
	int status = getRegister(NAU7802_CTRL1, &reg); // Retrieve current register setting
	if (status != PX4_OK) return status;

	reg &= 0b11000111;    // Clear LDO bits
	reg |= ldoValue << 3; // Mask in new LDO bits


	status = setRegister(NAU7802_CTRL1, reg); // Apply updated LDO configuration
	if (status != PX4_OK) return status;
	status = setBit(NAU7802_PU_CTRL_AVDDS, NAU7802_PU_CTRL); // Enable LDO control
	if (status != PX4_OK) return status;

	return PX4_OK;
}

/**
 * @brief Set sample rate for ADC conversions.
 *
 * @param rate - Desired sample rate (0b000 to 0b111).
 * @return PX4_OK if sample rate setting succeeds, otherwise an error code.
 */
int NAU7802::setSampleRate(uint8_t rate) {
	 if (rate > 0b111)
   		 rate = 0b111; //Error check

	uint8_t reg;
	int status = getRegister(NAU7802_CTRL2, &reg); // Retrieve current register setting
	if (status != PX4_OK) return status;

	reg &= 0b10001111; //Clear CRS bits
	reg |= rate << 4;  //Mask in new CRS bits

	status = setRegister(NAU7802_CTRL2, reg); // Apply new sample rate bits
	if (status != PX4_OK) return status;

	return PX4_OK;
}


/**
 * @brief Retrieve 24-bit reading from sensor's ADC.
 *
 * @param reading - Output pointer to store ADC reading.
 * @return PX4_OK if read operation succeeds, otherwise an error code.
 */
int NAU7802::getReading(int32_t *reading) {
	int32_t val = 0;
	int status = get24BitRegister(NAU7802_ADCO_B2, &val); // Retrieve ADC value
	if (status != PX4_OK) return status;

	*reading = val; // Store retrieved value
	return PX4_OK;

}

/**
 * @brief Calibrate the sensor's analog front end.
 *
 * @param mode - Calibration mode.
 * @return PX4_OK if calibration succeeds, otherwise an error code.
 */
int NAU7802::calibrateAFE(NAU7802_Cal_Mode mode) {
	int status = beginCalibrateAFE(NAU7802_CALMOD_INTERNAL);
	if (status != PX4_OK) return status;
	status = waitForCalibrateAFE(1000); // Wait for calibration to complete
	if (status != PX4_OK) return status;
  	return PX4_OK;
}

/**
 * @brief Begin calibration process on sensor's analog front end.
 *
 * @param mode - Calibration mode (e.g., internal or external).
 * @return PX4_OK if calibration initiation succeeds, otherwise an error code.
 */
int NAU7802::beginCalibrateAFE(NAU7802_Cal_Mode mode) {
	uint8_t reg;
	int status = getRegister(NAU7802_CTRL2, &reg); // Retrieve current control register setting
	if (status != PX4_OK) return status;

	reg &= 0xFC; // Clear CALMOD bits
	uint8_t calMode = (uint8_t)mode;
  	calMode &= 0x03; // Limit mode to 2 bits
  	reg |= calMode; // Set the mode

	status = setRegister(NAU7802_CTRL2, reg); // Apply calibration mode
	if (status != PX4_OK) return status;

	status = setBit(NAU7802_CTRL2_CALS, NAU7802_CTRL2); // Start calibration sequence
	if (status != PX4_OK) return status;

  	return PX4_OK;
}


/**
 * @brief Wait for calibration completion within a specified timeout.
 *
 * @param timeout_ms - Maximum wait time in milliseconds.
 * @return PX4_OK if calibration completes successfully, otherwise an error code.
 */
int NAU7802::waitForCalibrateAFE(unsigned long timeout_ms) {

	uint64_t startTime = hrt_absolute_time()/1000;
	NAU7802_Cal_Status cal_ready = calAFEStatus();

	while (cal_ready == NAU7802_CAL_IN_PROGRESS) { // Wait while calibration is in progress
		if ((timeout_ms > 0) && (((hrt_absolute_time()/1000) - startTime) > timeout_ms)) {
			break; // Exit if timeout is reached
		}
		px4_usleep(1000);
		cal_ready = calAFEStatus();
	}

	// Check if calibration completed successfully
	if (cal_ready == NAU7802_CAL_PX4_ERROR) {
		return PX4_ERROR;
	}
	if (cal_ready == NAU7802_CAL_FAILURE) {
		return PX4_ERROR;
	}
	return PX4_OK;
}

/**
 * @brief Retrieve calibration status.
 *
 * @return Calibration status (in-progress, success, or error).
 */
NAU7802_Cal_Status NAU7802::calAFEStatus() {
	bool bit;

	int status = getBit(NAU7802_CTRL2_CALS, NAU7802_CTRL2, &bit); // Check calibration status bit
	if (status != PX4_OK) return NAU7802_CAL_PX4_ERROR;

	if (bit) return NAU7802_CAL_IN_PROGRESS;

	status = getBit(NAU7802_CTRL2_CAL_ERROR, NAU7802_CTRL2, &bit); // Check for calibration error
	if (status != PX4_OK) return NAU7802_CAL_PX4_ERROR;

	if (bit) return NAU7802_CAL_FAILURE;

	// Calibration passed
	return NAU7802_CAL_SUCCESS;
}


/**
 * @brief Retrieve an 8-bit register value.
 *
 * @param registerAddress - Address of register to read.
 * @param data - Output pointer for register data.
 * @return PX4_OK if read operation succeeds, otherwise an error code.
 */
int NAU7802::getRegister(uint8_t registerAddress, uint8_t *data) {
	// // PX4 Transfer Function
	// // I2C::transfer(const uint8_t *send, const unsigned send_len, uint8_t *recv, const unsigned recv_len)

	// // Send the register address and return 1 byte response
	// uint8_t val;
	// uint8_t cmd = registerAddress;
	// transfer(&cmd, 1, &val, sizeof(val));

	// // TODO: Handle errors here
	// return val;

	uint8_t cmd = registerAddress;
	int status = transfer(&cmd, sizeof(cmd), data, sizeof(*data));
	if (status != PX4_OK) return status;

	return PX4_OK;
}

/**
 * @brief Set an 8-bit register value.
 *
 * @param registerAddress - Address of register to write.
 * @param value - Value to write.
 * @return PX4_OK if write operation succeeds, otherwise an error code.
 */
int NAU7802::setRegister(uint8_t registerAddress, uint8_t value) {
	// Send the two byte command and return success
	uint8_t cmd[2];
	cmd[0] = static_cast<uint8_t>(registerAddress);
	cmd[1] = static_cast<uint8_t>(value);

	int status = transfer(&cmd[0], 2, nullptr, 0);
	if (status != PX4_OK) return status;

	return PX4_OK;
}

/**
 * @brief Retrieve a 24-bit register value.
 *
 * @param registerAddress - Address of register to read.
 * @param data - Output pointer for register data.
 * @return PX4_OK if read operation succeeds, otherwise an error code.
 */
int NAU7802::get24BitRegister(uint8_t registerAddress, int32_t *data) {

	// Send the 1 byte command and record the 3 byte response
	uint8_t val[3] = {0,0,0};
	uint8_t cmd = registerAddress;

	int status = transfer(&cmd, 1, &val[0], sizeof(val));
	if (status != PX4_OK) return status;

	// Construct a union for conversion between unsigned and signed
	union {
		uint32_t usign;
		int32_t  sign;
	} union32_t;

	// Fill the union, convert to signed integer and return
	union32_t.usign = (uint32_t)val[0] << 16; //MSB
	union32_t.usign |= (uint32_t)val[1] << 8; //MidSB
	union32_t.usign |= (uint32_t)val[2];      //LSB
	if ((union32_t.usign & 0x00800000) == 0x00800000)
		union32_t.usign |= 0xFF000000; // Preserve 2's complement

	*data = (union32_t.sign);
	return PX4_OK;

}

/**
 * @brief Set a 24-bit register value.
 *
 * @param registerAddress - Address of register to write.
 * @param value - Value to write.
 * @return PX4_OK if write operation succeeds, otherwise an error code.
 */
int NAU7802::set24BitRegister(uint8_t registerAddress, int32_t value) {

	// Construct a union for conversion between unsigned and signed and fill with value
	union {
		uint32_t usign;
		int32_t  sign;
	} union32_t;
	union32_t.sign = value;

	// Fill the array of commands
	uint8_t cmd[4];
	cmd[0] = static_cast<uint8_t>(registerAddress);
	cmd[1] = static_cast<uint8_t>((union32_t.usign >> 16) & 0xFF);
	cmd[2] = static_cast<uint8_t>((union32_t.usign >> 8) & 0xFF);
	cmd[3] = static_cast<uint8_t>(union32_t.usign & 0xFF);

	int status = transfer(&cmd[0], 4, nullptr, 0);
	if (status != PX4_OK) return status;

	return PX4_OK;
}

/**
 * @brief Retrieve a 32-bit register value.
 *
 * @param registerAddress - Address of register to read.
 * @param data - Output pointer for register data.
 * @return PX4_OK if read operation succeeds, otherwise an error code.
 */
int NAU7802::get32BitRegister(uint8_t registerAddress, uint32_t *data) {
	// Send the register address and return 4 byte response
	uint8_t val[4] = {0,0,0,0};
	uint8_t cmd = registerAddress;

	int status = transfer(&cmd, 1, &(val[0]), sizeof(val));
	if (status != PX4_OK) return status;

	// Convert the 4 bytes to a single 32 bit unsigned integer
	uint32_t val32;
	val32  = (uint32_t)val[0] << 24; //MSB
	val32 |= (uint32_t)val[1] << 16;
	val32 |= (uint32_t)val[2] << 8;
	val32 |= (uint32_t)val[3];       //LSB

	// TODO: Handle errors here
	*data = val32;
	return PX4_OK;

}

/**
 * @brief Set a 32-bit register value.
 *
 * @param registerAddress - Address of register to write.
 * @param value - Value to write.
 * @return PX4_OK if write operation succeeds, otherwise an error code.
 */
int NAU7802::set32BitRegister(uint8_t registerAddress, uint32_t value) {
	// Fill the command array with the value and send
	uint8_t cmd[5];
	cmd[0] = static_cast<uint8_t>(registerAddress);
	cmd[1] = static_cast<uint8_t>((value >> 24) & 0xFF);
	cmd[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
	cmd[3] = static_cast<uint8_t>((value >> 8 ) & 0xFF);
	cmd[4] = static_cast<uint8_t>( value        & 0xFF);

	int status = transfer(&cmd[0], 5, nullptr, 0);
	if (status != PX4_OK) return status;

	return PX4_OK;

}

/**
 * @brief Set a specific bit in a register.
 *
 * @param bitNumber - Bit position to set.
 * @param registerAddress - Address of register to modify.
 * @return PX4_OK if operation succeeds, otherwise an error code.
 */
int NAU7802::setBit(uint8_t bitNumber, uint8_t registerAddress) {
	uint8_t reg = 0;
	int status = getRegister(registerAddress, &reg);
	if (status != PX4_OK) return status;

	reg |= (1 << bitNumber); //Set this bit
	status = setRegister(registerAddress, reg);
	if (status != PX4_OK) return status;

	return PX4_OK;
}

/**
 * @brief Clear a specific bit in a register.
 *
 * @param bitNumber - Bit position to clear.
 * @param registerAddress - Address of register to modify.
 * @return PX4_OK if operation succeeds, otherwise an error code.
 */
int NAU7802::clearBit(uint8_t bitNumber, uint8_t registerAddress) {
	uint8_t reg = 0;
	int status = getRegister(registerAddress, &reg);
	if (status != PX4_OK) return status;

	reg &= ~(1 << bitNumber); //Set this bit
	status =  setRegister(registerAddress, reg);
	if (status != PX4_OK) return status;

	return PX4_OK;
}

/**
 * @brief Retrieve a specific bit from a register.
 *
 * @param bitNumber - Bit position to read.
 * @param registerAddress - Address of register to read.
 * @param data - Output pointer for bit value.
 * @return PX4_OK if read operation succeeds, otherwise an error code.
 */
int NAU7802::getBit(uint8_t bitNumber, uint8_t registerAddress, bool *data) {
	uint8_t reg = 0;
	int status = getRegister(registerAddress,&reg);
	if (status != PX4_OK) return status;

	reg &= (1 << bitNumber);
	*data = (bool)reg;

	return PX4_OK;
}
