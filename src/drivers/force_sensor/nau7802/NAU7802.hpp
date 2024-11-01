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
 * @file NAU7802.hpp
 *
 * Driver for NAU7802 connected via I2C.
 *
 * Supported sensors:
 *
 *    - NAU7802
 *
 * Interface application notes:
 *
 *
 * @author Oliver Vannoort and Agustin Soto <opvannoort@gmail.com>
 */

#pragma once

#include <drivers/drv_hrt.h>
#include <drivers/device/i2c.h>
// #include <lib/perf/perf_counter.h>

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/param_macros.h>
#include <px4_platform_common/i2c_spi_buses.h>

#include <lib/systemlib/mavlink_log.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/force_sensor.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/debug_key_value.h>

#include <mathlib/math/filter/LowPassFilter2p.hpp>


//Register Map
typedef enum
{
  NAU7802_PU_CTRL = 0x00,
  NAU7802_CTRL1,
  NAU7802_CTRL2,
  NAU7802_OCAL1_B2,
  NAU7802_OCAL1_B1,
  NAU7802_OCAL1_B0,
  NAU7802_GCAL1_B3,
  NAU7802_GCAL1_B2,
  NAU7802_GCAL1_B1,
  NAU7802_GCAL1_B0,
  NAU7802_OCAL2_B2,
  NAU7802_OCAL2_B1,
  NAU7802_OCAL2_B0,
  NAU7802_GCAL2_B3,
  NAU7802_GCAL2_B2,
  NAU7802_GCAL2_B1,
  NAU7802_GCAL2_B0,
  NAU7802_I2C_CONTROL,
  NAU7802_ADCO_B2,
  NAU7802_ADCO_B1,
  NAU7802_ADCO_B0,
  NAU7802_ADC = 0x15, //Shared ADC and OTP 32:24
  NAU7802_OTP_B1,     //OTP 23:16 or 7:0?
  NAU7802_OTP_B0,     //OTP 15:8
  NAU7802_PGA = 0x1B,
  NAU7802_PGA_PWR = 0x1C,
  NAU7802_DEVICE_REV = 0x1F,
} Scale_Registers;

//Bits within the PU_CTRL register
typedef enum
{
  NAU7802_PU_CTRL_RR = 0,
  NAU7802_PU_CTRL_PUD,
  NAU7802_PU_CTRL_PUA,
  NAU7802_PU_CTRL_PUR,
  NAU7802_PU_CTRL_CS,
  NAU7802_PU_CTRL_CR,
  NAU7802_PU_CTRL_OSCS,
  NAU7802_PU_CTRL_AVDDS,
} PU_CTRL_Bits;

//Bits within the CTRL1 register
typedef enum
{
  NAU7802_CTRL1_GAIN = 2,
  NAU7802_CTRL1_VLDO = 5,
  NAU7802_CTRL1_DRDY_SEL = 6,
  NAU7802_CTRL1_CRP = 7,
} CTRL1_Bits;

//Bits within the CTRL2 register
typedef enum
{
  NAU7802_CTRL2_CALMOD = 0,
  NAU7802_CTRL2_CALS = 2,
  NAU7802_CTRL2_CAL_ERROR = 3,
  NAU7802_CTRL2_CRS = 4,
  NAU7802_CTRL2_CHS = 7,
} CTRL2_Bits;

//Bits within the PGA register
typedef enum
{
  NAU7802_PGA_CHP_DIS = 0,
  NAU7802_PGA_INV = 3,
  NAU7802_PGA_BYPASS_EN,
  NAU7802_PGA_OUT_EN,
  NAU7802_PGA_LDOMODE,
  NAU7802_PGA_RD_OTP_SEL,
} PGA_Bits;

//Bits within the PGA PWR register
typedef enum
{
  NAU7802_PGA_PWR_PGA_CURR = 0,
  NAU7802_PGA_PWR_ADC_CURR = 2,
  NAU7802_PGA_PWR_MSTR_BIAS_CURR = 4,
  NAU7802_PGA_PWR_PGA_CAP_EN = 7,
} PGA_PWR_Bits;

//Allowed Low drop out regulator voltages
typedef enum
{
  NAU7802_LDO_2V4 = 0b111,
  NAU7802_LDO_2V7 = 0b110,
  NAU7802_LDO_3V0 = 0b101,
  NAU7802_LDO_3V3 = 0b100,
  NAU7802_LDO_3V6 = 0b011,
  NAU7802_LDO_3V9 = 0b010,
  NAU7802_LDO_4V2 = 0b001,
  NAU7802_LDO_4V5 = 0b000,
} NAU7802_LDO_Values;

//Allowed gains
typedef enum
{
  NAU7802_GAIN_128 = 0b111,
  NAU7802_GAIN_64 = 0b110,
  NAU7802_GAIN_32 = 0b101,
  NAU7802_GAIN_16 = 0b100,
  NAU7802_GAIN_8 = 0b011,
  NAU7802_GAIN_4 = 0b010,
  NAU7802_GAIN_2 = 0b001,
  NAU7802_GAIN_1 = 0b000,
} NAU7802_Gain_Values;

//Allowed samples per second
typedef enum
{
  NAU7802_SPS_320 = 0b111,
  NAU7802_SPS_80 = 0b011,
  NAU7802_SPS_40 = 0b010,
  NAU7802_SPS_20 = 0b001,
  NAU7802_SPS_10 = 0b000,
} NAU7802_SPS_Values;

//Select between channel values
typedef enum
{
  NAU7802_CHANNEL_1 = 0,
  NAU7802_CHANNEL_2 = 1,
} NAU7802_Channels;

//Calibration state
typedef enum
{
  NAU7802_CAL_PX4_ERROR = -1,
  NAU7802_CAL_SUCCESS = 0,
  NAU7802_CAL_IN_PROGRESS = 1,
  NAU7802_CAL_FAILURE = 2,
} NAU7802_Cal_Status;

//Calibration mode
typedef enum
{
  NAU7802_CALMOD_INTERNAL = 0,
  NAU7802_CALMOD_OFFSET = 2,
  NAU7802_CALMOD_GAIN,
} NAU7802_Cal_Mode;


/* Configuration Constants */
static constexpr uint8_t I2C_ADDRESS_DEFAULT = 0x2A; /* 0x2A */
static constexpr uint32_t I2C_SPEED = 100000; // 100 kHz I2C serial interface
static constexpr uint32_t NUM_TARE_READINGS = 10;

// using namespace time_literals;

class NAU7802 : public device::I2C, public I2CSPIDriver<NAU7802>, public ModuleParams{
public:
	// PX4 Specific **************************************************************************************************************
	NAU7802(I2CSPIBusOption bus_option, const int bus, int bus_frequency, int address);
	virtual ~NAU7802() = default;
  static I2CSPIDriverBase *instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,int runtime_instance);
	static void print_usage();
	void RunImpl();
	void PublishMessage();
  int init();
  void print_status() override;


private:

	// PX4 Specific **************************************************************************************************************
  int probe() override;
  void updateParams() override;
  debug_key_value_s mav_debug_msg{0,0.0,"FORCE_MEA",0};
  orb_advert_t pub_mav = orb_advertise(ORB_ID(debug_key_value), &mav_debug_msg);
	uORB::PublicationMulti<force_sensor_s> _force_sensor_pub{ORB_ID(force_sensor)};
  uORB::SubscriptionInterval  _parameter_update_sub{ORB_ID(parameter_update), 1}; // subscription limited to 1 Hz updates

  float measurement_rate_hz = 200;
  math::LowPassFilter2p<float> _lpf{200, 10.f};
  bool tare_on_restart = true;


  DEFINE_PARAMETERS(
    (ParamInt<px4::params::SENS_NAU_TARE>) _param_tare,
    (ParamFloat<px4::params::SENS_NAU_GAIN>) _param_gain,
    (ParamFloat<px4::params::SENS_NAU_ZERO>) _param_offset,
    (ParamFloat<px4::params::SENS_NAU_RATE>) _param_meas_rate,
    (ParamFloat<px4::params::SENS_NAU_CTFREQ>) _param_cutoff_freq
  )


	// Sensor Specific ***********************************************************************************************************

	// Variables
  float zeroOffset = 0.0;
	float gainAdj = 0.001;
	unsigned long _ldoRampDelay = 250;

	// Functions
  int begin(); //Check communication and initialize sensor
  int reset(); //Resets all registers to Power Of Defaults
  int tare();  //Tares the zero_offset value to zero
  int powerUp();   //Power up digital and analog sections of scale, ~2mA
	int getRevisionCode(uint8_t *code);

	int setGain(uint8_t gainValue);
	int setLDO(uint8_t ldoValue);
	int setSampleRate(uint8_t rate);

	int getReading(int32_t *data);

	int calibrateAFE(NAU7802_Cal_Mode mode = NAU7802_CALMOD_INTERNAL);      //Synchronous calibration of the analog front end of the NAU7802. Returns true if CAL_ERR bit is 0 (no error)
	int beginCalibrateAFE(NAU7802_Cal_Mode mode = NAU7802_CALMOD_INTERNAL); //Begin asynchronous calibration of the analog front end of the NAU7802. Poll for completion with calAFEStatus() or wait with waitForCalibrateAFE().
	int waitForCalibrateAFE(unsigned long timeout_ms = 0); //Wait for asynchronous AFE calibration to complete with optional timeout.
	NAU7802_Cal_Status calAFEStatus();                 //Check calibration status.

	int getRegister(uint8_t registerAddress, uint8_t *data);             //Get contents of a register
	int setRegister(uint8_t registerAddress, uint8_t value); //Send a given value to be written to given address. Return true if successful
	int get24BitRegister(uint8_t registerAddress, int32_t *data);        //Get contents of a 24-bit signed register (conversion result and offsets)
	int set24BitRegister(uint8_t registerAddress, int32_t value); //Send 24 LSBs of value to given register address. Return true if successful
	int get32BitRegister(uint8_t registerAddress, uint32_t *data);       //Get contents of a 32-bit register (gains)
	int set32BitRegister(uint8_t registerAddress, uint32_t value); //Send a given value to be written to given address. Return true if successful
	int setBit(uint8_t bitNumber, uint8_t registerAddress);   //Mask & set a given bit within a register
	int clearBit(uint8_t bitNumber, uint8_t registerAddress); //Mask & clear a given bit within a register
	int getBit(uint8_t bitNumber, uint8_t registerAddress, bool *data);   //Return a given bit within a register

};

