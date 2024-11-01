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


#include "NAU7802.hpp"

/**
 * @brief Main entry point for the NAU7802 module.
 *
 * This function parses command-line arguments and executes the appropriate
 * command (start, stop, or status) for the NAU7802 driver. It also sets up
 * default I2C parameters for communication.
 *
 * @param argc Number of arguments.
 * @param argv Array of argument strings.
 * @return int Status of the operation. Returns -1 if the command is unrecognized.
 */
extern "C" int nau7802_main(int argc, char *argv[])
{
	using ThisDriver = NAU7802;
	BusCLIArguments cli{true, false};
	cli.i2c_address = I2C_ADDRESS_DEFAULT;
	cli.default_i2c_frequency = I2C_SPEED;

	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		ThisDriver::print_usage();
		return -1;
	}

	BusInstanceIterator iterator(MODULE_NAME, cli, DRV_FORCE_SENS_DEVTYPE_NAU7802);


	if (!strcmp(verb, "start")) {
		return ThisDriver::module_start(cli, iterator);
	} else if (!strcmp(verb, "stop")) {
		return ThisDriver::module_stop(iterator);
	} else if (!strcmp(verb, "status")) {
		return ThisDriver::module_status(iterator);
	}

	ThisDriver::print_usage();
	return -1;
}

/**
 * @brief Print usage instructions for the NAU7802 driver.
 *
 * This function outputs a description of the driver and usage information
 * for the commands that can be executed with the NAU7802 driver.
 */
void NAU7802::print_usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Driver to enable an external [NAU7802]
TE connected via I2C.
This is not included by default in firmware. It can be included with terminal command: "make <your_board> boardconfig"
or in default.px4board with adding the line: "CONFIG_DRIVERS_DIFFERENTIAL_PRESSURE_NAU7802=y"
It can be enabled with the "SENS_EN_NAU7802" parameter set to 1.
)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("nau7802", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("force_sensor");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(I2C_ADDRESS_DEFAULT);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

/**
 * @brief Instantiate an NAU7802 instance.
 *
 * This function allocates and initializes an instance of the NAU7802 driver
 * with specified bus and communication parameters.
 *
 * @param cli Command-line interface arguments specifying configuration.
 * @param iterator Iterator over bus instances for this driver.
 * @param runtime_instance Runtime instance number for multiple instances.
 * @return I2CSPIDriverBase* Pointer to the instantiated driver instance or nullptr if allocation fails.
 */
I2CSPIDriverBase *NAU7802::instantiate(const BusCLIArguments &cli, const BusInstanceIterator &iterator,
				     int runtime_instance)
{
	NAU7802 *instance = new NAU7802(iterator.configuredBusOption(), iterator.bus(), cli.bus_frequency, cli.i2c_address);

	if (instance == nullptr) {
		PX4_ERR("alloc failed");
		return nullptr;
	}

	if (instance->init() != PX4_OK) {
		delete instance;
		return nullptr;
	}

	return instance;
}
