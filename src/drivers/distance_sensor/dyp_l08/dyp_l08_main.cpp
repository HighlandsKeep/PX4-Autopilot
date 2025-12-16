/****************************************************************************
 *
 *   Copyright (c) 2025 Highlands Keep. All rights reserved.
 *
 *   This software is proprietary and confidential.
 *   Unauthorized copying or distribution is strictly prohibited.
 *
 ****************************************************************************/

/**
 * @file dyp_l08_main.cpp
 * @author Andrew Gregg
 *
 * Driver for the DYP-L08 ultrasonic underwater rangefinder.
 */

#include <px4_platform_common/cli.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

#include "DYP_L08.hpp"

/**
 * Local functions in support of the shell command.
 */
namespace dyp_l08
{

DYP_L08	*g_dev;

int reset(const char *port);
int start(const char *port, const uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING);
int status();
int stop();
int usage();

/**
 * Reset the driver.
 */
int
reset(const char *port)
{
	if (stop() == PX4_OK) {
		return start(port);
	}

	return PX4_ERROR;
}

/**
 * Start the driver.
 */
int
start(const char *port, const uint8_t rotation)
{
	if (port == nullptr) {
		PX4_ERR("invalid port");
		return PX4_ERROR;
	}

	if (g_dev != nullptr) {
		PX4_INFO("already started");
		return PX4_OK;
	}

	// Instantiate the driver.
	g_dev = new DYP_L08(port, rotation);

	if (g_dev == nullptr) {
		PX4_ERR("object instantiate failed");
		return PX4_ERROR;
	}

	if (g_dev->init() != PX4_OK) {
		PX4_ERR("driver start failed");
		delete g_dev;
		g_dev = nullptr;
		return PX4_ERROR;
	}

	return PX4_OK;
}

/**
 * Print the driver status.
 */
int
status()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver not running");
		return PX4_ERROR;
	}

	g_dev->print_info();

	return PX4_OK;
}

/**
 * Stop the driver
 */
int stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	return PX4_OK;
}

int
usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Driver for the DYP-L08 ultrasonic underwater rangefinder connected via serial port.

The sensor must be configured to use a specific serial port, which can be set via
the SENS_DYP_L08_CFG parameter.

### Examples

Start driver on GPS 2 port with default settings:
$ dyp_l08 start -d /dev/ttyS5

Start with downward facing orientation:
$ dyp_l08 start -d /dev/ttyS5 -R 25
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("dyp_l08", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("distance_sensor");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start driver");
	PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, "<file:dev>", "Serial device", false);
	PRINT_MODULE_USAGE_PARAM_INT('R', 25, 0, 25, "Sensor rotation - downward facing by default", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print driver status");
	PRINT_MODULE_USAGE_COMMAND_DESCR("reset", "Reset driver");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return PX4_OK;
}

} // namespace dyp_l08


/**
 * Driver 'main' command.
 */
extern "C" __EXPORT int dyp_l08_main(int argc, char *argv[])
{
	uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
	const char *device_path = nullptr;
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "R:d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'R': {
				int rot = -1;

				if (px4_get_parameter_value(myoptarg, rot) != 0) {
					PX4_ERR("rotation parsing failed");
					return -1;
				}

				rotation = (uint8_t)rot;
				break;
			}

		case 'd':
			device_path = myoptarg;
			break;

		default:
			PX4_WARN("Unknown option!");
			return dyp_l08::usage();
		}
	}

	if (myoptind >= argc) {
		return dyp_l08::usage();
	}

	// Reset the driver.
	if (!strcmp(argv[myoptind], "reset")) {
		return dyp_l08::reset(device_path);
	}

	// Start/load the driver.
	if (!strcmp(argv[myoptind], "start")) {
		return dyp_l08::start(device_path, rotation);
	}

	// Print driver information.
	if (!strcmp(argv[myoptind], "status")) {
		return dyp_l08::status();
	}

	// Stop the driver
	if (!strcmp(argv[myoptind], "stop")) {
		return dyp_l08::stop();
	}

	return dyp_l08::usage();
}
