/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
 *
 ****************************************************************************/

/**
 * @file heartbeat.cpp
 *
 * Simple LED heartbeat for boards without serial console access
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/tasks.h>
#include <drivers/drv_hrt.h>
#include "board_config.h"
#include <nuttx/arch.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

static bool _should_exit = false;
static px4_task_t _task_handle = -1;

static int heartbeat_task(int argc, char *argv[])
{
	PX4_INFO("Heartbeat task started");

	while (!_should_exit) {
		// Check if CAN devices exist
		struct stat st;
		bool can0_exists = (stat("/dev/can0", &st) == 0);
		bool can1_exists = (stat("/dev/can1", &st) == 0);

		// Blink pattern indicates status:
		// 1 short blink = no CAN devices
		// 2 short blinks = CAN0 only
		// 3 short blinks = CAN1 only  
		// 4 short blinks = both CAN devices exist
		int pattern = 1;

		if (can0_exists && can1_exists) {
			pattern = 4;

		} else if (can0_exists) {
			pattern = 2;

		} else if (can1_exists) {
			pattern = 3;
		}

		// Blink the pattern
		for (int i = 0; i < pattern; i++) {
			stm32_gpiowrite(GPIO_LED_BLUE, true);
			px4_usleep(100000);  // 100ms on
			stm32_gpiowrite(GPIO_LED_BLUE, false);
			px4_usleep(100000);  // 100ms off
		}

		// Long pause between patterns
		px4_usleep(800000);  // 800ms off
	}

	PX4_INFO("Heartbeat task exiting");
	return 0;
}

extern "C" __EXPORT int heartbeat_main(int argc, char *argv[])
{
	if (argc < 2) {
		PX4_INFO("usage: heartbeat {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {
		if (_task_handle != -1) {
			PX4_WARN("already running");
			return 0;
		}

		_should_exit = false;

		_task_handle = px4_task_spawn_cmd("heartbeat",
						  SCHED_DEFAULT,
						  SCHED_PRIORITY_DEFAULT - 10,
						  1024,
						  heartbeat_task,
						  nullptr);

		if (_task_handle < 0) {
			PX4_ERR("task start failed");
			return -1;
		}

		PX4_INFO("heartbeat started");
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (_task_handle == -1) {
			PX4_WARN("not running");
			return 0;
		}

		_should_exit = true;

		// Wait for task to exit
		for (int i = 0; i < 10 && _task_handle != -1; i++) {
			px4_usleep(100000);
		}

		_task_handle = -1;
		PX4_INFO("heartbeat stopped");
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (_task_handle == -1) {
			PX4_INFO("not running");
			return 1;
		}

		PX4_INFO("running");
		return 0;
	}

	PX4_ERR("unrecognized command");
	return 1;
}
