/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * @file init.c
 *
 * Matek CAN-G474 board-specific initialization
 */

#include <px4_platform_common/px4_config.h>

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/mmcsd.h>
#include <nuttx/analog/adc.h>

#include <stm32.h>
#include <stm32_gpio.h>
#include "board_config.h"

#include <arch/board/board.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_board_led.h>

#include <systemlib/px4_macros.h>

#include <px4_arch/io_timer.h>
#include <px4_platform_common/init.h>
#include <px4_platform/gpio.h>

__EXPORT void stm32_boardinitialize(void);

#if defined(CONFIG_NET_CAN)
extern int can_devinit(void);
#endif

/****************************************************************************
 * Name: stm32_boardinitialize
 *
 * Description:
 *   All STM32 architectures must provide the following entry point.  This entry
 *   point is called early in the initialization -- after all memory has been
 *   configured and mapped but before any devices have been initialized.
 *
 ****************************************************************************/

__EXPORT void
stm32_boardinitialize(void)
{
	/* Configure GPIO pins */
	const uint32_t gpio[] = PX4_GPIO_INIT_LIST;

	for (unsigned i = 0; i < arraySize(gpio); i++) {
		stm32_configgpio(gpio[i]);
	}

	/* Configure LEDs */
	board_autoled_initialize();

	/* Blink LED rapidly on boot to show we're alive - 20 rapid blinks */
	for (int i = 0; i < 20; i++) {
		stm32_gpiowrite(GPIO_LED_BLUE, true);  // LED ON (active HIGH)
		up_mdelay(50);
		stm32_gpiowrite(GPIO_LED_BLUE, false);   // LED OFF
		up_mdelay(50);
	}

	/* Leave LED ON to show boot completed */
	stm32_gpiowrite(GPIO_LED_BLUE, true);
}

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform application specific initialization.  This function is never
 *   called directly from application code, but only indirectly via the
 *   (non-standard) boardctl() interface using the command BOARDIOC_INIT.
 *
 ****************************************************************************/

__EXPORT int board_app_initialize(uintptr_t arg)
{
	/* Configure the HRT */
	hrt_init();

	/* Initialize work queues */
	px4_platform_init();

	/* Initialize CAN */
#if defined(CONFIG_NET_CAN)
	int ret = can_devinit();

	if (ret < 0) {
		syslog(LOG_ERR, "ERROR: CAN initialization failed: %d\n", ret);
	}

#endif

	return OK;
}
