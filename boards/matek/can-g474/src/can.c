/****************************************************************************
 *
 *   Copyright (C) 2024 PX4 Development Team. All rights reserved.
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
 * @file can.c
 *
 * Board-specific CAN functions for SocketCAN / FDCAN.
 */

#include <px4_platform_common/px4_config.h>

#include <errno.h>
#include <debug.h>

#include <nuttx/can/can.h>
#include <arch/board/board.h>

#include "chip.h"
#include "arm_internal.h"

#include "stm32.h"

#ifdef CONFIG_STM32_FDCAN
#include "stm32_fdcan.h"
#endif

#include "board_config.h"

/****************************************************************************
 * Name: can_devinit
 *
 * Description:
 *   Initialize SocketCAN interfaces for FDCAN1 and FDCAN2
 *
 ****************************************************************************/

#if defined(CONFIG_NET_CAN) && (defined(CONFIG_STM32_FDCAN1) || defined(CONFIG_STM32_FDCAN2))

int can_devinit(void);

int can_devinit(void)
{
	static bool initialized = false;
	int ret = OK;

	/* Check if we have already initialized */
	if (!initialized) {
#ifdef CONFIG_STM32_FDCAN1
		/* Initialize FDCAN1 as SocketCAN interface can0 */
		ret = stm32_fdcansockinitialize(1);

		if (ret < 0) {
			canerr("ERROR: Failed to initialize FDCAN1 SocketCAN: %d\n", ret);
			return ret;
		}

		caninfo("FDCAN1 SocketCAN initialized as can0\n");
#endif

#ifdef CONFIG_STM32_FDCAN2
		/* Initialize FDCAN2 as SocketCAN interface can1 */
		ret = stm32_fdcansockinitialize(2);

		if (ret < 0) {
			canerr("ERROR: Failed to initialize FDCAN2 SocketCAN: %d\n", ret);
			return ret;
		}

		caninfo("FDCAN2 SocketCAN initialized as can1\n");
#endif

		initialized = true;
	}

	return ret;
}

#endif /* CONFIG_NET_CAN && (CONFIG_STM32_FDCAN1 || CONFIG_STM32_FDCAN2) */
