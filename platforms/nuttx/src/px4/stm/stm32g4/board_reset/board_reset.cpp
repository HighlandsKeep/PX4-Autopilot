/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * @file board_reset.cpp
 * Implementation of STM32G4 based Board RESET API using RTC backup registers
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/shutdown.h>
#include <systemlib/px4_macros.h>
#include <errno.h>

#include <hardware/stm32g4xxxx_pwr.h>
#include <hardware/stm32_rtc.h>

#include <nuttx/board.h>

#include "board_config.h"
#include <px4_arch/micro_hal.h>

#ifdef CONFIG_BOARDCTL_RESET

static const uint32_t modes[] = {
/*                                      to  tb   */
/* BOARD_RESET_MODE_CLEAR                5   y   */  0,
/* BOARD_RESET_MODE_BOOT_TO_BL           0   n   */  0xb007b007,
/* BOARD_RESET_MODE_BOOT_TO_VALID_APP    0   y   */  0xb0070002,
/* BOARD_RESET_MODE_CAN_BL               10  n   */  0xb0080000,
/* BOARD_RESET_MODE_RTC_BOOT_FWOK        0   n   */  0xb0093a26
};

int board_configure_reset(reset_mode_e mode, uint32_t arg)
{
int rv = -1;

if (mode < arraySize(modes)) {

/* STM32G4 uses RTC backup registers - need to enable RTC access */
modifyreg32(STM32_PWR_CR1, 0, PWR_CR1_DBP);

/* Wait for RTC register access to be enabled */
while ((getreg32(STM32_PWR_CR1) & PWR_CR1_DBP) == 0);

/* Write mode value to RTC backup register */
putreg32(modes[mode], PX4_BBSRAM_ADDR);

/* Disable RTC access */
modifyreg32(STM32_PWR_CR1, PWR_CR1_DBP, 0);

rv = OK;
}

return rv;
}

reset_mode_e board_reset_reason(void)
{
reset_mode_e reason = BOARD_RESET_MODE_CLEAR;

/* STM32G4 uses RTC backup registers - need to enable RTC access */
modifyreg32(STM32_PWR_CR1, 0, PWR_CR1_DBP);

/* Wait for RTC register access to be enabled */
while ((getreg32(STM32_PWR_CR1) & PWR_CR1_DBP) == 0);

/* Read mode value from RTC backup register */
uint32_t regvalue = getreg32(PX4_BBSRAM_ADDR);

/* Clear the register */
putreg32(0, PX4_BBSRAM_ADDR);

/* Disable RTC access */
modifyreg32(STM32_PWR_CR1, PWR_CR1_DBP, 0);

/* Find matching mode */
for (unsigned i = 0; i < arraySize(modes); i++) {
if (regvalue == modes[i]) {
reason = (reset_mode_e)i;
break;
}
}

return reason;
}

/****************************************************************************
 * Name: board_reset
 *
 * Description:
 *   Reset the board to the bootloader or just reboot
 *
 * Input Parameters:
 *   status  - The reason for the reset
 *
 * Returned Value:
 *   0 - does not return
 *
 ****************************************************************************/

int board_reset(int status)
{
	if (status == REBOOT_TO_BOOTLOADER) {
		board_configure_reset(BOARD_RESET_MODE_BOOT_TO_BL, 0);
	}

	up_systemreset();
	return 0;
}

#endif /* CONFIG_BOARDCTL_RESET */

