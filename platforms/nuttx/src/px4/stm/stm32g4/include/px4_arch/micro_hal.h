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
#pragma once

/* STM32G4 doesn't have GPIO_SPEED_2MHz, use 25MHz instead */
#ifndef GPIO_SPEED_2MHz
#define GPIO_SPEED_2MHz GPIO_SPEED_25MHz
#endif

#define HRT_TIMER                    7  /* use timer 7 for HRT */
#define HRT_TIMER_CHANNEL            1  /* use capture/compare channel 1 */

/* STM32G4 uses RTC backup registers, not BKP domain like F1 */
#define STM32_BKP_BASE               STM32_RTC_BASE

#include "../../../stm32_common/include/px4_arch/micro_hal.h"

__BEGIN_DECLS

#include <stm32.h>
/* STM32G4 RTC uses the new-style RTCC with backup registers */
#include <hardware/stm32_rtcc.h>

#define PX4_SOC_ARCH_ID             PX4_SOC_ARCH_ID_STM32G4
#define PX4_FLASH_BASE  STM32_FLASH_BASE

/* STM32G4 uses FDCAN instead of bxCAN - no filter registers to use for signature storage */
/* Use backup registers in RTC domain instead */
#undef crc_HiLOC
#undef crc_LoLOC
#undef signature_LOC
#undef bus_speed_LOC
#undef node_id_LOC

/* Use RTC backup registers for boot data storage */
#define crc_HiLOC       (STM32_RTC_BASE + 0x50 + 0)   /* RTC_BKP0R */
#define crc_LoLOC       (STM32_RTC_BASE + 0x50 + 4)   /* RTC_BKP1R */
#define signature_LOC   (STM32_RTC_BASE + 0x50 + 8)   /* RTC_BKP2R */
#define bus_speed_LOC   (STM32_RTC_BASE + 0x50 + 12)  /* RTC_BKP3R */
#define node_id_LOC     (STM32_RTC_BASE + 0x50 + 16)  /* RTC_BKP4R */

#define BOOTLOADER_RESERVATION_SIZE (32 * 1024)

#define PX4_NUMBER_I2C_BUSES STM32_NI2C
#define PX4_ADC_INTERNAL_TEMP_SENSOR_CHANNEL 16

/* STM32G4 RTC Backup Register for bootloader communication */
#define PX4_BBSRAM_ADDR   STM32_RTC_BK0R
#define PX4_BBSRAM_SIZE   128  /* STM32G4 has 32 backup registers * 4 bytes */
#define PX4_BBSRAM_SIGNATURE       0xa5a5a5a5
#define PX4_BBSRAM_SIGNATURE_MASK  0xffffff00

__END_DECLS


