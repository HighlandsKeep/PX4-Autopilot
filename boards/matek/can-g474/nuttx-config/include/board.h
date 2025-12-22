/****************************************************************************
 * boards/matek/can-g474/include/board.h
 *
 ****************************************************************************/

#ifndef __BOARDS_ARM_STM32_MATEK_CAN_G474_INCLUDE_BOARD_H
#define __BOARDS_ARM_STM32_MATEK_CAN_G474_INCLUDE_BOARD_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
# include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Clocking *************************************************************************/
/* The Matek CAN-G474 uses an 8MHz crystal oscillator (matching ArduPilot config) */

#define STM32_BOARD_XTAL        8000000ul

#define STM32_HSI_FREQUENCY     16000000ul
#define STM32_LSI_FREQUENCY     32000
#define STM32_HSE_FREQUENCY     STM32_BOARD_XTAL
#define STM32_LSE_FREQUENCY     0

/* Main PLL Configuration.
 *
 * PLL source is HSE = 8MHz (matching ArduPilot)
 * PLLN = 85, PLLM = 2, PLLP = 10, PLLQ = 8, PLLR = 2
 *
 * f(VCO Clock) = f(PLL Clock Input) x (PLLN / PLLM)
 * f(PLL_P) = f(VCO Clock) / PLLP
 * f(PLL_Q) = f(VCO Clock) / PLLQ
 * f(PLL_R) = f(VCO Clock) / PLLR
 *
 * f(VCO Clock) = HSE   x PLLN / PLLM
 *              = 8MHz x 85   / 2
 *              = 340MHz
 *
 * PLLPCLK      = 340MHz / 10 = 34MHz
 * PLLQCLK      = 340MHz / 8  = 42.5MHz (FDCAN)
 * PLLRCLK      = 340MHz / 2  = 170MHz (SYSCLK)
 */

#define STM32_PLLCFGR_PLLSRC           RCC_PLLCFGR_PLLSRC_HSE
#define STM32_PLLCFGR_PLLCFG           (RCC_PLLCFGR_PLLPEN | \
                                       RCC_PLLCFGR_PLLQEN | \
                                       RCC_PLLCFGR_PLLREN)

#define STM32_PLLCFGR_PLLN             RCC_PLLCFGR_PLLN(85)
#define STM32_PLLCFGR_PLLM             RCC_PLLCFGR_PLLM(2)
#define STM32_PLLCFGR_PLLP             RCC_PLLCFGR_PLLPDIV(10)
#define STM32_PLLCFGR_PLLQ             RCC_PLLCFGR_PLLQ_8
#define STM32_PLLCFGR_PLLR             RCC_PLLCFGR_PLLR_2

#define STM32_VCO_FREQUENCY            ((STM32_HSE_FREQUENCY / 2) * 85)
#define STM32_PLLP_FREQUENCY           (STM32_VCO_FREQUENCY / 10)
#define STM32_PLLQ_FREQUENCY           (STM32_VCO_FREQUENCY / 8)
#define STM32_PLLR_FREQUENCY           (STM32_VCO_FREQUENCY / 2)

/* Use the PLL and set the SYSCLK source to be PLLR (170MHz) */

#define STM32_SYSCLK_SW                RCC_CFGR_SW_PLL
#define STM32_SYSCLK_SWS               RCC_CFGR_SWS_PLL
#define STM32_SYSCLK_FREQUENCY         STM32_PLLR_FREQUENCY

/* AHB clock (HCLK) is SYSCLK (170MHz) */

#define STM32_RCC_CFGR_HPRE            RCC_CFGR_HPRE_SYSCLK
#define STM32_HCLK_FREQUENCY           STM32_SYSCLK_FREQUENCY

/* APB1 clock (PCLK1) is HCLK/1 (170MHz) */

#define STM32_RCC_CFGR_PPRE1           RCC_CFGR_PPRE1_HCLK
#define STM32_PCLK1_FREQUENCY          (STM32_HCLK_FREQUENCY / 1)

/* APB2 clock (PCLK2) is HCLK/1 (170MHz) */

#define STM32_RCC_CFGR_PPRE2           RCC_CFGR_PPRE2_HCLK
#define STM32_PCLK2_FREQUENCY          (STM32_HCLK_FREQUENCY / 1)

/* APB1 timers 2-7 will receive PCLK1 */
#define STM32_APB1_TIM2_CLKIN   (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM3_CLKIN   (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM4_CLKIN   (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM5_CLKIN   (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM6_CLKIN   (STM32_PCLK1_FREQUENCY)
#define STM32_APB1_TIM7_CLKIN   (STM32_PCLK1_FREQUENCY)

/* APB2 timers 1, 8, 20, 15-17 will receive PCLK2 */
#define STM32_APB2_TIM1_CLKIN   (STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM8_CLKIN   (STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM15_CLKIN  (STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM16_CLKIN  (STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM17_CLKIN  (STM32_PCLK2_FREQUENCY)
#define STM32_APB2_TIM20_CLKIN  (STM32_PCLK2_FREQUENCY)

/* FDCAN clock (PLLQ = 42.5 MHz) */
#define STM32_FDCAN_FREQUENCY   STM32_PLLQ_FREQUENCY

/* FDCAN clock source - use PLLQ */
#define STM32_RCC_CCIPR_FDCANSEL RCC_CCIPR_FDCANSEL_PLLQ

/* LED definitions ******************************************************************/
#define GPIO_LED1       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz| \
			 GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN14)
#define GPIO_LED2       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz| \
			 GPIO_OUTPUT_SET|GPIO_PORTC|GPIO_PIN15)
#define GPIO_LED3       (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz| \
			 GPIO_OUTPUT_SET|GPIO_PORTA|GPIO_PIN8)

#define LED_STARTED       0
#define LED_HEAPALLOCATE  1
#define LED_IRQSENABLED   2
#define LED_STACKCREATED  3
#define LED_INIRQ         4
#define LED_SIGNAL        5
#define LED_ASSERTION     6
#define LED_PANIC         7
#define LED_IDLE          8

/* USART ***************************************************************************/
/* Matching ArduPilot MatekG474 pin assignments */
#define GPIO_USART1_TX (GPIO_USART1_TX_1|GPIO_SPEED_50MHz)   /* PA9 */
#define GPIO_USART1_RX (GPIO_USART1_RX_1|GPIO_SPEED_50MHz)   /* PA10 */

#define GPIO_USART2_TX (GPIO_USART2_TX_1|GPIO_SPEED_50MHz)   /* PB3 */
#define GPIO_USART2_RX (GPIO_USART2_RX_1|GPIO_SPEED_50MHz)   /* PB4 */

#define GPIO_USART3_TX (GPIO_USART3_TX_2|GPIO_SPEED_50MHz)   /* PB10 - ESC telemetry */
#define GPIO_USART3_RX (GPIO_USART3_RX_2|GPIO_SPEED_50MHz)   /* PB11 */

/* FDCAN (dual CAN support) ********************************************************/
#define GPIO_FDCAN1_RX (GPIO_FDCAN1_RX_2|GPIO_SPEED_50MHz)   /* PA11 */
#define GPIO_FDCAN1_TX (GPIO_FDCAN1_TX_2|GPIO_SPEED_50MHz)   /* PA12 */

#define GPIO_FDCAN2_RX (GPIO_FDCAN2_RX_1|GPIO_SPEED_50MHz)   /* PB5 */
#define GPIO_FDCAN2_TX (GPIO_FDCAN2_TX_1|GPIO_SPEED_50MHz)   /* PB6 */

/* SPI2 for RM3100 Compass (optional) **********************************************/
#define GPIO_SPI2_SCK  (GPIO_SPI2_SCK_3|GPIO_SPEED_50MHz)    /* PB13 */
#define GPIO_SPI2_MISO (GPIO_SPI2_MISO_2|GPIO_SPEED_50MHz)   /* PB14 */
#define GPIO_SPI2_MOSI (GPIO_SPI2_MOSI_2|GPIO_SPEED_50MHz)   /* PB15 */

/* I2C (for sensors) ***************************************************************/
#define GPIO_I2C1_SCL (GPIO_I2C1_SCL_3|GPIO_SPEED_50MHz)     /* PA13 */
#define GPIO_I2C1_SDA (GPIO_I2C1_SDA_2|GPIO_SPEED_50MHz)     /* PA14 */

#define GPIO_I2C2_SCL (GPIO_I2C2_SCL_2|GPIO_SPEED_50MHz)     /* PC4 */
#define GPIO_I2C2_SDA (GPIO_I2C2_SDA_2|GPIO_SPEED_50MHz)     /* PA8 */

/* DMA channels ********************************************************************/

#define DMACHAN_TIM1_CH1  STM32_DMA_CHAN(STM32_DMACHAN_TIM1_CH1_0)
#define DMACHAN_TIM1_CH2  STM32_DMA_CHAN(STM32_DMACHAN_TIM1_CH2_0)
#define DMACHAN_TIM1_CH3  STM32_DMA_CHAN(STM32_DMACHAN_TIM1_CH3_0)
#define DMACHAN_TIM1_CH4  STM32_DMA_CHAN(STM32_DMACHAN_TIM1_CH4_0)

#endif  /* __BOARDS_ARM_STM32_MATEK_CAN_G474_INCLUDE_BOARD_H */
