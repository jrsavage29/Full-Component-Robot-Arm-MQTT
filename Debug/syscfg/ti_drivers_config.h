/*
 *  ======== ti_drivers_config.h ========
 *  Configured TI-Drivers module declarations
 *
 *  The macros defines herein are intended for use by applications which
 *  directly include this header. These macros should NOT be hard coded or
 *  copied into library source code.
 *
 *  Symbols declared as const are intended for use with libraries.
 *  Library source code must extern the correct symbol--which is resolved
 *  when the application is linked.
 *
 *  DO NOT EDIT - This file is generated for the CC3220SF_LAUNCHXL
 *  by the SysConfig tool.
 */
#ifndef ti_drivers_config_h
#define ti_drivers_config_h

#define CONFIG_SYSCONFIG_PREVIEW

#define CONFIG_CC3220SF_LAUNCHXL
#ifndef DeviceFamily_CC3220
#define DeviceFamily_CC3220
#endif

#include <ti/devices/DeviceFamily.h>

#include <stdint.h>

/* support C++ sources */
#ifdef __cplusplus
extern "C" {
#endif


/*
 *  ======== Crypto ========
 */

extern const uint_least8_t          CONFIG_Crypto_0_CONST;
#define CONFIG_Crypto_0             0

/*
 *  ======== GPIO ========
 */

/* P15 */
extern const uint_least8_t          CONFIG_GPIO_P15_BIT_1_CONST;
#define CONFIG_GPIO_P15_BIT_1       0
/* P63 */
extern const uint_least8_t          CONFIG_GPIO_P63_BIT_2_CONST;
#define CONFIG_GPIO_P63_BIT_2       1
/* P50 */
extern const uint_least8_t          CONFIG_GPIO_P50_BIT_3_CONST;
#define CONFIG_GPIO_P50_BIT_3       2
/* P53 */
extern const uint_least8_t          CONFIG_GPIO_P53_BIT_4_CONST;
#define CONFIG_GPIO_P53_BIT_4       3
/* P18 */
extern const uint_least8_t          CONFIG_GPIO_P18_BIT_5_CONST;
#define CONFIG_GPIO_P18_BIT_5       4
/* P03 */
extern const uint_least8_t          CONFIG_GPIO_P03_BIT_6_CONST;
#define CONFIG_GPIO_P03_BIT_6       5
/* P61 */
extern const uint_least8_t          CONFIG_GPIO_P61_BIT_7_CONST;
#define CONFIG_GPIO_P61_BIT_7       6
/* P62 */
extern const uint_least8_t          CONFIG_GPIO_P62_BIT_8_CONST;
#define CONFIG_GPIO_P62_BIT_8       7

/* LEDs are active high */
#define CONFIG_GPIO_LED_ON  (1)
#define CONFIG_GPIO_LED_OFF (0)

#define CONFIG_LED_ON  (CONFIG_GPIO_LED_ON)
#define CONFIG_LED_OFF (CONFIG_GPIO_LED_OFF)


/*
 *  ======== PWM ========
 */

/* P21 */
extern const uint_least8_t          CONFIG_PWM_0_CONST;
#define CONFIG_PWM_0                0
/* P01 */
extern const uint_least8_t          CONFIG_PWM_1_CONST;
#define CONFIG_PWM_1                1
/* P02 */
extern const uint_least8_t          CONFIG_PWM_2_CONST;
#define CONFIG_PWM_2                2
/* P64 */
extern const uint_least8_t          CONFIG_PWM_3_CONST;
#define CONFIG_PWM_3                3


/*
 *  ======== SPI ========
 */

extern const uint_least8_t          CONFIG_NWP_SPI_CONST;
#define CONFIG_NWP_SPI              0
/*
 *  MOSI: P07
 *  MISO: P06
 *  SCLK: P05
 *  SS: P08
 */
extern const uint_least8_t          CONFIG_SPI_0_CONST;
#define CONFIG_SPI_0                1


/*
 *  ======== Timer ========
 */

extern const uint_least8_t          CONFIG_TIMER_0_CONST;
#define CONFIG_TIMER_0              0

/*
 *  ======== UART ========
 */

/*
 *  TX: P55
 *  RX: P57
 *  XDS110 UART
 */
extern const uint_least8_t          CONFIG_UART_0_CONST;
#define CONFIG_UART_0               0


/*
 *  ======== Watchdog ========
 */

extern const uint_least8_t          CONFIG_WATCHDOG_0_CONST;
#define CONFIG_WATCHDOG_0           0


/*
 *  ======== Board_init ========
 *  Perform all required TI-Drivers initialization
 *
 *  This function should be called once at a point before any use of
 *  TI-Drivers.
 */
extern void Board_init(void);

/*
 *  ======== Board_initGeneral ========
 *  (deprecated)
 *
 *  Board_initGeneral() is defined purely for backward compatibility.
 *
 *  All new code should use Board_init() to do any required TI-Drivers
 *  initialization _and_ use <Driver>_init() for only where specific drivers
 *  are explicitly referenced by the application.  <Driver>_init() functions
 *  are idempotent.
 */
#define Board_initGeneral Board_init

#ifdef __cplusplus
}
#endif

#endif /* include guard */
