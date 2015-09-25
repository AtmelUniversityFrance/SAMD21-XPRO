#ifndef _SYSTEM_SAMD21_H_INCLUDED_
#define _SYSTEM_SAMD21_H_INCLUDED_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

extern uint32_t SystemCoreClock;   /*!< System Clock Frequency (Core Clock)  */

/**
 * @brief Setup the microcontroller system.
 * Initialize the System and update the SystemCoreClock variable.
 */
void SystemInit(void);

/**
 * @brief Updates the SystemCoreClock with current core Clock
 * retrieved from cpu registers.
 */
void SystemCoreClockUpdate(void);

#ifdef __cplusplus
}
#endif

#endif /* SYSTEM_SAMD21_H_INCLUDED */

