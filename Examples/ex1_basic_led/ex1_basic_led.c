/*
 * ex1_basic_led.c
 *  Author: thibaut.viard
 */

#include "sam.h"

/*
 * I/O Ports definitions
 */
#define PORTA     (0ul)
#define PORTB     (1ul)

/*
 * LED0 definitions
 */
#define LED0_PORT            PORTB
#define LED0_PIN_NUMBER      (30ul)
#define LED0_PIN             PORT_PB30 // (1<<30ul)

/**
 * \brief Application entry point.
 *
 * \return Unused (ANSI-C compatibility).
 */
int main(void)
{
  // Assign the LED0 pin as OUTPUT
  PORT->Group[LED0_PORT].DIRSET.reg = LED0_PIN ;

  // Clear the LED0 pin level, ie put to 0V -> the pin is connected to the LED cathode, anode is connected to +3.3V.
  PORT->Group[LED0_PORT].OUTCLR.reg = LED0_PIN ;

  return 0 ;
}

