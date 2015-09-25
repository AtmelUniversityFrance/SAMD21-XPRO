/*
 * ex2_basic_button.c
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
#define LED0_PIN             PORT_PB30

/*
 * SW0 definitions
 */
#define SW0_PORT             PORTA
#define SW0_PIN_NUMBER       (15ul)
#define SW0_PIN              PORT_PA15

// Global state variable for LED toggle
static uint32_t ul_toggle=0 ;

/**
 * \brief Initialization of board components.
 *
 */
void initialize_hardware( void )
{
  // Assign the LED0 pin as OUTPUT
  PORT->Group[LED0_PORT].DIRSET.reg = LED0_PIN ;
  // Set the LED0 pin level, ie put to 3.3V -> this light off the LED
  PORT->Group[LED0_PORT].OUTSET.reg = LED0_PIN ;

  // Assign the SW0 pin as INPUT
  PORT->Group[SW0_PORT].DIRCLR.reg = SW0_PIN ;
  // Enable the internal pull-up resistance for SW0 as it is connected to ground
  PORT->Group[SW0_PORT].PINCFG[SW0_PIN_NUMBER].reg = PORT_PINCFG_PULLEN|PORT_PINCFG_INEN ;
  PORT->Group[SW0_PORT].OUTSET.reg = SW0_PIN ;
  // Activate a better input sampling clock
  PORT->Group[SW0_PORT].CTRL.reg = SW0_PIN ;
}

/**
 * \brief Toggle the LED.
 *
 */
static void toggle_LED( void )
{
  if ( ul_toggle == 0 )
  {
    // Clear the LED0 pin level, ie put to 0V -> the pin is connected to the LED cathode, anode is connected to +3.3V.
    PORT->Group[LED0_PORT].OUTCLR.reg = LED0_PIN ;

    ul_toggle=1 ;
  }
  else
  {
    // Set the LED0 pin level, ie put to 3.3V -> this light off the LED
    PORT->Group[LED0_PORT].OUTSET.reg = LED0_PIN ;

    ul_toggle=0 ;
  }
}

/**
 * \brief Application entry point.
 *
 * \return Unused (ANSI-C compatibility).
 */
int main( void )
{
  volatile uint32_t ul ;
	
  // Initialize hardware
  initialize_hardware() ;

  while ( 1 )
  {
    // Read the current value in IN register
	  ul=PORT->Group[SW0_PORT].IN.reg ;
	  // Mask the read value with the bit we want to evaluate
	  ul &= SW0_PIN ;

    // if button is pressed, I/O pin is connected to ground, ie IN value is 0
    if ( ul == 0 )
    {
      toggle_LED() ;
    }
  }
}


