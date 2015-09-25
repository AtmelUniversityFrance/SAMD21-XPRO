/*
 * ex5_clocks.c
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

// Global state variable for tick count
static uint32_t ul_tickcount=0 ;

/**
 * \brief Initialization of board components.
 *
 */
static void initialize_hardware( void )
{
  // Assign the LED0 pin as OUTPUT
  PORT->Group[LED0_PORT].DIRSET.reg = LED0_PIN ;
  // Set the LED0 pin level, ie put to 3.3V -> this light off the LED
  PORT->Group[LED0_PORT].OUTSET.reg = LED0_PIN ;

  // Configure Systick to trig every millisecond
  SysTick_Config( SystemCoreClock/1000UL ) ;
  NVIC_EnableIRQ( SysTick_IRQn ) ;
}

/**
 * \brief Toggle the LED.
 *
 */
static void toggle_LED( void )
{
  // Toggle LED pin output level.
  PORT->Group[LED0_PORT].OUTTGL.reg = LED0_PIN ;
}

/**
 *  \brief SysTick Interrupt handler.
 */
void SysTick_Handler( void )
{
  ul_tickcount++ ;

  // Toggle LEDs every second (ie 1000ms)
  if ( ul_tickcount % 1000 == 0 )
  {
    toggle_LED() ;
  }
}

/**
 * \brief Application entry point.
 *
 * \return Unused (ANSI-C compatibility).
 */
int main( void )
{
  // Initialize hardware
  initialize_hardware() ;

  while ( 1 )
  {
  }
}


