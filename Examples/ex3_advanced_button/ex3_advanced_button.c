/*
 * ex3_advanced_button.c
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
#define SW0_PIN              PORT_PA15 // (1ul << 15ul)
#define SW0_MUX              MUX_PA15A_EIC_EXTINT15 // (0ul)

/* Configure I/O interrupt sources */
static void initialize_ExternalInt( void )
{
  NVIC_DisableIRQ( EIC_IRQn ) ;
  NVIC_ClearPendingIRQ( EIC_IRQn ) ;
  NVIC_SetPriority( EIC_IRQn, 0 ) ;
  NVIC_EnableIRQ( EIC_IRQn ) ;

  // Enable GCLK for IEC (External Interrupt Controller)
  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID( GCLK_CLKCTRL_ID_EIC_Val )) ;

  // Do a software reset on EIC
  EIC->CTRL.bit.SWRST = 1 ;

  while ( (EIC->CTRL.bit.SWRST == 1) && (EIC->STATUS.bit.SYNCBUSY == 1) )
  {
    // Waiting for synchronization
  }


  // Enable EIC
  EIC->CTRL.bit.ENABLE = 1 ;

  while ( EIC->STATUS.bit.SYNCBUSY == 1 )
  {
    // Waiting for synchronization
  }
}

static void assign_Pin2Peripheral( uint32_t ulPort, uint32_t ulPinNumber, uint32_t ulPeripheral )
{
  // is pin odd?
  if ( ulPinNumber & 1 )
  {
    uint32_t temp ;

	  // Get whole current setup for both odd and even pins and remove even one
	  temp = (PORT->Group[ulPort].PMUX[ulPinNumber >> 1].reg) & PORT_PMUX_PMUXE( 0xF ) ;
    // Set new multiplexing
    PORT->Group[ulPort].PMUX[ulPinNumber >> 1].reg = temp|PORT_PMUX_PMUXO( ulPeripheral ) ;
    // Enable port multiplexing
    PORT->Group[ulPort].PINCFG[ulPinNumber].reg |= PORT_PINCFG_PMUXEN ;
  }
  else // even pin
  {
    uint32_t temp ;

    // Get whole current setup for both odd and even pins and remove odd one
    temp = (PORT->Group[ulPort].PMUX[ulPinNumber >> 1].reg) & PORT_PMUX_PMUXO( 0xF ) ;
      // Set new multiplexing
    PORT->Group[ulPort].PMUX[ulPinNumber >> 1].reg = temp|PORT_PMUX_PMUXE( ulPeripheral ) ;
    // Enable port multiplexing
    PORT->Group[ulPort].PINCFG[ulPinNumber].reg |= PORT_PINCFG_PMUXEN ;
  }
}

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

  // Assign the SW0 pin as INPUT
  PORT->Group[SW0_PORT].DIRCLR.reg = SW0_PIN ;
  // Enable the internal pull-up resistance for SW0 as it is connected to ground
  PORT->Group[SW0_PORT].PINCFG[SW0_PIN_NUMBER].reg = PORT_PINCFG_PULLEN|PORT_PINCFG_INEN ;
  PORT->Group[SW0_PORT].OUTSET.reg = SW0_PIN ;
  // Activate a better input sampling clock
  PORT->Group[SW0_PORT].CTRL.reg = SW0_PIN ;

  assign_Pin2Peripheral( SW0_PORT, SW0_PIN_NUMBER, SW0_MUX ) ;
  initialize_ExternalInt() ;

  // Configure the interrupt mode
  EIC->CONFIG[1].reg |= EIC_CONFIG_SENSE7_RISE | EIC_CONFIG_FILTEN7 ;

  // Enable the interrupt
  EIC->INTENSET.reg = EIC_INTENSET_EXTINT( EIC_INTENSET_EXTINT15 ) ;
}

/*
 * External Interrupt Controller NVIC Interrupt Handler
 */
void EIC_Handler( void )
{
  if ( (EIC->INTFLAG.reg & (1ul << 15) ) != 0 )
  {
    // Toggle LED pin output level.
    PORT->Group[LED0_PORT].OUTTGL.reg = LED0_PIN ;

    // Clear the interrupt
    EIC->INTFLAG.reg = (1ul << 15) ;
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


