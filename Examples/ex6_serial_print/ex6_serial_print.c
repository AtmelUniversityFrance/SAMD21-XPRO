/*
 * ex6_serial_print.c
 *  Author: thibaut.viard
 */

#include "sam.h"

// Board configuration
//
// PA22 SERCOM3 UART TX -> EDBG USB CDC
// PA23 SERCOM3 UART RX -> EDBG USB CDC
//

/*
 * I/O Ports definitions
 */
#define PORTA     (0ul)
#define PORTB     (1ul)

/*
 * UART TX definitions
 */
#define UART_TX_PORT         PORTA
#define UART_TX_PIN_NUMBER   (22ul)
#define UART_TX_PIN          PORT_PA22 // (1ul << 22ul)
#define UART_TX_MUX          MUX_PA22C_SERCOM3_PAD0 // (2ul)
#define UART_TX_PAD          (0x0ul) // PAD0

/*
 * UART RX definitions
 */
#define UART_RX_PORT         PORTA
#define UART_RX_PIN_NUMBER   (23ul)
#define UART_RX_PIN          PORT_PA23 // (1ul << 23ul)
#define UART_RX_MUX          MUX_PA23C_SERCOM3_PAD1 // (2ul)
#define UART_RX_PAD          (0x1ul) // PAD1

#define UART_BAUDRATE        (115200ul)

// Global state variable for tick count
static uint32_t ul_tickcount=0 ;


static void assign_Pin2Peripheral( uint32_t ulPort, uint32_t ulPinNumber, uint32_t ulPeripheral )
{
  // is pin odd?
  if ( ulPinNumber & 1 )
  {
    uint32_t temp ;

    // Get whole current setup for both odd and even pins and remove odd one
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
  // Provide clock to SERCOMx
  PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0 | PM_APBCMASK_SERCOM1 | PM_APBCMASK_SERCOM2 | PM_APBCMASK_SERCOM3 | PM_APBCMASK_SERCOM4 | PM_APBCMASK_SERCOM5 ;

  // Start the Software Reset
  SERCOM3->USART.CTRLA.bit.SWRST = 1 ;

  while ( SERCOM3->USART.CTRLA.bit.SWRST || SERCOM3->USART.SYNCBUSY.bit.SWRST )
  {
    // Wait for both bits Software Reset from CTRLA and SYNCBUSY coming back to 0
  }

  // Set NVIC
  NVIC_EnableIRQ( SERCOM3_IRQn ) ;
//  NVIC_SetPriority( SERCOM3_IRQn, (1<<__NVIC_PRIO_BITS) - 1 ) ;  /* set Priority */

  // Peripheral clock configuration 
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID( GCLK_CLKCTRL_ID_SERCOM3_CORE_Val ) | // Generic Clock 0 (SERCOM3)
                      GCLK_CLKCTRL_GEN_GCLK0 | // Generic Clock Generator 0 is source
                      GCLK_CLKCTRL_CLKEN ;

  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
  {
    /* Wait for synchronization */
  }

  assign_Pin2Peripheral( UART_TX_PORT, UART_TX_PIN_NUMBER, UART_TX_MUX ) ;
  assign_Pin2Peripheral( UART_RX_PORT, UART_RX_PIN_NUMBER, UART_RX_MUX ) ;

  // Sett the CTRLA register
  SERCOM3->USART.CTRLA.reg =	SERCOM_USART_CTRLA_MODE_USART_INT_CLK | SERCOM_USART_CTRLA_SAMPR( 0x0 ) ; // 16x over-sampling using arithmetic baud rate generation

  // Set the Interrupt register
  SERCOM3->USART.INTENSET.reg =	SERCOM_USART_INTENSET_RXC |  // Receive complete
                                SERCOM_USART_INTENSET_ERROR ; // All others errors

  // Asynchronous arithmetic mode, sampleRateValue=16
  //
  // 65535 * ( 1 - sampleRateValue * baudrate / SystemCoreClock)
  SERCOM3->USART.BAUD.reg = 65535.0f * ( 1.0f - (16.0f) * (float)(UART_BAUDRATE) / (float)(SystemCoreClock));

  // Set the CTRLA register with 'no parity', LSB first
  SERCOM3->USART.CTRLA.reg |=	SERCOM_USART_CTRLA_FORM( 0x0ul ) | SERCOM_USART_CTRLA_DORD ;

  // Set the CTRLB register with '8bits', '1 stop', 'no parity'
  SERCOM3->USART.CTRLB.reg |=	SERCOM_USART_CTRLB_CHSIZE( 0x0ul ) ; 

  // Set the CTRLA register with TX PAD 2 and RX PAD 3
  SERCOM3->USART.CTRLA.reg |= SERCOM_USART_CTRLA_TXPO( UART_TX_PAD ) | SERCOM_USART_CTRLA_RXPO( UART_RX_PAD ) ;

  // Enable Transceiver and Receiver
  SERCOM3->USART.CTRLB.reg |= SERCOM_USART_CTRLB_TXEN | SERCOM_USART_CTRLB_RXEN ;

  // Enable USART
  SERCOM3->USART.CTRLA.bit.ENABLE = 0x1u ;

  // Wait for the enable bit from SYNCBUSY being equal to 0
  while ( SERCOM3->USART.SYNCBUSY.bit.ENABLE ) ;

// --------------------------------------------------------------------------------------------------------------------

  // Set Systick to 1ms interval, common to all Cortex-M variants
  if ( SysTick_Config( SystemCoreClock / 1000 ) )
  {
    // Capture error
    while ( 1 ) ;
  }

  NVIC_EnableIRQ( SysTick_IRQn ) ;
}

/**
 *  \brief SysTick Interrupt handler.
 */
void SysTick_Handler( void )
{
  ul_tickcount++ ;

  // Send characters every second (ie 1000ms)
  if ( ul_tickcount % 1000 == 0 )
  {
    SERCOM3->USART.DATA.reg = (uint16_t)'T' ;
    // Wait for transmission to complete
    while ( SERCOM3->USART.INTFLAG.bit.DRE != SERCOM_USART_INTFLAG_DRE ) ;

    SERCOM3->USART.DATA.reg = (uint16_t)'E' ;
    // Wait for transmission to complete
    while ( SERCOM3->USART.INTFLAG.bit.DRE != SERCOM_USART_INTFLAG_DRE ) ;

    SERCOM3->USART.DATA.reg = (uint16_t)'S' ;
    // Wait for transmission to complete
    while ( SERCOM3->USART.INTFLAG.bit.DRE != SERCOM_USART_INTFLAG_DRE ) ;

    SERCOM3->USART.DATA.reg = (uint16_t)'T' ;
    // Wait for transmission to complete
    while ( SERCOM3->USART.INTFLAG.bit.DRE != SERCOM_USART_INTFLAG_DRE ) ;

    SERCOM3->USART.DATA.reg = (uint16_t)'\r' ;
    // Wait for transmission to complete
    while ( SERCOM3->USART.INTFLAG.bit.DRE != SERCOM_USART_INTFLAG_DRE ) ;

    SERCOM3->USART.DATA.reg = (uint16_t)'\n' ;
    // Wait for transmission to complete
    while ( SERCOM3->USART.INTFLAG.bit.DRE != SERCOM_USART_INTFLAG_DRE ) ;
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


