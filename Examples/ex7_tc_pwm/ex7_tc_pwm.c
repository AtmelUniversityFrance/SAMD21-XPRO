/*
 * ex7_tc_pwm.c
 *  Author: thibaut.viard
 */

#include "sam.h"

// Board configuration
//
// PA20 TC7/WO[0] -> pin DGI_GPIO2 on EDBG (Atmel Data Visualizer)
// PB30 TCC0/WO[0] -> pin 5 on EXT3
//

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
 * TC7/WO[0] definitions
 */
#define TC7_PORT             PORTA
#define TC7_PIN_NUMBER       (20ul)
#define TC7_PIN              PORT_PA20 // (1ul << 20ul)
#define TC7_MUX              MUX_PA20E_TC7_WO0 // (4ul)
#define TC7_CHANNEL          (0ul)

/*
 * TCC0/WO[0] definitions
 */
#define TCC0_PORT             PORTB
#define TCC0_PIN_NUMBER       (30ul)
#define TCC0_PIN              PORT_PB30 // (1ul << 30ul)
#define TCC0_MUX              MUX_PB30E_TCC0_WO0 // (4ul)
#define TCC0_CHANNEL          (0ul)

#define PWM_DUTY_CYCLE        (64ul)

// Global state variable for tick count
static volatile uint32_t ul_tickcount=0 ;

// Global state variable for PWM Duty cycle
static volatile uint32_t ul_duty_cycle=0 ;

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
  // Assign the LED0 pin as OUTPUT
  PORT->Group[LED0_PORT].DIRSET.reg = LED0_PIN ;
  // Set the LED0 pin level, ie put to 3.3V -> this light off the LED
  PORT->Group[LED0_PORT].OUTSET.reg = LED0_PIN ;
  
  // Clock TC/TCC
  PM->APBCMASK.reg |= PM_APBCMASK_TCC0 | PM_APBCMASK_TCC1 | PM_APBCMASK_TCC2 | PM_APBCMASK_TC3 | PM_APBCMASK_TC4 | PM_APBCMASK_TC5 | PM_APBCMASK_TC6 | PM_APBCMASK_TC7 ;

  // Enable GCLK for TCC0 and TCC1 (timer counter input clock)
  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TCC0_TCC1 ) ;
  while ( GCLK->STATUS.bit.SYNCBUSY == 1 ) ;

  assign_Pin2Peripheral( TCC0_PORT, TCC0_PIN_NUMBER, TCC0_MUX ) ;

  // -- Configure TCC0
  // Disable TCC0
  TCC0->CTRLA.reg &= ~TCC_CTRLA_ENABLE ;
  // Set TCC0 as normal PWM
  TCC0->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM ;
  // Set TCC0 in waveform mode Normal PWM
  TCC0->CC[TCC0_CHANNEL].reg = (uint32_t)0 ;
  // Set PER to maximum counter value (resolution : 0xFF)
  TCC0->PER.reg = 0xFF ;
  // Enable TCC0
  TCC0->CTRLA.reg |= TCC_CTRLA_ENABLE ;

#if 0  
  // Enable GCLK for TC6 and TC7 (timer counter input clock)
  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID_TC6_TC7 ) ;
  while ( GCLK->STATUS.bit.SYNCBUSY == 1 ) ;

  assign_Pin2Peripheral( TC7_PORT, TC7_PIN_NUMBER, TC7_MUX ) ;

  // Disable TC7
  TC7->COUNT8.CTRLA.reg &= ~TC_CTRLA_ENABLE ;
  // Set Timer counter Mode to 8 bits
  TC7->COUNT8.CTRLA.reg |= TC_CTRLA_MODE_COUNT8 ;
  // Set TC7 as normal PWM
  TC7->COUNT8.CTRLA.reg |= TC_CTRLA_WAVEGEN_NPWM ;
  // Set TC7 in waveform mode Normal PWM
  TC7->COUNT8.CC[TC7_CHANNEL].reg = (uint8_t)0 ;
  // Set PER to maximum counter value (resolution : 0xFF)
  TC7->COUNT8.PER.reg = 0xFF ;
  // Enable TC7
  TC7->COUNT8.CTRLA.reg |= TC_CTRLA_ENABLE ;
#endif 

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

  // Change PWM duty cycle every 30ms
  if ( ul_tickcount % 30 == 0 )
  {
    ul_duty_cycle += 4 ;
    if ( ul_duty_cycle > 0xff )
    {
      ul_duty_cycle=0 ;
    }
  
#if 0    
    // Set TC7 in waveform mode Normal PWM
    TC7->COUNT8.CC[TC7_CHANNEL].reg = (uint8_t)ul_duty_cycle ;
#endif

    // Set TCC0 in waveform mode Normal PWM
    TCC0->CC[TCC0_CHANNEL].reg = (uint32_t)ul_duty_cycle ;
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


