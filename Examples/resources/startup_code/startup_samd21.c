#include "sam.h"

/* Initialize segments */
extern uint32_t __etext ;
extern uint32_t __data_start__ ;
extern uint32_t __data_end__ ;
extern uint32_t __bss_start__ ;
extern uint32_t __bss_end__ ;
extern uint32_t __StackTop ;

extern int main(void);
extern void __libc_init_array(void);

/* Default empty handler */
void Dummy_Handler(void);

/* Cortex-M0+ core handlers */
#if defined DEBUG
void NMI_Handler( void )
{
  while ( 1 )
  {
  }
}

void HardFault_Handler( void )
{
  while ( 1 )
  {
  }
}

void SVC_Handler( void )
{
  while ( 1 )
  {
  }
}

void PendSV_Handler( void )
{
  while ( 1 )
  {
  }
}

void SysTick_Handler         ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
#else
void NMI_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void HardFault_Handler       ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SVC_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void PendSV_Handler          ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SysTick_Handler         ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
#endif //DEBUG

/* Peripherals handlers */
void PM_Handler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SYSCTRL_Handler         ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void WDT_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void RTC_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void EIC_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void NVMCTRL_Handler         ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void DMAC_Handler            ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
#ifdef ID_USB
void USB_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
#endif
void EVSYS_Handler           ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM0_Handler         ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM1_Handler         ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM2_Handler         ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void SERCOM3_Handler         ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
#ifdef ID_SERCOM4
void SERCOM4_Handler         ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
#endif
#ifdef ID_SERCOM5
void SERCOM5_Handler         ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
#endif
void TCC0_Handler            ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TCC1_Handler            ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TCC2_Handler            ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TC3_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TC4_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
void TC5_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
#ifdef ID_TC6
void TC6_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
#endif
#ifdef ID_TC7
void TC7_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
#endif
#ifdef ID_ADC
void ADC_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
#endif
#ifdef ID_AC
void AC_Handler              ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
#endif
#ifdef ID_DAC
void DAC_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
#endif
#ifdef ID_PTC
void PTC_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
#endif
#ifdef ID_I2S
void I2S_Handler             ( void ) __attribute__ ((weak, alias("Dummy_Handler")));
#endif

/* Exception Table */
__attribute__ ((section(".isr_vector")))
const DeviceVectors exception_table=
{
        /* Configure Initial Stack Pointer, using linker-generated symbols */
        .pvStack = (void*) (&__StackTop),

        .pfnReset_Handler      = (void*) Reset_Handler,
        .pfnNMI_Handler        = (void*) NMI_Handler,
        .pfnHardFault_Handler  = (void*) HardFault_Handler,
        .pfnReservedM12        = (void*) (0UL), /* Reserved */
        .pfnReservedM11        = (void*) (0UL), /* Reserved */
        .pfnReservedM10        = (void*) (0UL), /* Reserved */
        .pfnReservedM9         = (void*) (0UL), /* Reserved */
        .pfnReservedM8         = (void*) (0UL), /* Reserved */
        .pfnReservedM7         = (void*) (0UL), /* Reserved */
        .pfnReservedM6         = (void*) (0UL), /* Reserved */
        .pfnSVC_Handler        = (void*) SVC_Handler,
        .pfnReservedM4         = (void*) (0UL), /* Reserved */
        .pfnReservedM3         = (void*) (0UL), /* Reserved */
        .pfnPendSV_Handler     = (void*) PendSV_Handler,
        .pfnSysTick_Handler    = (void*) SysTick_Handler,

        /* Configurable interrupts */
        .pfnPM_Handler         = (void*) PM_Handler,             /*  0 Power Manager */
        .pfnSYSCTRL_Handler    = (void*) SYSCTRL_Handler,        /*  1 System Control */
        .pfnWDT_Handler        = (void*) WDT_Handler,            /*  2 Watchdog Timer */
        .pfnRTC_Handler        = (void*) RTC_Handler,            /*  3 Real-Time Counter */
        .pfnEIC_Handler        = (void*) EIC_Handler,            /*  4 External Interrupt Controller */
        .pfnNVMCTRL_Handler    = (void*) NVMCTRL_Handler,        /*  5 Non-Volatile Memory Controller */
        .pfnDMAC_Handler       = (void*) DMAC_Handler,           /*  6 Direct Memory Access Controller */
#ifdef ID_USB
        .pfnUSB_Handler        = (void*) USB_Handler,            /*  7 Universal Serial Bus */
#else
        .pfnUSB_Handler        = (void*) (0UL), /* Reserved */
#endif
        .pfnEVSYS_Handler      = (void*) EVSYS_Handler,          /*  8 Event System Interface */
        .pfnSERCOM0_Handler    = (void*) SERCOM0_Handler,        /*  9 Serial Communication Interface 0 */
        .pfnSERCOM1_Handler    = (void*) SERCOM1_Handler,        /* 10 Serial Communication Interface 1 */
        .pfnSERCOM2_Handler    = (void*) SERCOM2_Handler,        /* 11 Serial Communication Interface 2 */
        .pfnSERCOM3_Handler    = (void*) SERCOM3_Handler,        /* 12 Serial Communication Interface 3 */
#ifdef ID_SERCOM4
        .pfnSERCOM4_Handler    = (void*) SERCOM4_Handler,        /* 13 Serial Communication Interface 4 */
#else
        .pfnSERCOM4_Handler    = (void*) (0UL), /* Reserved */
#endif
#ifdef ID_SERCOM5
        .pfnSERCOM5_Handler    = (void*) SERCOM5_Handler,        /* 14 Serial Communication Interface 5 */
#else
        .pfnSERCOM5_Handler    = (void*) (0UL), /* Reserved */
#endif
        .pfnTCC0_Handler       = (void*) TCC0_Handler,           /* 15 Timer Counter Control 0 */
        .pfnTCC1_Handler       = (void*) TCC1_Handler,           /* 16 Timer Counter Control 1 */
        .pfnTCC2_Handler       = (void*) TCC2_Handler,           /* 17 Timer Counter Control 2 */
        .pfnTC3_Handler        = (void*) TC3_Handler,            /* 18 Basic Timer Counter 0 */
        .pfnTC4_Handler        = (void*) TC4_Handler,            /* 19 Basic Timer Counter 1 */
        .pfnTC5_Handler        = (void*) TC5_Handler,            /* 20 Basic Timer Counter 2 */
#ifdef ID_TC6
        .pfnTC6_Handler        = (void*) TC6_Handler,            /* 21 Basic Timer Counter 3 */
#else
        .pfnTC6_Handler        = (void*) (0UL), /* Reserved */
#endif
#ifdef ID_TC7
        .pfnTC7_Handler        = (void*) TC7_Handler,            /* 22 Basic Timer Counter 4 */
#else
        .pfnTC7_Handler        = (void*) (0UL), /* Reserved */
#endif
#ifdef ID_ADC
        .pfnADC_Handler        = (void*) ADC_Handler,            /* 23 Analog Digital Converter */
#else
        .pfnADC_Handler        = (void*) (0UL), /* Reserved */
#endif
#ifdef ID_AC
        .pfnAC_Handler         = (void*) AC_Handler,             /* 24 Analog Comparators 0 */
#else
        .pfnAC_Handler         = (void*) (0UL), /* Reserved */
#endif
#ifdef ID_DAC
        .pfnDAC_Handler        = (void*) DAC_Handler,            /* 25 Digital Analog Converter */
#else
        .pfnDAC_Handler        = (void*) (0UL), /* Reserved */
#endif
#ifdef ID_PTC
        .pfnPTC_Handler        = (void*) PTC_Handler,            /* 26 Peripheral Touch Controller */
#else
        .pfnPTC_Handler        = (void*) (0UL), /* Reserved */
#endif
#ifdef ID_I2S
        .pfnI2S_Handler        = (void*) I2S_Handler,            /* 27 Inter-IC Sound Interface */
#else
        .pfnI2S_Handler        = (void*) (0UL), /* Reserved */
#endif
};

/**
 * \brief This is the code that gets called on processor reset.
 * To initialize the device, and call the main() routine.
 */
void Reset_Handler( void )
{
	uint32_t *pSrc, *pDest;

	/* Initialize the initialized data section */
	pSrc = &__etext;
	pDest = &__data_start__;

	if ( (&__data_start__ != &__data_end__) && (pSrc != pDest) )
	{
		for (; pDest < &__data_end__ ; pDest++, pSrc++ )
		{
			*pDest = *pSrc ;
		}
	}

	/* Clear the zero section */
	if ( (&__bss_start__ != &__bss_end__) && (pSrc != pDest) )
	{
		for ( pDest = &__bss_start__ ; pDest < &__bss_end__ ; pDest++ )
		{
			*pDest = 0 ;
		}
	}

	/* Initialize the C library */
	__libc_init_array();

	SystemInit() ;

	/* Branch to main function */
	main() ;

	/* Infinite loop */
	while ( 1 )
	{
	}
}

/**
 * \brief Default interrupt handler for unused IRQs.
 */
void Dummy_Handler(void)
{
    while ( 1 )
    {
    }
}
