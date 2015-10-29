/*---------------------------------------------------------------------------------------------------------*/
/*																										   */
/* Copyright (c) Nuvoton Technology	Corp. All rights reserved.											   */
/*																										   */
/*---------------------------------------------------------------------------------------------------------*/

#ifndef	__N572_H__
#define	__N572_H__
/*
 * ==========================================================================
 * ---------- Interrupt	Number Definition -----------------------------------
 * ==========================================================================
 */
 
typedef	enum IRQn
{
/******	 Cortex-M0 Processor Exceptions	Numbers	***************************************************/
	NonMaskableInt_IRQn		= -14,		/*!< 2 Non Maskable	Interrupt							  */
	HardFault_IRQn			= -13,		/*!< 3 Cortex-M0 Hard Fault	Interrupt					  */
	SVCall_IRQn				= -5,		/*!< 11	Cortex-M0 SV Call Interrupt						  */
	PendSV_IRQn				= -2,		/*!< 14	Cortex-M0 Pend SV Interrupt						  */
	SysTick_IRQn			= -1,		/*!< 15	Cortex-M0 System Tick Interrupt					  */

/******	 N572 specific Interrupt Numbers **********************************************************/
	WDT_IRQn				= 0,
	APU_IRQn				= 1,
	ADC_IRQn				= 2,
	EXINT_IRQn				= 3,
	USB_IRQn				= 4,
	TMR0_IRQn				= 5,
	TMR1_IRQn				= 6,
	TMR2_IRQn				= 7,
	GPAB_IRQn				= 8,
	SPI0_IRQn				= 9,
	PWM0_IRQn				= 10,
	SPI1_IRQn				= 11,
	TMRF_IRQn				= 12,
	RTC_IRQn				= 13,
	PWRWU_IRQn				= 14
										/*!< maximum of	32 Interrupts are possible				  */
} IRQn_Type;


/*
 * ==========================================================================
 * ----------- Processor and Core Peripheral Section ------------------------
 * ==========================================================================
 */

/* Configuration of	the	Cortex-M0 Processor	and	Core Peripherals */
#define	__MPU_PRESENT			0		/*!< armikcmu does not provide a MPU present or	not		*/
#define	__NVIC_PRIO_BITS		2		/*!< armikcmu Supports 2 Bits for the Priority Levels	*/
#define	__Vendor_SysTickConfig	0		/*!< Set to	1 if different SysTick Config is used		*/
#include "core_cm0.h"					/* Cortex-M0 processor and core	peripherals				*/

/******************************************************************************/
/*                Device Specific Peripheral registers structures             */
/******************************************************************************/
#include "SFR.h"						/* Extended peripherals									*/

#endif
																								 
