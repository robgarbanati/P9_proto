/****************************************************************
 *																*
 * Copyright (c) Nuvoton Technology	Corp. All rights reserved.	*
 *																*
 ****************************************************************/
#ifndef	__SFR_H__
#define	__SFR_H__

#include "NVTChipDef.h"

#if !defined(__N572F072__) && !defined(__N572P072__)
#warning "Select N572F072 as the target chip!. Please define __N572F072__ or __N572P072__ to select N572F072 or N572P072 as target chip!"
#define __N572F072__
#endif

#define	__N572__
#define	__CORTEX_M0__


/******************************************************************************/
/*						   Peripheral memory definitions					  */
/******************************************************************************/
#include "SYS.h"
#include "CLK.h"
#include "SYSINT.h"
#include "GPIO.h"
#include "FMC.h"
#include "WDT.h"
#include "RTC.h"
#include "TMR.h"
#include "SPI.h"
#include "PWM.h"
#include "APU.h"
#include "ADC.h"

/******************************************************************************/
/*						   Peripheral Registers 							  */
/******************************************************************************/
extern SYS_T		SYS;		// System Global Control Registers
extern CLK_T		CLK;		// Clock Control Registers
extern SYSINT_T		SYSINT;		// Interrupt Multiplexer Control Registers
extern GPIO_T		GPIOA;		// GPIOA Control Registers
extern GPIO_T		GPIOB;		// GPIOB Control Registers
extern FMC_T		FMC;		// Flash Memory	Control	Registers
extern WDT_T		WDT;		// Watch-Dog Timer Control Registers
extern RTC_T		RTC;		// Real Time Clock (RTC) Control Register
extern TMR_T		TMR0;		// Timer0 Control Registers
extern TMR_T		TMR1;		// Timer1 Control Registers
extern TMR_T		TMR2;		// Timer2 Control Registers
extern TMRF_T		TMRF;		// TimerF Control Registers
extern TMR_IR_T	TMRIR;		// Timer IR carrier Control Registers
extern SPI_T	SPI0;		// SPI0 Control Registers
extern SPI_T	SPI1;		// SPI1 Control Registers
extern PWM_T		PWM;		// PWM Control Registers
extern APU_T		APU;		// APU Controller Registers
extern ADC_T		ADC;		// Analog-Digital-Converter (ADC) Control Registers

/******************************************************************************/
/*						   Peripheral memory map							  */
/******************************************************************************/
#define	FLASH_BASE		(0x00000000)		// FLASH Memory	Space (72KB)
#define	SRAM_BASE			(0x20000000)		// SRAM	Memory Space (8KB)
/* AHB modules */
#define	SYS_BASE			(0x50000000)		// System Global Control Registers
#define	CLK_BASE			(0x50000200)		// Clock Control Registers
#define	INT_BASE			(0x50000300)		// Interrupt Multiplexer Control Registers
#define	GPIOA_BASE		(0x50004000)		// GPIOA Control Registers
#define	GPIOB_BASE		(0x50004040)		// GPIOB Control Registers
#define	APU_BASE			(0x50008000)		// APU Controller Registers
#define	FMC_BASE			(0x5000C000)		// Flash Memory	Control	Registers
/* APB modules */
#define	WDT_BASE			(0x40004000)		// Watch-Dog Timer Control Registers
#define	RTC_BASE			(0x40008000)		// Real Time Clock (RTC) Control Register
#define	TMR_BASE			(0x40010000)		// Timer Control Registers
#define	TMR0_BASE			(0x40010000)		// Timer0 Control Registers
#define	TMR1_BASE			(0x40010020)		// Timer1 Control Registers
#define	TMRF_BASE			(0x40010030)		// TimerF Control Registers
#define	TMR_IR_BASE		(0x40010034)		// Timer IR Control Registers
#define	TMR2_BASE			(0x40010040)		// Timer2 Control Registers
#define	SPI0_BASE			(0x40030000)		// SPI0 Control Registers
#define	SPI1_BASE			(0x40034000)		// SPI1 Control Registers
#define PWM_BASE			(0x40040000)		// PWM Control Registers
#define	ADC_BASE			(0x400E0000)		// Analog-Digital-Converter (ADC) Control Registers

#endif
