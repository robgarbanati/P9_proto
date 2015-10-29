/*---------------------------------------------------------------------------------------------------------*/
/*																										   */
/* Copyright (c) Nuvoton Technology	Corp. All rights reserved.											   */
/*																										   */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/* Include related header files                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#include "SFR.h"

// ARM CC SFR register definition
SYS_T		SYS			__attribute__((at(SYS_BASE), zero_init));
CLK_T		CLK			__attribute__((at(CLK_BASE), zero_init));
SYSINT_T	SYSINT		__attribute__((at(INT_BASE), zero_init));
GPIO_T		GPIOA		__attribute__((at(GPIOA_BASE), zero_init));
GPIO_T		GPIOB		__attribute__((at(GPIOB_BASE), zero_init));
FMC_T		FMC			__attribute__((at(FMC_BASE), zero_init));
WDT_T		WDT			__attribute__((at(WDT_BASE), zero_init));
RTC_T		RTC			__attribute__((at(RTC_BASE), zero_init));
TMR_T		TMR0		__attribute__((at(TMR0_BASE), zero_init));
TMR_T		TMR1		__attribute__((at(TMR1_BASE), zero_init));
TMRF_T		TMRF		__attribute__((at(TMRF_BASE), zero_init));
TMR_IR_T	TMRIR		__attribute__((at(TMR_IR_BASE), zero_init));
TMR_T		TMR2		__attribute__((at(TMR2_BASE), zero_init));
SPI_T		SPI0		__attribute__((at(SPI0_BASE), zero_init));
SPI_T		SPI1		__attribute__((at(SPI1_BASE), zero_init));
PWM_T		PWM			__attribute__((at(PWM_BASE), zero_init));
#if defined(__N572F065__)
USBD_T		USBD		__attribute__((at(USBD_BASE), zero_init));
#endif
APU_T		APU			__attribute__((at(APU_BASE), zero_init));
ADC_T		ADC			__attribute__((at(ADC_BASE), zero_init));
