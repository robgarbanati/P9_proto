/*---------------------------------------------------------------------------------------------------------*/
/*																										   */
/* Copyright (c) Nuvoton Technology	Corp. All rights reserved.											   */
/*																										   */
/*---------------------------------------------------------------------------------------------------------*/

#ifndef	__RTC_H__
#define	__RTC_H__

/*-----------------------------	RTC	Controller -------------------------------*/
// RTCR(RTC Control Register)
#define RTC_RTCR_RTIF	BIT0			// RTC Interrupt Flag
#define RTC_RTCR_RTIE	BIT1			// RTC Interrupt Enable
#define RTC_RTCR_RTCE	BIT2			// RTC Enable
#define RTC_RTCR_RTIS	BIT(4,3)		// RTC Timer Interval Select

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__IO uint32_t	RTIF:1;			// RTC Interrupt Flag
		__IO uint32_t	RTIE:1;			// RTC Interrupt Enable
		__IO uint32_t	RTCE:1;			// RTC Enable
		__IO uint32_t	RTIS:2;			// RTC Timer Interval Select
	};
} RTCR_T;

typedef	struct
{
	RTCR_T  RTCR;
} RTC_T;

#endif
