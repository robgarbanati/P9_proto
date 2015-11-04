/*---------------------------------------------------------------------------------------------------------*/
/*																										   */
/* Copyright (c) Nuvoton Technology	Corp. All rights reserved.											   */
/*																										   */
/*---------------------------------------------------------------------------------------------------------*/

#ifndef	__ADC_H__
#define	__ADC_H__

/*-----------------------------	ADC	Controller -------------------------------*/
// ADCDRx(A/D Data	Register)
#define	ADC_ADCDR_RSLT			BIT(15, 0)	// A/D Conversion Result
#define	ADC_ADCDR_EXTS			BIT(15,12)	// Extension bits of RSLT for different	data format
#define	ADC_ADCDR_OVERRUN		BIT16		// Over	Run	Flag
#define	ADC_ADCDR_VALID			BIT17		// Valid Flag

// ADCR(A/D Control Register)
#define	ADC_ADCR_ADEN			BIT0		// A/D Converter Enable
#define	ADC_ADCR_ADIE			BIT1		// A/D Interrupt Enable
#define	ADC_ADCR_ADMD			BIT(3,2)	// A/D Converter Operation Mode
#define	ADC_ADCR_TRGCOND		BIT(7,6)	// Trigger Condition
#define	ADC_ADCR_TRGE			BIT8		// Trigger Enable
#define	ADC_ADCR_ADST			BIT11		// A/D Conversion Start
#define	ADC_ADCR_ADRM			BIT12		// Data	Format of ADC Conversion Result

// ADCHSR(A/D Channel Sequence	Register)
#define	ADC_ADCHSR_CSEQ0		BIT(3,0)	// Select channel n	as the 1st conversion in scan sequence
#define	ADC_ADCHSR_CSEQ1		BIT(7,4)	// Select channel n	as the 2nd conversion in scan sequence
#define	ADC_ADCHSR_CSEQ2		BIT(11,8)	// Select channel n	as the 3nd conversion in scan sequence
#define	ADC_ADCHSR_CSEQ3		BIT(15,12)	// Select channel n	as the 4th conversion in scan sequence
#define	ADC_ADCHSR_CSEQ4		BIT(19,16)	// Select channel n	as the 5th conversion in scan sequence
#define	ADC_ADCHSR_CSEQ5		BIT(23,20)	// Select channel n	as the 6th conversion in scan sequence
#define	ADC_ADCHSR_CSEQ6		BIT(27,24)	// Select channel n	as the 7th conversion in scan sequence
#define	ADC_ADCHSR_CSEQ7		BIT(31,28)	// Select channel n	as the 8th conversion in scan sequence

// ADCMPRx(A/D Compare Register	0~1)
#define	ADC_ADCMPR_CMPEN		BIT0		// Compare Enable
#define	ADC_ADCMPR_CMPIE		BIT1		// Compare Interrupt Enable
#define	ADC_ADCMPR_CMPCOND		BIT2		// Compare Condition
#define	ADC_ADCMPR_CMPCH		BIT(5,3)	// Compare Channel Selection
#define	ADC_ADCMPR_CMPMATCNT	BIT(11,8)	// Compare Match Count
#define	ADC_ADCMPR_CMPD			BIT(27,16)	// Compare Data

// ADSR(A/D Status Register)
#define	ADC_ADSR_ADF			BIT0		// A/D Conversion End Flag
#define	ADC_ADSR_CMPF0			BIT1		// Compare Flag0
#define	ADC_ADSR_CMPF1			BIT2		// Compare Flag1
#define	ADC_ADSR_BUSY			BIT3		// BUSY/IDLE
#define	ADC_ADSR_CHANNEL		BIT(6,4)	// Current Conversion Channel
#define	ADC_ADSR_VALID			BIT(15,8)	// Data	Valid flag
#define	ADC_ADSR_OVERRUN		BIT(23,16)	// Over	Run	flag
/*
// ADCALR(A/D Calibration Register)
#define	ADC_ADCALR_CALEN		BIT0		// Self	Calibration	Enable
#define	ADC_ADCALR_CALDONE		BIT1		// Calibration is Done (read only)
*/
// PGCR(Gain Control Register0
#define ADC_PGCR_OPMUTE		BIT0				// Input switch control bit
#define ADC_PGCR_PAG_II		BIT(2,1)		// Gain setting bits for the second stage of pre-amp
#define ADC_PGCR_OS				BIT(16,8)		// Recorder OP offset control bit
#define ADC_PGCR_MICE			BIT22				// MIC_BIAS output enable.
#define ADC_PGCR_APPS			BIT23				// ADC and pre-amplifier power source selection.
#define ADC_PGCR_PAG_I		BIT(28,24)		// Gain setting bits for the first stage of pre-amp

// MIC Bias and PGC Control Register
#define ADC_MIBSCR_FWU		BIT(1,0)		// Fast wake up for RC
#define ADC_MIBSCR_CTRS		BIT(5,4)		// AVDD/2 accelerating

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__I	 uint32_t  RSLT:16;				// 12 bits A/D Conversion Result+ 
											// 4 Extension bits of RSLT for different data format
		__I	 uint32_t  OVERRUN:1;			// Over	Run	Flag
		__I	 uint32_t  VALID:1;				// Valid Flag
	};
} ADC_ADDR_T;

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__IO uint32_t  ADEN:1;				// A/D Converter Enable
		__IO uint32_t  ADIE:1;				// A/D Interrupt Enable
		__IO uint32_t  ADMD:2;				// A/D Converter Operation Mode
		__I	 uint32_t  RESERVE0:2;
		__IO uint32_t  TRGCOND:2;			// Trigger Condition
		__IO uint32_t  TRGEN:1;				// Trigger Enable
		__I	 uint32_t  RESERVE1:2;
		__IO uint32_t  ADST:1;				// A/D Conversion Start
		__IO uint32_t  ADFM:1;				// Data	Format of ADC Conversion Result
	};
} ADC_ADCR_T;

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__IO uint32_t  CSEQ0:4;				// Select channel n	as the 1st conversion in scan sequence
		__IO uint32_t  CSEQ1:4;				// Select channel n	as the 2nd conversion in scan sequence
		__IO uint32_t  CSEQ2:4;				// Select channel n	as the 3nd conversion in scan sequence
		__IO uint32_t  CSEQ3:4;				// Select channel n	as the 4th conversion in scan sequence
		__IO uint32_t  CSEQ4:4;				// Select channel n	as the 5th conversion in scan sequence
		__IO uint32_t  CSEQ5:4;				// Select channel n	as the 6th conversion in scan sequence
		__IO uint32_t  CSEQ6:4;				// Select channel n	as the 7th conversion in scan sequence
		__IO uint32_t  CSEQ7:4;				// Select channel n	as the 8th conversion in scan sequence
	};
} ADC_ADCHSR_T;

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__IO uint32_t  CMPEN:1;				// Compare Enable
		__IO uint32_t  CMPIE:1;				// Compare Interrupt Enable
		__IO uint32_t  CMPCOND:1;			// Compare Condition
		__IO uint32_t  CMPCH:3;				// Compare Channel Selection
		__I	 uint32_t  RESERVE0:2;
		__IO uint32_t  CMPMATCNT:4;			// Compare Match Count
		__I	 uint32_t  RESERVE1:4;
		__IO uint32_t  CMPD:12;				// Compare Data
		__I	 uint32_t  RESERVE2:4;
	};
} ADC_ADCMPR_T;

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__IO uint32_t  ADF:1;				// A/D Conversion End Flag
		__IO uint32_t  CMPF0:1;				// Compare Flag0
		__IO uint32_t  CMPF1:1;				// Compare Flag1
		__IO uint32_t  BUSY:1;				// BUSY/IDLE
		__IO uint32_t  CHANNEL:3;			// Current Conversion Channel
		__I	 uint32_t  RESERVE0:1;
		__IO uint32_t  VALID:8;				// Data	Valid flag
		__IO uint32_t  OVERRUN:8;			// Over	Run	flag
		__I	 uint32_t  RESERVE1:8;
	};
} ADC_ADSR_T;

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__IO uint32_t  OPMUTE:1;			// Input switch control bit
		__IO uint32_t  PAG_II:2;			// Gain setting bits for the second stage of pre-amp
		__I	 uint32_t  RESERVE0:5;
		__IO uint32_t  OS:9;				// Recorder OP offset control bit
		__I	 uint32_t  RESERVE1:5;
		__IO uint32_t	 MICE:1;
		__IO uint32_t	 APPS:1;	
		__IO uint32_t  PAG_1:5;				// Gain setting bits for the first stage of pre-amp
	};
} ADC_PGCR_T;

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__IO uint32_t  FWU:2;					// Fast wake up for RC
		__I	 uint32_t  RESERVE0:2;
		__IO uint32_t  CTRS:2;				// AVDD/2 accelerating
		__I	 uint32_t  RESERVE1:26;   
	};
} ADC_MIBSCR_T;

typedef	struct
{
	ADC_ADDR_T		ADDR[8];					// A/D Data	Register 0~7
	ADC_ADCR_T		ADCR;							// A/D Control Register
	ADC_ADCHSR_T	ADCHSR;						// A/D Channel Sequence	Register
	ADC_ADCMPR_T	ADCMPR[2];				// A/D Compare Register	0~1
	ADC_ADSR_T		ADSR;							// A/D Status Register
	__I	 uint32_t	RESERVE0[2];
	ADC_PGCR_T		PGCR;							// Gain Control Register
	ADC_MIBSCR_T	MIBSCR;						// MIC bias and PGC Control Register
} ADC_T;

#endif
