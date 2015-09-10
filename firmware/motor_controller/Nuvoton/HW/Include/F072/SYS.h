/*---------------------------------------------------------------------------------------------------------*/
/*																										   */
/* Copyright (c) Nuvoton Technology	Corp. All rights reserved.											   */
/*																										   */
/*---------------------------------------------------------------------------------------------------------*/

#ifndef	__SYS_H__
#define	__SYS_H__

/*---------------------------- Global Controller -----------------------------*/
// PDID(Product Identification Register)

// RSTSRC(System Reset Source Register)
#define SYS_RSTSRC_RSTS_POR		BIT0					// The RSTS_POR flag is set by the ¡§reset signal¡¨ from the module POR to indicate the previous reset source.
#define SYS_RSTSRC_RSTS_PAD		BIT1					// The RSTS_PAD flag is set by the ¡§reset signal¡¨ from the RSTB pin to indicate the previous reset source.
#define SYS_RSTSRC_RSTS_WDT		BIT2					// The RSTS_WDT flag is set by the ¡§reset signal¡¨ from the Watch-Dog Timer module to indicate the previous reset source.
#define SYS_RSTSRC_RSTS_LVR		BIT3					// The RSTS_LVR flag is set by the ¡§reset signal¡¨ from the Low-Voltage-Reset module to indicate the previous reset source.
//#define SYS_RSTSRC_RSTS_MCU		BIT5					// The RSTS_MCU flag is set by the ¡§reset signal¡¨ from the MCU Cortex_M0 module to indicate the previous reset source.
#define SYS_RSTSRC_RSTS_PMU		BIT6					// The RSTS_PMU flag is set by the ¡§reset signal¡¨ from the PMU module to indicate the previous reset source.

// IPRSTC1(Reset Control Resister1)
#define SYS_IPRSTC1_CHIP_RST	BIT0					// CHIP one shut reset.
#define SYS_IPRSTC1_CPU_RST		BIT1					// CPU one shut reset.
#define SYS_IPRSTC1_CPUWS		BIT2					// CPU wait-state control for Flash memory access
#define SYS_IPRSTC1_RAMWS		BIT3					// CPU wait-state control for RAM access

// IPRSTC2(IP Reset Control Resister2)
#define SYS_IPRSTC2_GPIO_RST	BIT1					// GPIO controller Reset
#define SYS_IPRSTC2_TMR0_RST	BIT2					// Timer0 controller Reset
#define SYS_IPRSTC2_TMR1_RST	BIT3					// Timer1 controller Reset
#define SYS_IPRSTC2_TMR2_RST	BIT4					// Timer2 controller Reset
#define SYS_IPRSTC2_APU_RST		BIT5					// APU controller Reset
#define SYS_IPRSTC2_TMRF_RST	BIT6					// TimerF controller Reset
#define SYS_IPRSTC2_SPI0_RST	BIT12					// SPI0 controller Reset
#define SYS_IPRSTC2_SPI1_RST	BIT13					// SPI1 controller Reset
#define SYS_IPRSTC2_PWM_RST		BIT20					// PWM controller Reset
//#define SYS_IPRSTC2_USBD_RST	BIT27					// USB Device Controller Reset
#define SYS_IPRSTC2_ADC_RST		BIT28					// ADC Controller Reset

// VDCR(Voltage Detector Control Register)
#define SYS_VDCR_VD_EN			BIT0					// Voltage Detector Enable (initiate & protected bit)
#define SYS_VDCR_VD_VL			BIT1					// Voltage Detector Threshold Voltage Selection (initiate & protected bit)
#define SYS_VDCR_VD_OUT			BIT6					// The status for Voltage Detector output state
#define SYS_VDCR_LVR_EN			BIT7					// Low Voltage Reset Enable (protected bit)

// PORCR(Power-On-Reset Controller Register)
#define SYS_PORCR_POR_DIS_CODE	BITS(15,0)				// The register is used for the Power-On-Reset enable control. 

// GPA_MFP(GPIOA multiple function and input type control register)
#define SYS_GPA_MFP_GPA_MFS0	BIT0					// 1 = SPI0 2nd chip select output
														// 0 = The GPIOA-0 is selected to the pin GPA0
#define SYS_GPA_MFP_GPA_MFS1	BIT1					// 1 = SPI0 1st chip select output
														// 0 = The GPIOA-1 is selected to the pin GPA1
#define SYS_GPA_MFP_GPA_MFS2	BIT2					// 1 = SPI0 clock output
														// 0 = The GPIOA-2 is selected to the pin GPA2
#define SYS_GPA_MFP_GPA_MFS3	BIT3					// 1 = SPI0 data input
														// 0 = The GPIOA-3 is selected to the pin GPA3
#define SYS_GPA_MFP_GPA_MFS4	BIT4					// 1 = SPI0 data output
														// 0 = The GPIOA-4 is selected to the pin GPA4
#define SYS_GPA_MFP_GPA_MFS5	BIT5					// 1 = Timer0 counter external input
														// 0 = The GPIOA-5 is selected to the pin GPA5
#define SYS_GPA_MFP_GPA_MFS6	BIT6					// 1 = External interrupt input
														// 0 = The GPIOA-6 is selected to the pin GPA6
#define SYS_GPA_MFP_GPA_MFS7	BIT7					// 1 = ADC input external trigger input
														// 0 = The GPIOA-7 is selected to the pin GPA7
#define SYS_GPA_MFP_GPA_MFS8	BIT8					// 1 = ADC input channel 0
														// 0 = The GPIOA-8 is selected to the pin GPA8
#define SYS_GPA_MFP_GPA_MFS9	BIT9					// 1 = ADC input channel 1
														// 0 = The GPIOA-9 is selected to the pin GPA9
#define SYS_GPA_MFP_GPA_MFS10	BIT10					// 1 = ADC input channel 2
														// 0 = The GPIOA-10 is selected to the pin GPA10
#define SYS_GPA_MFP_GPA_MFS11	BIT11					// 1 = ADC input channel 3
														// 0 = The GPIOA-11 is selected to the pin GPA11
#define SYS_GPA_MFP_GPA_MFS12	BIT12					// 1 = ADC input channel 4
														// 0 = The GPIOA-12 is selected to the pin GPA12
#define SYS_GPA_MFP_GPA_MFS13	BIT13					// 1 = ADC input channel 5
														// 0 = The GPIOA-13 is selected to the pin GPA13
#define SYS_GPA_MFP_GPA_MFS14	BIT14					// 1 = ADC input channel 6
														// 0 = The GPIOA-14 is selected to the pin GPA14
#define SYS_GPA_MFP_GPA_MFS15	BIT15					// 1 = ADC input channel 7
														// 0 = The GPIOA-15 is selected to the pin GPA15
#define SYS_GPA_MFP_GPA_TYPEN	BITS(31,16)				// 1 = GPA[n] I/O cell input Schmitt Trigger function enable
														// 0 = GPA[n] I/O cell input Schmitt Trigger function disable 

// GPB_MFP(GPIOB multiple function and input type control register)
#define SYS_GPB_MFP_GPB_MFS0	BIT0					// 1 = SPI1 2nd chip select output
														// 0 = The GPIOB-0 is selected to the pin GPB0
#define SYS_GPB_MFP_GPB_MFS1	BIT1					// 1 = SPI1 1st chip select output
														// 0 = The GPIOB-1 is selected to the pin GPB1
#define SYS_GPB_MFP_GPB_MFS2	BIT2					// 1 = SPI1 clock output
														// 0 = The GPIOB-2 is selected to the pin GPB2
#define SYS_GPB_MFP_GPB_MFS3	BIT3					// 1 = SPI1 data input
														// 0 = The GPIOB-3 is selected to the pin GPB3
#define SYS_GPB_MFP_GPB_MFS4	BIT4					// 1 = SPI1 data output
														// 0 = The GPIOB-4 is selected to the pin GPB4
#define SYS_GPB_MFP_GPB_MFS8	BIT8					// 1 = PWM output pin 0
														// 0 = The GPIOB-8 is selected to the pin GPB8
#define SYS_GPB_MFP_GPB_MFS9	BIT9					// 1 = PWM output pin 1
														// 0 = The GPIOB-9 is selected to the pin GPB9
#define SYS_GPB_MFP_GPB_MFS10	BIT10					// 1 = PWM output pin 2
														// 0 = The GPIOB-10 is selected to the pin GPB10
#define SYS_GPB_MFP_GPB_MFS11	BIT11					// 1 = PWM output pin 3
														// 0 = The GPIOB-11 is selected to the pin GPB11
#define SYS_GPB_MFP_GPB_MFS12	BIT12					// 1 = PWM timer capture input
														// 0 = The GPIOB-12 is selected to the pin GPB12
#define SYS_GPB_MFP_GPB_MFS13	BIT13					// 1 = IR carrier output
														// 0 = The GPIOB-13 is selected to the pin GPB13
#define SYS_GPB_MFP_GPB_MFS14	BIT14					// 1 = Timer1 counter external input
														// 0 = The GPIOB-14 is selected to the pin GPB14
#define SYS_GPB_MFP_GPB_MFS15	BIT15					// 1 = Timer2 counter external input
														// 0 = The GPIOB-15 is selected to the pin GPB15
#define SYS_GPB_MFP_GPB_TYPEN	BITS(31,16)				// 1 = GPB[n] I/O cell input Schmitt Trigger function enable
														// 0 = GPB[n] I/O cell input Schmitt Trigger function disable 

// RegLockAddr(Register Lock Key address)
#define SYS_REGLOCKADDR_REGUNLOCK	BIT0				// 1 = Protected registers are Unlock
														// 0 = Protected registers are locked. Any write to the target register is ignored.

typedef	__I	uint32_t SYS_PDID_T;

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__IO uint32_t  RSTS_POR:1;					// The RSTS_POR flag is set by the ¡§reset signal¡¨ from the module POR to indicate the previous reset source.
		__IO uint32_t  RSTS_PAD:1;					// The RSTS_PAD flag is set by the ¡§reset signal¡¨ from the RSTB pin to indicate the previous reset source.
		__IO uint32_t  RSTS_WDT:1;					// The RSTS_WDT flag is set by the ¡§reset signal¡¨ from the Watch-Dog Timer module to indicate the previous reset source.
		__IO uint32_t  RSTS_LVR:1;					// The RSTS_LVR flag is set by the ¡§reset signal¡¨ from the Low-Voltage-Reset module to indicate the previous reset source.
		__I  uint32_t  RESERVE:2;
		//__IO uint32_t  RSTS_MCU:1;					// The RSTS_MCU flag is set by the ¡§reset signal¡¨ from the MCU Cortex_M0 module to indicate the previous reset source.
		__IO uint32_t  RSTS_PMU:1;					// The RSTS_PMU flag is set by the ¡§reset signal¡¨ from the PMU module to indicate the previous reset source.
	};
} SYS_RSTSRC_T;

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__IO uint32_t  CHIP_RST:1;					// CHIP one shut reset.
		__IO uint32_t  CPU_RST:1;					// CPU one shut reset.
		__IO uint32_t  CPUWS:1;						// CPU wait-state control for Flash memory access
		__IO uint32_t  RAMWS:1;						// CPU wait-state control for RAM access
	};
} SYS_IPRSTC1_T;

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__I	 uint32_t  RESERVE0:1;					// Reserved
		__IO uint32_t  GPIO_RST:1;					// GPIO controller Reset
		__IO uint32_t  TMR0_RST:1;					// Timer0 controller Reset
		__IO uint32_t  TMR1_RST:1;					// Timer1 controller Reset
		__IO uint32_t  TMR2_RST:1;					// Timer2 controller Reset
		__IO uint32_t  APU_RST:1;					// APU controller Reset
		__IO uint32_t  TMRF_RST:1;					// TimerF controller Reset
		__I	 uint32_t  RESERVE1:5;
		__IO uint32_t  SPI0_RST:1;					// SPI0 controller Reset
		__IO uint32_t  SPI1_RST:1;					// SPI1 controller Reset
		__I	 uint32_t  RESERVE3:6;
		__IO uint32_t  PWM_RST:1;					// PWM controller Reset
		__I	 uint32_t  RESERVE4:7;
		//__IO uint32_t  USBD_RST:1;					// USB Device Controller Reset
		__IO uint32_t  ADC_RST:1;					// ADC Controller Reset
	};
} SYS_IPRSTC2_T;

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__IO uint32_t  VD_EN:1;						// Voltage Detector Enable (initiate & protected bit)
		__IO uint32_t  VD_VL:1;						// Voltage Detector Threshold Voltage Selection (initiate & protected bit)
		__I  uint32_t  RESERVE:4;
		__I  uint32_t  VD_OUT:1;					// The status for Voltage Detector output state
		__IO uint32_t  LVR_EN:1;					// Low Voltage Reset Enable (protected bit)
	};
} SYS_VDCR_T;

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__I	 uint32_t  POR_DIS_CODE:16;					// The register is used for the Power-On-Reset enable control. 
	};
} SYS_PORCR_T;

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__IO uint32_t  GPA_MFS0:1;						// 1 = SPI0 2nd chip select output
														// 0 = The GPIOA-0 is selected to the pin GPA0
		__IO uint32_t  GPA_MFS1:1;						// 1 = SPI0 1st chip select output
														// 0 = The GPIOA-1 is selected to the pin GPA1
		__IO uint32_t  GPA_MFS2:1;						// 1 = SPI0 clock output
														// 0 = The GPIOA-2 is selected to the pin GPA2
		__IO uint32_t  GPA_MFS3:1;						// 1 = SPI0 data input
														// 0 = The GPIOA-3 is selected to the pin GPA3
		__IO uint32_t  GPA_MFS4:1;						// 1 = SPI0 data output
														// 0 = The GPIOA-4 is selected to the pin GPA4
		__IO uint32_t  GPA_MFS5:1;						// 1 = Timer0 counter external input
														// 0 = The GPIOA-5 is selected to the pin GPA5
		__IO uint32_t  GPA_MFS6:1;						// 1 = External interrupt input
														// 0 = The GPIOA-6 is selected to the pin GPA6
		__IO uint32_t  GPA_MFS7:1;						// 1 = ADC input external trigger input
														// 0 = The GPIOA-7 is selected to the pin GPA7
		__IO uint32_t  GPA_MFS8:1;						// 1 = ADC input channel 0
														// 0 = The GPIOA-8 is selected to the pin GPA8
		__IO uint32_t  GPA_MFS9:1;						// 1 = ADC input channel 1
														// 0 = The GPIOA-9 is selected to the pin GPA9
		__IO uint32_t  GPA_MFS10:1;						// 1 = ADC input channel 2
														// 0 = The GPIOA-10 is selected to the pin GPA10
		__IO uint32_t  GPA_MFS11:1;						// 1 = ADC input channel 3
														// 0 = The GPIOA-11 is selected to the pin GPA11
		__IO uint32_t  GPA_MFS12:1;						// 1 = ADC input channel 4
														// 0 = The GPIOA-12 is selected to the pin GPA12
		__IO uint32_t  GPA_MFS13:1;						// 1 = ADC input channel 5
														// 0 = The GPIOA-13 is selected to the pin GPA13
		__IO uint32_t  GPA_MFS14:1;						// 1 = ADC input channel 6
														// 0 = The GPIOA-14 is selected to the pin GPA14
		__IO uint32_t  GPA_MFS15:1;						// 1 = ADC input channel 7
														// 0 = The GPIOA-15 is selected to the pin GPA15
		__IO uint32_t  GPA_TYPEN:16;					// 1 = GPA[n] I/O cell input Schmitt Trigger function enable
														// 0 = GPA[n] I/O cell input Schmitt Trigger function disable 

	};
} SYS_GPAMFP_T;

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__IO uint32_t  GPB_MFS0:1;						// 1 = SPI1 2nd chip select output
														// 0 = The GPIOB-0 is selected to the pin GPB0
		__IO uint32_t  GPB_MFS1:1;						// 1 = SPI1 1st chip select output
														// 0 = The GPIOB-1 is selected to the pin GPB1
		__IO uint32_t  GPB_MFS2:1;						// 1 = SPI1 clock output
														// 0 = The GPIOB-2 is selected to the pin GPB2
		__IO uint32_t  GPB_MFS3:1;						// 1 = SPI1 data input
														// 0 = The GPIOB-3 is selected to the pin GPB3
		__IO uint32_t  GPB_MFS4:1;						// 1 = SPI1 data output
														// 0 = The GPIOB-4 is selected to the pin GPB4
		__I  uint32_t  RESERVE:3;

		__IO uint32_t  GPB_MFS8:1;						// 1 = PWM output pin 0
														// 0 = The GPIOB-8 is selected to the pin GPB8
		__IO uint32_t  GPB_MFS9:1;						// 1 = PWM output pin 1
														// 0 = The GPIOB-9 is selected to the pin GPB9
		__IO uint32_t  GPB_MFS10:1;						// 1 = PWM output pin 2
														// 0 = The GPIOB-10 is selected to the pin GPB10
		__IO uint32_t  GPB_MFS11:1;						// 1 = PWM output pin 3
														// 0 = The GPIOB-11 is selected to the pin GPB11
		__IO uint32_t  GPB_MFS12:1;						// 1 = PWM timer capture input
														// 0 = The GPIOB-12 is selected to the pin GPB12
		__IO uint32_t  GPB_MFS13:1;						// 1 = IR carrier output
														// 0 = The GPIOB-13 is selected to the pin GPB13
		__IO uint32_t  GPB_MFS14:1;						// 1 = Timer1 counter external input
														// 0 = The GPIOB-14 is selected to the pin GPB14
		__IO uint32_t  GPB_MFS15:1;						// 1 = Timer2 counter external input
														// 0 = The GPIOB-15 is selected to the pin GPB15
		__IO uint32_t  GPB_TYPEN:16;					// 1 = GPB[n] I/O cell input Schmitt Trigger function enable
														// 0 = GPB[n] I/O cell input Schmitt Trigger function disable 

	};
} SYS_GPBMFP_T;

typedef union
{
	__IO uint32_t  u32Reg;	
	struct
	{
		__IO uint32_t	GPA_HS:5;	// GPA_HS[n] =1: GPAn I/O high speed (up to 48MHz) transition enable.  (n=0~4)
														//           =0: GPAn I/O normal speed (less than 15MHz) transition. 
		__I  uint32_t	RESERVE:11;
		__IO uint32_t	GPRB:5;		// Five GPIOB general purpose R/W register bits.	
	};
} SYS_GPIOHS_T;

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__I  uint32_t  REGUNLOCK:1;						// 1 = Protected registers are Unlock
																					// 0 = Protected registers are locked. Any write to the target register is ignored.
	};
} SYS_REGLOCKADDR_T;

typedef union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__IO  uint32_t	FINEADJ:8;						//User can further fine tune the RC_48M frequency by writing an adequate value to FINEADJ if necessary.
	};
} SYS_RCFFA_T;

typedef union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__IO  uint32_t	PGCOUT_EN:1;					//Enable PGC output to GPA14.
	};
} SYS_PGCDO_T;

typedef	struct
{
	SYS_PDID_T			PDID;							// Part number defined by marketing dept.
	SYS_RSTSRC_T		RSTSRC;							  // RSTSRC(System Reset Source Register)
	SYS_IPRSTC1_T		IPRSTC1;						  // IPRSTC1(Reset Control Resister1)
	SYS_IPRSTC2_T		IPRSTC2;						  // IPRSTC2(IP Reset Control Resister2)
	__I	 uint32_t		RESERVE0[2];
	SYS_VDCR_T			VDCR;						    	// VDCR(Voltage Detector Control Register)
	SYS_PORCR_T			PORCR;							  // PORCR(Power-On-Reset Controller Register)
	__I	 uint32_t		RESERVE1[4];
	SYS_GPAMFP_T		GPA_MFP;						  // GPA_MFP(GPIOA multiple function and input type control register)
	SYS_GPBMFP_T		GPB_MFP;						  // GPB_MFP(GPIOB multiple function and input type control register)
	SYS_GPIOHS_T		GPIO_HS;							// GPA high speed transition control register
	__I	 uint32_t		RESERVE2[47];
	__I  uint32_t		IMGMAP0;						  // MAP0 Data Image Register
	__I  uint32_t		IMGMAP1;						  // MAP1 Data Image Register
	SYS_REGLOCKADDR_T	REGLOCKADDR;				// RegLockAddr(Register Lock Key address)
	__I	 uint32_t		RESERVE3[7];
	SYS_RCFFA_T			RCFFA;								// RC_48M frequency fine adjustment register
	__I	 uint32_t		RESERVE4[3];
	SYS_PGCDO_T     PGCDO;								// PGC direct output control register
} SYS_T;

#endif
