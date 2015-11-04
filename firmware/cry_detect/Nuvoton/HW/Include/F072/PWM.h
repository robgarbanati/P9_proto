/*---------------------------------------------------------------------------------------------------------*/
/*																										   */
/* Copyright (c) Nuvoton Technology	Corp. All rights reserved.											   */
/*																										   */
/*---------------------------------------------------------------------------------------------------------*/

#ifndef	__PWM_H__
#define	__PWM_H__

/*-----------------------------	PWM	Controller -------------------------------*/
// PPR(PWM Prescaler Register)
#define PWM_PPP_CP0					BIT(7,0)		// Clock prescaler for PWM Timer 
#define PWM_PPP_DZI0				BIT(23,16)		// Dead zone interval register 0
#define PWM_PPP_DZI1				BIT(31,24)		// Dead zone interval register 1

// CSR(PWM Clock Select Register)
#define PWM_CSR_CSR0				BIT(2,0)		// PWM Timer Clock Source Selection

// PCR(PWM Control Register)
#define PWM_PCR_PEN					BIT0			// PWM Timer Enable/Disable
#define PWM_PCR_PINV				BIT2			// PWM Timer Inverter ON/OFF
#define PWM_PCR_PMOD				BIT3			// PWM Timer Toggle/One-Shot Mode
#define PWM_PCR_DZEN0				BIT4			// Dead-Zone 0 Generator Enable/Disable
#define PWM_PCR_DZEN1				BIT5			// Dead-Zone 1 Generator Enable/Disable

// CNR(PWM Counter Register)
#define PWM_CNR_CNR					BIT(15,0)		// PWM Counter/Timer Loaded Value

// PDR0(PWM Data Register)
#define PWM_PDR_PDR					BIT(15,0)		// PWM Data Register

// CMRx(PWM Comparator Register 0~3)
#define PWM_CMR_CMR					BIT(15,0)		// PWM Comparator Register

// PIER(PWM Interrupt Enable Register)
#define PWM_PIER_PWMIE0				BIT0			// PWM Timer Interrupt Enable

// PIIR(PWM Interrupt Indication Register)
#define PWM_PIIR_PWMIF				BIT0			// PWM Timer Interrupt Flag

// CCR0(Capture Control Register)
#define PWM_CCR0_INV0				BIT0			// Inverter ON/OFF
#define PWM_CCR0_RL_IE0				BIT1			// Rising Interrupt Enable ON/OFF
#define PWM_CCR0_FL_IE0				BIT2			// Falling Interrupt Enable ON/OFF
#define PWM_CCR0_CAPEN				BIT3			// Capture input transition Enable/Disable
#define PWM_CCR0_CAPIF0				BIT4			// Capture Interrupt Indication Enable/Disable
#define PWM_CCR0_CRLRI0				BIT6			// CRLR0 Latched Indicator Bit
#define PWM_CCR0_CFLRI0				BIT7			// CFLR0 Latched Indicator Bit

// CRLR0(Capture Rising Latch Register)
#define PWM_CRLR_CRLR0				BIT(15,0)		// Capture Rising Latch Register

// CFLR0(Capture Falling Latch Register)
#define PWM_CFLR_CFLR0				BIT(15,0)		// Capture Falling Latch Register

// PCE(PWM Output and Capture Input Enable)
#define PWM_PCE_PWME0				BIT0			// PWM0 Output Enable Register
#define PWM_PCE_PWME1				BIT1			// PWM1 Output Enable Register
#define PWM_PCE_PWME2				BIT2			// PWM2 Output Enable Register
#define PWM_PCE_PWME3				BIT3			// PWM3 Output Enable Register
#define PWM_PCE_CAPIE0				BIT8			// Capture Input Enable Register

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__IO uint32_t  CP0:8;						// Clock prescaler for PWM Timer
		__I  uint32_t  RESERVE:8;
		__IO uint32_t  DZI0:8;						// Dead zone interval register 0
		__IO uint32_t  DZI1:8;						// Dead zone interval register 1
	};
} PWM_PPR_T;

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__IO uint32_t  CSR0:3;						// PWM Timer Clock Source Selection
	};
} PWM_CSR_T;

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__IO uint32_t  PEN:1;						// PWM Timer Enable/Disable
		__I  uint32_t  RESERVE:1;
		__IO uint32_t  PINV:1;						// PWM Timer Inverter ON/OFF
		__IO uint32_t  PMOD:1;						// PWM Timer Toggle/One-Shot Mode
		__IO uint32_t  DZEN0:1;						// Dead-Zone 0 Generator Enable/Disable
		__IO uint32_t  DZEN1:1;						// Dead-Zone 1 Generator Enable/Disable
	};	
} PWM_PCR_T;

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__IO uint32_t  CNR:16;						// PWM Counter/Timer Loaded Value
	};
} PWM_CNR_T;

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__IO uint32_t  CMR:16;						// PWM Comparator Register
	};
} PWM_CMR_T;

typedef	union
{
	__I  uint32_t  u32Reg;
	struct
	{
		__I  uint32_t  PDR:16;						// PWM Data Register
	};
} PWM_PDR_T;

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__IO uint32_t  PWMIE0:1;					// PWM Timer Interrupt Enable
	};
} PWM_PIER_T;

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__IO uint32_t  PWMIF:1;						// PWM Timer Interrupt Flag
	};
} PWM_PIIR_T;

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__IO uint32_t  INV0:1;						// Inverter ON/OFF
		__IO uint32_t  RL_IE0:1;					// Rising Interrupt Enable ON/OFF
		__IO uint32_t  FL_IE0:1;					// Falling Interrupt Enable ON/OFF
		__IO uint32_t  CAPEN:1;						// Capture input transition Enable/Disable
		__IO uint32_t  CAPIF0:1;					// Capture Interrupt Indication Enable/Disable
		__I  uint32_t  REVERSE:1;
		__IO uint32_t  CRLRI0:1;					// CRLR0 Latched Indicator Bit
		__IO uint32_t  CFLRI0:1;					// CFLR0 Latched Indicator Bit
	};
} PWM_CCR0_T;

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__IO uint32_t  CRLR0:16;					// Capture Rising Latch Register
	};
} PWM_CRLR_T;

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__IO uint32_t  CFLR0:16;					// Capture Falling Latch Register
	};
} PWM_CFLR_T;


typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__IO uint32_t  PWME0:1;						// PWM0 Output Enable Register
		__IO uint32_t  PWME1:1;						// PWM1 Output Enable Register
		__IO uint32_t  PWME2:1;						// PWM2 Output Enable Register
		__IO uint32_t  PWME3:1;						// PWM3 Output Enable Register
		__I  uint32_t  RESERVE:4;
		__IO uint32_t  CAPIE0:1;					// Capture Input Enable Register
	};
} PWM_PCE_T;

typedef	struct
{
	PWM_PPR_T		PPR;							// PPR(PWM Prescaler Register)
	PWM_CSR_T		CSR;							// CSR(PWM Clock Select Register)
	PWM_PCR_T		PCR;							// PCR(PWM Control Register)
	PWM_CNR_T		CNR0;							// CNR0(PWM Counter Register)
	PWM_CMR_T		CMR0;							// CMR0(PWM Comparator Register 0)
	PWM_PDR_T		PDR0;							// PDR0(PWM Data Register)
	__I uint32_t	RESERVE0;
	PWM_CMR_T		CMR1;							// CMR1(PWM Comparator Register 1)
	__I uint32_t	RESERVE1[2];
	PWM_CMR_T		CMR2;							// CMR2(PWM Comparator Register 2)
	__I uint32_t	RESERVE2[2];
	PWM_CMR_T		CMR3;							// CMR3(PWM Comparator Register 3)
	__I uint32_t	RESERVE3[2];
	PWM_PIER_T		PIER;							// PIER(PWM Interrupt Enable Register)
	PWM_PIIR_T		PIIR;							// PIIR(PWM Interrupt Indication Register)
	__I uint32_t	RESERVE4[2];
	PWM_CCR0_T		CCR0;							// CCR0(Capture Control Register)
	__I uint32_t	RESERVE5;
	PWM_CRLR_T		CRLR0;							// CRLR0(Capture Rising Latch Register)
	PWM_CFLR_T		CFLR0;							// CFLR0(Capture Falling Latch Register)
	__I uint32_t	RESERVE6[7];
	PWM_PCE_T		PCE;							// PCE(PWM Output and Capture Input Enable)	 
} PWM_T;

#endif
