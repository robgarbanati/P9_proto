/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright (c) Nuvoton Technology Corp. All rights reserved.                                             */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/

#ifndef __CLK_H__
#define __CLK_H__

/*---------------------------- Clock Controller ------------------------------*/
// PWRCON	(System Power Down Control Register)
#define CLK_PWRCON_XTL32K_EN		BIT1				// External 32.768 KHz Crystal Control
#define CLK_PWRCON_OSC48M_EN		BIT2				// Internal 48MHz Oscillator Control
#define CLK_PWRCON_LDO30_EN			BIT3				// Driving out 3.0V (through LDO30E pad) LDO Control
#define CLK_PWRCON_WU_DLY			BIT4				// Enable the wake up delay counter. When the chip wakes up from idle mode, the clock control will delay the 64 clock cycles to wait the 12M (or 6MHz) crystal or the internal 24MHz oscillator clock stable. 
#define CLK_PWRCON_WINT_EN			BIT5				// Wake Up Interrupt Enable
#define CLK_PWRCON_INTSTS			BIT6				// Chip wake up interrupt status
#define CLK_PWRCON_PWR_DOWN			BIT7				// System power down active or enable bit
#define CLK_PWRCON_PD_WAIT_CPU		BIT8				// This bit control the power down entry condition

// AHBCLK	(AHB Devices Clock Enable Control Register)
#define CLK_AHBCLK_ISP_EN			BIT2				// Flash ISP Controller Clock Enable Control
#define CLK_AHBCLK_APU_EN			BIT3				// APU Clock Enable Control.

// APBCLK	(APB Devices Clock Enable Control Register)
#define CLK_APBCLK_WDT_EN			BIT0				// Watch Dog Clock Enable.
#define CLK_APBCLK_RTC_EN			BIT1				// Real-Time-Clock APB interface Clock Control.
#define CLK_APBCLK_TMR0_EN			BIT2				// Timer0 Clock Enable Control
#define CLK_APBCLK_TMR1_EN			BIT3				// Timer1 Clock Enable Control
#define CLK_APBCLK_TMR2_EN			BIT4				// Timer2 Clock Enable Control
#define CLK_APBCLK_TMRF_EN			BIT5				// TimerF Clock Enable Control
#define CLK_APBCLK_SPI0_EN			BIT12				// SPI0 Clock Enable Control. 
#define CLK_APBCLK_SPI1_EN			BIT13				// SPI1 Clock Enable Control.
#define CLK_APBCLK_PWM_EN			BIT20				// PWM Timer Clock Enable Control.
#define CLK_APBCLK_ADC_EN			BIT28				// Analog-Digital-Converter (ADC) Clock Enable Control.

// CLKSEL0	(Clock Source Select Control Register 0)
#define CLK_CLKSEL0_HCLK_S			BIT(2,0)			// HCLK clock source select.
#define CLK_CLKSEL0_STCLK_S			BIT(4,3)			// Cortex_M0 SysTick clock source select.

// CLKSEL1	(Clock Source Select Control Register 1)
#define CLK_CLKSEL1_WDT_S		BIT(1,0)				// WDT clock source select.
#define CLK_CLKSEL1_ADC_S		BIT(3,2)				// ADC clock source select.
#define CLK_CLKSEL1_SPI0_S	BIT(5,4)				// SPI0 clock source select.
#define CLK_CLKSEL1_TMR0_S		BIT(10,8)			// Timer0 clock source select.
#define CLK_CLKSEL1_TMR1_S		BIT(14,12)		// Timer1 clock source select.
#define CLK_CLKSEL1_TMR2_S		BIT(18,16)		// Timer2 clock source select.
#define CLK_CLKSEL1_TMRF_S		BIT(22,20)		// TimerF clock source select. 
#define CLK_CLKSEL1_PWM_S		BIT(29,28)			// PWM Timer clock source select.

// CLKDIV	(Clock Divider Number Register)
#define CLK_CLKDIV_HCLK_N		BIT(3,0)				// HCLK clock divide number from HCLK clock source
#define CLK_CLKDIV_ADC_N		BIT(27,16)				// ADC engine clock divide number from ADC clock source

// PLLCON	(PLL Control Register)
#define CLK_PLLCON_PLL2PD		BIT0					// PLL2 Power Down Mode
#define CLK_PLLCON_PLL1PD		BIT1					// PLL1 Power Down Mode
#define CLK_PLLCON_PLL2_SRC		BIT2				// PLL2 Source Clock Select
#define CLK_PLLCON_PLL2_FRQ		BIT(4,3)		// PLL2 Clock Output Frequency Select.

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__I  uint32_t  RESERVE:1;
		__IO uint32_t  XTL32K_EN:1;						// External 32.768 KHz Crystal Control
		__IO uint32_t  OSC48M_EN:1;						// Internal 48MHz Oscillator Control
		__IO uint32_t  LDO30_EN:1;						// Driving out 3.0V (through LDO30E pad) LDO Control
		__IO uint32_t  WU_DLY:1;						// Enable the wake up delay counter. When the chip wakes up from idle mode, the clock control will delay the 64 clock cycles to wait the 12M (or 6MHz) crystal or the internal 24MHz oscillator clock stable. 
		__IO uint32_t  WINT_EN:1;						// Wake Up Interrupt Enable
		__IO uint32_t  INTSTS:1;						// Chip wake up interrupt status
		__IO uint32_t  PWR_DOWN:1;						// System power down active or enable bit
		__IO uint32_t  PD_WAIT_CPU:1;					// This bit control the power down entry condition
	};
} CLK_PWRCON_T;

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__I  uint32_t  RESERVE:2;
		__IO uint32_t  ISP_EN:1;						// Flash ISP Controller Clock Enable Control
		__IO uint32_t  APU_EN:1;						// APU Clock Enable Control.
	};
} CLK_AHBCLK_T;

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__IO uint32_t  WDT_EN:1;						// Watch Dog Clock Enable.
		__IO uint32_t  RTC_EN:1;						// Real-Time-Clock APB interface Clock Control.
		__IO uint32_t  TMR0_EN:1;						// Timer0 Clock Enable Control
		__IO uint32_t  TMR1_EN:1;						// Timer1 Clock Enable Control
		__IO uint32_t  TMR2_EN:1;						// Timer2 Clock Enable Control
		__IO uint32_t  TMRF_EN:1;						// TimerF Clock Enable Control
		__IO uint32_t  RESERVE0:6;
		__IO uint32_t  SPI0_EN:1;						// SPI0 Clock Enable Control. 
		__IO uint32_t  SPI1_EN:1;						// SPI1 Clock Enable Control.
		__IO uint32_t  RESERVE1:6;
		__IO uint32_t  PWM_EN:1;						// PWM Timer Clock Enable Control.
		__IO uint32_t  RESERVE2:7;
		__IO uint32_t  ADC_EN:1;						// Analog-Digital-Converter (ADC) Clock Enable Control.
		__IO uint32_t  RESERVE3:3;						// Reserved
	};
} CLK_APBCLK_T;

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__IO uint32_t  HCLK_S:3;						// HCLK clock source select.
		__IO uint32_t  STCLK_S:2;						// Cortex_M0 SysTick clock source select.
	};
} CLK_CLKSEL0_T;


typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__IO uint32_t  WDT_S:2;							// WDT clock source select.
		__IO uint32_t  ADC_S:2;							// ADC clock source select.
		__IO uint32_t  SPI0_S:2;						// SPI0 clock source select.
		__I  uint32_t  RESERVE0:2;
		__IO uint32_t  TMR0_S:3;						// Timer0 clock source select.
		__I  uint32_t  RESERVE2:1;
		__IO uint32_t  TMR1_S:3;						// Timer1 clock source select.
		__I  uint32_t  RESERVE3:1;
		__IO uint32_t  TMR2_S:3;						// Timer2 clock source select.
		__I  uint32_t  RESERVE4:1;
		__IO uint32_t  TMRF_S:3;						// TimerF clock source select. 
		__I  uint32_t  RESERVE5:5;
		__IO uint32_t  PWM_S:2;							// PWM Timer clock source select.
	};
} CLK_CLKSEL1_T;

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{			
		__IO uint32_t  HCLK_N:4;						// HCLK clock divide number from HCLK clock source
		__IO uint32_t  RESERVE:12;
		__IO uint32_t  ADC_N:12;						// ADC engine clock divide number from ADC clock source
	};
} CLK_CLKDIV_T;

typedef	union
{
	__IO uint32_t  u32Reg;
	struct
	{
		__IO uint32_t  PLL2PD:1;						// PLL2 Power Down Mode
		__IO uint32_t  PLL1PD:1;						// PLL1 Power Down Mode
		__IO uint32_t  PLL2_SRC:1;					// PLL2 Source Clock Select
		__IO uint32_t  PLL2_FRQ:2;					// PLL2 Clock Output Frequency Select.
	};
} CLK_PLLCON_T;

typedef struct
{
	CLK_PWRCON_T	PWRCON;								// System Power Down Control Register
	CLK_AHBCLK_T	AHBCLK;								// AHB Devices Clock Enable Control Register
	CLK_APBCLK_T	APBCLK;								// APB Devices Clock Enable Control Register
	__I  uint32_t	RESERVED0;
	CLK_CLKSEL0_T	CLKSEL0;							// Clock Source Select Control Register 0
	CLK_CLKSEL1_T	CLKSEL1;							// Clock Source Select Control Register 1
	CLK_CLKDIV_T	CLKDIV;								// Clock Divider Number Register
	__I  uint32_t	RESERVED1;
	CLK_PLLCON_T	PLLCON;								// PLL Control Register
} CLK_T;

#endif
