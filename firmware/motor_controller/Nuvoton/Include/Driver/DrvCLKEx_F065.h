/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.  *
 *                                                              *
 ****************************************************************/

#ifndef __DRVCLKEX_H__
#define __DRVCLKEX_H__

// Enable oscillator
typedef enum{
	 eDRVCLK_XTL12M6M = 1,
	 eDRVCLK_XTL32K = 2,
	 eDRVCLK_OSC24M = 4	 	
} E_DRVCLK_OSCILLATOR;

// PLL clock source selection. PS: PLL1 clock source is dedicated from 32K
typedef enum{
	 eDRVCLK_PLLSRC_6M = 0,		   // PLL2OUT = 6M*8 = 48MHz, PLL1OUT = 6MHz
     eDRVCLK_PLLSRC_PLL1 = 1,	   // PLL2OUT = PLL1*8 = 48MHz, PLL1OUT = PLL1 = 6MHz
     eDRVCLK_PLLSRC_12M = 2		   // PLL2OUT = 12M*4 = 48MHz, PLL1OUT = 12MHz
} E_DRVCLK_PLLSRC;

// HCLK clock source selection
typedef enum{
	 eDRVCLK_HCLKSRC_32K = 0,
	 eDRVCLK_HCLKSRC_12M_6M = 1,
	 eDRVCLK_HCLKSRC_PLL1OUT = 2,
	 eDRVCLK_HCLKSRC_PLL2OUT = 4,
	 eDRVCLK_HCLKSRC_24M = 7
} E_DRVCLK_HCLKSRC;

// Timer0/1/2 clock source selection
typedef enum{
	 eDRVCLK_TIMERSRC_HCLK = 0,
     eDRVCLK_TIMERSRC_32K = 1,
	 eDRVCLK_TIMERSRC_12M_6M = 2,
	 eDRVCLK_TIMERSRC_TRIGGER = 3,
	 eDRVCLK_TIMERSRC_24M = 4
} E_DRVCLK_TIMERSRC;

// TimerF clock source selection. PS: 'D4, D32' means 'divide 4, divide 32'
typedef enum{
	  eDRVCLK_TIMERFSRC_32K_D32 = 0,
	  eDRVCLK_TIMERFSRC_32K_D32D4 = 1,
	  eDRVCLK_TIMERFSRC_12M_6M_D16384 = 2,
	  eDRVCLK_TIMERFSRC_12M_6M_D16384D4 = 3,
	  eDRVCLK_TIMERFSRC_24M_D32768 = 6,
	  eDRVCLK_TIMERFSRC_24M_D32768D4 = 7,
} E_DRVCLK_TIMERFSRC;

// PWM clock source selection
typedef enum{
	  eDRVCLK_PWMSRC_HCLK = 0,
      eDRVCLK_PWMSRC_32K = 1,
	  eDRVCLK_PWMSRC_12M_6M = 2,
	  eDRVCLK_PWMSRC_24M = 3
} E_DRVCLK_PWMSRC;

// ADC clock source selection
typedef enum{
	   eDRVCLK_ADCSRC_PLL2OUT = 0,
       eDRVCLK_ADCSRC_12M_6M = 1,
	   eDRVCLK_ADCSRC_24M = 2
} E_DRVCLK_ADCSRC;

// WDT clock source selection. PS: 'D2048' means 'divide 2048'
typedef enum{
	   eDRVCLK_WDTSRC_HCLK_D2048 = 0,
       eDRVCLK_WDTSRC_32K = 1,
	   eDRVCLK_WDTSRC_12M_6M = 2,
	   eDRVCLK_WDTSRC_24M = 3,      
} E_DRVCLK_WDTSRC;

extern UINT32 DrvCLK_GetTimerClk(UINT8 u8TimerSrc);

// Enable/disable IP clock
//APB
#define DRVCLK_CLK_ADC					CLK_APBCLK_ADC_EN
#define DRVCLK_CLK_APU					CLK_APBCLK_APU_EN
#define DRVCLK_CLK_USBD					CLK_APBCLK_USBD_EN
#define DRVCLK_CLK_PWM					CLK_APBCLK_PWM_EN
#define DRVCLK_CLK_SPI1					CLK_APBCLK_SPI1_EN
#define DRVCLK_CLK_SPI0					CLK_APBCLK_SPI0_EN
#define DRVCLK_CLK_TMRF					CLK_APBCLK_TMRF_EN
#define DRVCLK_CLK_TMR2					CLK_APBCLK_TMR2_EN
#define DRVCLK_CLK_TMR1					CLK_APBCLK_TMR1_EN
#define DRVCLK_CLK_TMR0					CLK_APBCLK_TMR0_EN
#define DRVCLK_CLK_RTC					CLK_APBCLK_RTC_EN
#define DRVCLK_CLK_WDT					CLK_APBCLK_WDT_EN
//AHB
#define DRVCLK_CLK_ISP					CLK_AHBCLK_ISP_EN

#define DRVCLK_PLL1						CLK_PLLCON_PLL1PD
#define DRVCLK_PLL2						CLK_PLLCON_PLL2PD

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvCLK_SetClkSrcPLL()
//
// Parameter:
//	            eClkSrc			  [in]   clock source selection number
//										 Please refer to  E_DRVCLK_PLLSRC
//
// Description:
//              Set clock source of PLL. Note: PLL1 clock source is always 32K osc.
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvCLK_SetClkSrcPLL(
	E_DRVCLK_PLLSRC eClkSrc)
{
	CLK.PLLCON.PLL_SRC = eClkSrc;
}



#endif
