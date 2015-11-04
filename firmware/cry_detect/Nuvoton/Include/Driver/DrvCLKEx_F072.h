/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.  *
 *                                                              *
 ****************************************************************/

#ifndef __DRVCLKEX_H__
#define __DRVCLKEX_H__

// Enable oscillator
typedef enum{
	 eDRVCLK_XTL32K = 2,
	 eDRVCLK_OSC48M = 4	 	
} E_DRVCLK_OSCILLATOR;

// PLL2 clock source selection. PLL1 clock source is dedicated from 32K
// PLL2 input clock is fixed 6MHz.
typedef enum{
	 eDRVCLK_PLLSRC_48MD8 = 0,	// PLL2IN = 48MHz/8 = 6Mhz, PLL1OUT = 6MHz.
     eDRVCLK_PLLSRC_PLL1 = 1,	// PLL2IN = PLL1OUT, PLL1OUT = 6MHz.
} E_DRVCLK_PLLSRC;

// PLL2 clcok output selection
typedef enum{
	 eDRVCLK_PLLOUT_30M = 0,
	 eDRVCLK_PLLOUT_36M = 1,
	 eDRVCLK_PLLOUT_42M = 2,
	 eDRVCLK_PLLOUT_48M = 3
} E_DRVCLK_PLLOUTPUT;

// HCLK clock source selection
typedef enum{
	 eDRVCLK_HCLKSRC_32K = 0,
	 eDRVCLK_HCLKSRC_PLL1OUT = 2,
	 eDRVCLK_HCLKSRC_PLL2OUT = 4,
	 eDRVCLK_HCLKSRC_48M = 7
} E_DRVCLK_HCLKSRC;

// Timer0/1/2 clock source selection
typedef enum{
	 eDRVCLK_TIMERSRC_HCLK = 0,
     eDRVCLK_TIMERSRC_32K = 1,
	 eDRVCLK_TIMERSRC_TRIGGER = 3,
	 eDRVCLK_TIMERSRC_48M = 4
} E_DRVCLK_TIMERSRC;

// TimerF clock source selection. PS: 'D4, D32' means 'divide 4, divide 32'
typedef enum{
	  eDRVCLK_TIMERFSRC_32K_D32 = 0,
	  eDRVCLK_TIMERFSRC_32K_D32D4 = 1,
	  eDRVCLK_TIMERFSRC_48M_D65536 = 6,
	  eDRVCLK_TIMERFSRC_48M_D65536D4 = 7,
} E_DRVCLK_TIMERFSRC;

// PWM clock source selection
typedef enum{
	  eDRVCLK_PWMSRC_HCLK = 0,
      eDRVCLK_PWMSRC_32K = 1,
	  eDRVCLK_PWMSRC_48M = 3
} E_DRVCLK_PWMSRC;

// SPI0 clock source selection
typedef enum{
	  eDRVCLK_SPI0SRC_HCLK = 0,
      eDRVCLK_SPI0SRC_PLL2OUT = 1,
	  eDRVCLK_SPI0SRC_48M =2	   
} E_DRVCLK_SPI0SRC;

// ADC clock source selection
typedef enum{
	   eDRVCLK_ADCSRC_PLL2OUT = 0,
       eDRVCLK_ADCSRC_HCLK = 1,
	   eDRVCLK_ADCSRC_48M = 2
} E_DRVCLK_ADCSRC;

// WDT clock source selection. PS: 'D2048' means 'divide 2048'
typedef enum{
	   eDRVCLK_WDTSRC_HCLK_D2048 = 0,
       eDRVCLK_WDTSRC_32K = 1,
	   eDRVCLK_WDTSRC_PLL1OUT = 2,
	   eDRVCLK_WDTSRC_48MD2 = 3,      
} E_DRVCLK_WDTSRC;

extern UINT32 DrvCLK_GetTimerClk(UINT8 u8TimerSrc);

// Enable/disable IP clock
//APB
#define DRVCLK_CLK_ADC					CLK_APBCLK_ADC_EN
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
#define DRVCLK_CLK_APU					CLK_AHBCLK_APU_EN
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
	CLK.PLLCON.PLL2_SRC = eClkSrc;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:     
//				DrvCLK_SetPLLOutput()
// Parameter:
//	            PLL2 output clock selection
// Return:
//              void
// Description:
//              set clock output of PLL2. 
//				Note: This API must called after DrvCLK_EnableOscillator_P that enabled 
//					  oscillator.
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvCLK_SetPLLOutput(
	E_DRVCLK_PLLOUTPUT eOutputClk)
{
	CLK.PLLCON.PLL2_FRQ = eOutputClk;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:     
//				DrvCLK_GetPLLOutput()
// Parameter:
//	            void
// Return:
//              PLL2 output clock
// Description:
//              Get clock of PLL2. 
//				Note: This API must called after DrvCLK_EnableOscillator_P that enabled 
//					  oscillator. The default output clock is 30MHz.
/*---------------------------------------------------------------------------------------------------------*/
UINT32
DrvCLK_GetPLLOutput(void);

/*---------------------------------------------------------------------------------------------------------*/
// Function:     
//				DrvCLK_SetClkSrcSPI0()
// Parameter:
//	            void
// Return:
//              Set SPI0 Clock source
// Description:
//              Set clock source of SPI0
//				Note: This API must be called after DrvCLK_IsEnabledApbDevice that enabled APB devices clock.
//					  The defalut value of clock is HCLK.
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvCLK_SetClkSrcSPI0(
	E_DRVCLK_SPI0SRC eClkSrc
)
{
	CLK.CLKSEL1.SPI0_S = eClkSrc;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:     
//				DrvCLK_GetClkSrcSPI0()
// Parameter:
//	            void
// Return:
//              Get SPI0 Clock source
// Description:
//              Get clock source of SPI0
//				Note: This API must be called after DrvCLK_IsEnabledApbDevice that enabled APB devices clock.
//					  The defalut value of clock is HCLK.
/*---------------------------------------------------------------------------------------------------------*/
UINT32 
DrvCLK_GetClkSrcSPI0(void);


#endif
