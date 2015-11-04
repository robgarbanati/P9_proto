/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c)Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#if (defined(__N572F072__)||defined(__N572P072__))
/*---------------------------------------------------------------------------------------------------------*/
/* Include related header files                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#include "Driver/DrvCLK.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
// none

/*---------------------------------------------------------------------------------------------------------*/
/* Implement Functons                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
/*---------------------------------------------------------------------------------------------------------*/
// Get Timer driver version
/*---------------------------------------------------------------------------------------------------------*/
UINT32
DrvCLK_GetVersion(void)
{
	return DRVCLK_VERSION_NUM;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:     
//				DrvCLK_GetPLLOutput()
// Parameter:
//	            void
// Return:
//              PLL2 output clock
// Description:
//              Get timer clock of PLL2. 
//				Note: This API must called after DrvCLK_EnableOscillator_P that enabled 
//					  oscillator. The default output clock is 30MHz.
/*---------------------------------------------------------------------------------------------------------*/
UINT32
DrvCLK_GetPLLOutput(void)
{
  	UINT32 u32Clk = 0;
	
	u32Clk = 30000000+(CLK.PLLCON.PLL2_FRQ*6000000);
	return u32Clk;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:     
//				DrvCLK_GetHclk()
// Parameter:
//	            void
// Return:
//              Timer Clock
// Description:
//              Get timer clock of HCLK. 
//				Note: When power on, 48MHz RC is selected as HCLK clock source.Before clock switch,
//			          the related clock sources (pre-select and new-select) must be turned on.This 
//					  API must called after DrvCLK_EnableOscillator_P that enabled oscillator.
/*---------------------------------------------------------------------------------------------------------*/
UINT32 
DrvCLK_GetHclk(void)
{
	UINT32 u32Clk = 0;
	switch(CLK.CLKSEL0.HCLK_S)
	{
		case eDRVCLK_HCLKSRC_PLL1OUT:	// PLL1OUT
		{
			u32Clk = 6000000;
			break;
		}
		case eDRVCLK_HCLKSRC_PLL2OUT: // PLL2OUT
		{
			u32Clk = DrvCLK_GetPLLOutput();
			break;
		}
		case eDRVCLK_HCLKSRC_32K: // 32KHz
		{
			u32Clk = 32000;
			break;
		}
		default:					// RC_48MHz
		{
			u32Clk = 48000000;
			break;
		}
	}
	u32Clk = u32Clk / (CLK.CLKDIV.HCLK_N + 1);
	return u32Clk;
}
/*---------------------------------------------------------------------------------------------------------*/
// Function:     
//				DrvCLK_GetTimerClk()
// Parameter:
//	            u32ClkSel         	[in] Timer0/Timer1/Timer2 clock source   
// Return:
//              Timer Clock
// Description:
//              Get timer clock of Timer0/1/2.
//				Note: This API must be called after DrvCLK_IsEnabledApbDevice that enabled APB devices clock.
//					  The defalut value of timer clock is HCLK.	
/*---------------------------------------------------------------------------------------------------------*/
UINT32
DrvCLK_GetTimerClk(
	UINT8 u8TimerSrc)
{
	UINT32 u32Clk = 0;
	switch(u8TimerSrc)
	{
		case eDRVCLK_TIMERSRC_32K:	// 32KHz crystal clock
		{
			u32Clk = 32000;						
			break;
		}
		case eDRVCLK_TIMERSRC_TRIGGER:	// trigger
		{
			break;							
		}
		case eDRVCLK_TIMERSRC_48M: 		// 1xx : 48MHz oscillator clock
		{
			u32Clk = 48000000; 
			break;
		}
		default:	// HCLK. default value after reset
		{
			u32Clk = DrvCLK_GetHclk();
						
			break;
		}
	}
	return u32Clk;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:     
//				DrvCLK_GetClkPwm()
// Parameter:
//	            void
// Return:
//              Timer Clock
// Description:
//              Get timer clock of PWM timer.
//				Note: This API must be called after DrvCLK_IsEnabledApbDevice that enabled APB devices clock.
//					  The defalut value of timer clock is HCLK.
/*---------------------------------------------------------------------------------------------------------*/
UINT32 
DrvCLK_GetClkPwm(void)
{
	UINT32 u32Clk = 0;
	switch(CLK.CLKSEL1.PWM_S)
	{
		case eDRVCLK_PWMSRC_48M: // RC_48MHz.
		{
			u32Clk = 48000000;		
			break;
		}
		case eDRVCLK_PWMSRC_32K: // 32K
		{
			u32Clk = 32000;
			break;
		}
		default:  				 // HCLK. default value after reset      
		{
			u32Clk = DrvCLK_GetHclk();
			break;
		}
	}
	return u32Clk;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:     
//				DrvCLK_GetClkTmrf()
// Parameter:
//	            void
// Return:
//              Timer Clock
// Description:
//              Get timer clock of TimerF (fix frequency) timer.
//				Note: This API must be called after DrvCLK_IsEnabledApbDevice that enabled APB devices clock.
//					  The defalut value of timer clock is RC_48M/(4x65536).
/*---------------------------------------------------------------------------------------------------------*/
UINT32 
DrvCLK_GetClkTmrf(void)
{
	UINT32 u32Clk = 0;
	switch(CLK.CLKSEL1.TMRF_S)
	{
		case eDRVCLK_TIMERFSRC_32K_D32:       // 32K / 32 ==> 1000
		{
			u32Clk = 32000/32;
			break;
		}
		case eDRVCLK_TIMERFSRC_32K_D32D4: 	  // 32K / (4*32) ==> 250
		{
			u32Clk = 32000/(32*4);
			break;
		}
		case eDRVCLK_TIMERFSRC_48M_D65536:   // 48M / 65536 ==> 732.42..
		{
			u32Clk = 48000000/32768;
			break;
		}
		default: 							// 48M / (4*65536)==>183.10...
		{
			u32Clk = 48000000/(4*32768);
			break;
		}
		
	}
	return u32Clk;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:     
//				DrvCLK_GetClkSPI0()
// Parameter:
//	            void
// Return:
//              SPI0 Clock source
// Description:
//              Get timer clock of WDT timer
//				Note: This API must be called after DrvCLK_IsEnabledApbDevice that enabled APB devices clock.
//					  The defalut value of timer clock is HCLK.
/*---------------------------------------------------------------------------------------------------------*/
UINT32 
DrvCLK_GetClkSrcSPI0(void)
{
	UINT32 u32Clk = 0;
	switch(CLK.CLKSEL1.SPI0_S)
	{
		case eDRVCLK_SPI0SRC_PLL2OUT:	  // PLL2 output clcok
		{
			u32Clk = DrvCLK_GetPLLOutput();
			break;
		}
		case eDRVCLK_ADCSRC_48M:		 // RC_48MHz
		{
			u32Clk = 48000000;
			break;
		}
		default:					     // HCLK
		{
			u32Clk = DrvCLK_GetHclk();
			break;
		} 
	}
	return u32Clk;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:     
//				DrvCLK_GetSrcClkAdc()
// Parameter:
//	            void
// Return:
//              Timer Clock
// Description:
//              Get timer clock of ADC
//				Note: This API must be called after DrvCLK_IsEnabledApbDevice that enabled APB devices clock.
//					  The defalut value of timer clock is HCLK.
/*---------------------------------------------------------------------------------------------------------*/
UINT32 
DrvCLK_GetSrcClkAdc(void)
{
	UINT32 u32Clk = 0;
	switch(CLK.CLKSEL1.ADC_S)
	{
		case eDRVCLK_ADCSRC_PLL2OUT: // PLL2 output clcok
		{
			u32Clk = DrvCLK_GetPLLOutput();
			break;
		}
		case eDRVCLK_ADCSRC_48M:    // RC_48MHz
		{
			u32Clk = 48000000;
			break;
		}
		default:					// HCLK
		{
			u32Clk = DrvCLK_GetHclk();
			break;
		}
	}
	return u32Clk;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:     
//				DrvCLK_GetClkWdt()
// Parameter:
//	            void
// Return:
//              WDT timer Clock
// Description:
//              Get timer clock of WDT timer
//				Note: This API must be called after DrvCLK_IsEnabledApbDevice that enabled APB devices clock.
//					  The defalut value of timer clock is HCLK/2048.
/*---------------------------------------------------------------------------------------------------------*/
UINT32 
DrvCLK_GetClkWdt(void)
{
	UINT32 u32Clk = 0;
	switch(CLK.CLKSEL1.WDT_S)
	{
	
		case eDRVCLK_WDTSRC_32K: // 32K
		{
			u32Clk = 32000;
			break;
		}
		case eDRVCLK_WDTSRC_PLL1OUT: // PLL1 output clock fixed 6MHz
		{
			u32Clk = 6000000;
			break;
		}
		case eDRVCLK_WDTSRC_48MD2: // 24MHz
		{
			u32Clk = 48000000>>1;
			break;
		}
		default: 				// Hclk / 2048
		{
			u32Clk = DrvCLK_GetHclk() >> 11;
			break;
		}
	}
	return u32Clk;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:     
//				DrvCLK_EnableWakeUpInt
// Parameter:
//	            u8EnableWakeUpDelay	[in] 	1: enable 64 clock cycles delay to wait oscillator clock stable, 
//											0: disable 64 clock cycles delay to wait oscillator clock stable
// Return:
//              void
// Description:
//              Enable 'wake up interrupt' and 'delay wake up to wait for oscillator clock stable' 
/*---------------------------------------------------------------------------------------------------------*/
void
DrvCLK_EnableWakeUpInt(
	UINT8 u8EnableWakeUpDelay)
{
	//CLK.PWRCON.WU_DLY = u8EnableWakeUpDelay;
	//CLK.PWRCON.WINT_EN = 1;
	CLK.PWRCON.u32Reg = ((CLK.PWRCON.u32Reg&(~CLK_PWRCON_INTSTS))|CLK_PWRCON_WINT_EN|(u8EnableWakeUpDelay<<4));
	NVIC_EnableIRQ(PWRWU_IRQn);
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:     
//				DrvCLK_DisableWakeUpInt
// Parameter:
//	           void
// Return:
//              void
// Description:
//              Disable 'wake up interrupt' and 'delay wake up to wait for oscillator clock stable' 
/*---------------------------------------------------------------------------------------------------------*/
void
DrvCLK_DisableWakeUpInt(void)
{
	//CLK.PWRCON.WINT_EN = 0;
	CLK.PWRCON.u32Reg = CLK.PWRCON.u32Reg&(~(CLK_PWRCON_INTSTS|CLK_PWRCON_WINT_EN));
	NVIC_DisableIRQ(PWRWU_IRQn);
}


/*---------------------------------------------------------------------------------------------------------*/
#endif // (defined(__N572F072__)||defined(__N572P072__))
