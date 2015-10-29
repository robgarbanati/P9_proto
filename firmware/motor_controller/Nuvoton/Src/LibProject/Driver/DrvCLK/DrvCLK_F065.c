/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/

#if (defined(__N572F064__)||defined(__N572F065__))
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
//				DrvCLK_GetHclk()
// Parameter:
//	            void
// Return:
//              Timer Clock
// Description:
//              Get timer clock of HCLK. 
//				Note: DrvCLK_SetClkSrcPLL2() must be called first if 12M/6M is selected.
/*---------------------------------------------------------------------------------------------------------*/
UINT32 
DrvCLK_GetHclk(void)
{
	UINT32 u32Clk = 0;
	switch(CLK.CLKSEL0.HCLK_S)
	{
		case eDRVCLK_HCLKSRC_12M_6M: // 12MHz/6MHz
		case eDRVCLK_HCLKSRC_PLL1OUT:	// PLL1OUT
		{
			if (CLK.PLLCON.PLL_SRC != eDRVCLK_PLLSRC_PLL1)
				u32Clk = (CLK.PLLCON.PLL_SRC == eDRVCLK_PLLSRC_12M)? 12000000 : 6000000;
			 else
				u32Clk = 6000000;
			break;
		}
		case eDRVCLK_HCLKSRC_24M: // 24MHz
		{
			u32Clk = 24000000;
			break;
		}
		case eDRVCLK_HCLKSRC_PLL2OUT:					// PLL2OUT
		{
			u32Clk = 48000000;
			break;
		}
		case eDRVCLK_HCLKSRC_32K: // 32KHz
		default:
		{
			u32Clk = 32000;
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
//	            u32ClkSel         	[in] Timer clock source selection
// Return:
//              Timer Clock
// Description:
//              Get timer clock of Timer0/1/2.
//				Note: DrvCLK_SetClkSrcPLL2() must be called first if 12M/6M is selected.
/*---------------------------------------------------------------------------------------------------------*/
UINT32
DrvCLK_GetTimerClk(
	E_DRVCLK_TIMERSRC eClkSel)
{
	UINT32 u32Clk = 0;
	switch(eClkSel)
	{
		case eDRVCLK_TIMERSRC_HCLK:	// HCLK. default value after reset
		{
			u32Clk = DrvCLK_GetHclk();
						
			break;
		}
		case eDRVCLK_TIMERSRC_32K:	// 32KHz crystal clock
		{
			u32Clk = 32000;						
			break;
		}
		case eDRVCLK_TIMERSRC_12M_6M:	// 12M/6M crystal clock
		{
			u32Clk = (CLK.PLLCON.PLL_SRC == eDRVCLK_PLLSRC_12M)? 12000000 : 6000000;
			break;
		}
		case eDRVCLK_TIMERSRC_TRIGGER:	// trigger
		{
			break;							
		}
		case eDRVCLK_TIMERSRC_24M:
		default: 					 	// 1xx : 24MHz oscillator clock
		{
			u32Clk = 24000000; 
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
//				Note: DrvCLK_SetClkSrcPLL2() must be called first if 12M/6M is selected.
/*---------------------------------------------------------------------------------------------------------*/
UINT32 
DrvCLK_GetClkPwm(void)
{
	UINT32 u32Clk = 0;
	switch(CLK.CLKSEL1.PWM_S)
	{
		case eDRVCLK_PWMSRC_24M:// 24MHz 
		{		
			u32Clk = 24000000;
			break;
		}
		case eDRVCLK_PWMSRC_32K: // 32K
		{
			u32Clk = 32000;
			break;
		}
		case eDRVCLK_PWMSRC_12M_6M: // (12M or 6M) 
		{
			u32Clk = (CLK.PLLCON.PLL_SRC == eDRVCLK_PLLSRC_12M)? 12000000 : 6000000;
			break;
		}
		case eDRVCLK_PWMSRC_HCLK: // HCLK. default value after reset      
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
//				Note: DrvCLK_SetClkSrcPLL2() must be called first if 12M/6M is selected.
/*---------------------------------------------------------------------------------------------------------*/
UINT32 
DrvCLK_GetClkTmrf(void)
{
	UINT32 u32Clk = 0;
	switch(CLK.CLKSEL1.TMRF_S)
	{
		case eDRVCLK_TIMERFSRC_32K_D32: // 32K / 32 ==> 1000
		{
			u32Clk = 32000/32;
			break;
		}
		case eDRVCLK_TIMERFSRC_12M_6M_D16384: // (12M or 6M) / 16384 ==> (732.42... 0r 366.21...)
		{
			u32Clk = (CLK.PLLCON.PLL_SRC == eDRVCLK_PLLSRC_12M)? 12000000/16384 : 6000000/16384;
			break;
		}
		case eDRVCLK_TIMERFSRC_12M_6M_D16384D4: // (12M or 6M) / (16384*4) ==> (183.10... or 91.55..)
		{
			u32Clk = (CLK.PLLCON.PLL_SRC == eDRVCLK_PLLSRC_12M)? 12000000/(4*16384) : 6000000/(4*16384);
			break;
		}
		case eDRVCLK_TIMERFSRC_24M_D32768: // 24M / 32768 ==> 732.42..
		{
			u32Clk = 24000000/32768;
			break;
		}
		case eDRVCLK_TIMERFSRC_24M_D32768D4: // 24M / (4*32768)==>183.10...
		{
			u32Clk = 24000000/(4*32768);
			break;
		}
		case eDRVCLK_TIMERFSRC_32K_D32D4: 	  // 32K / (4*32) ==> 250
		default:
		{
			u32Clk = 32000/(32*4);
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
//				Note: DrvCLK_SetClkSrcPLL2() must be called first if 12M/6M is selected.
/*---------------------------------------------------------------------------------------------------------*/
UINT32 
DrvCLK_GetSrcClkAdc(void)
{
	UINT32 u32Clk = 0;
	switch(CLK.CLKSEL1.ADC_S)
	{
		case eDRVCLK_ADCSRC_PLL2OUT: // PLL2
		{
			u32Clk = 48000000;
			break;
		}
		case eDRVCLK_ADCSRC_12M_6M: // (12M or 6M)
		{
			u32Clk = (CLK.PLLCON.PLL_SRC == eDRVCLK_PLLSRC_12M)? 12000000 : 6000000;
			break;
		}
		case eDRVCLK_ADCSRC_24M: // 24MHz
		default: // 24MHz
		{
			u32Clk = 24000000;
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
//				Note: DrvCLK_SetClkSrcPLL2() must be called first if 12M/6M is selected.
/*---------------------------------------------------------------------------------------------------------*/
UINT32 
DrvCLK_GetClkWdt(void)
{
	UINT32 u32Clk = 0;
	switch(CLK.CLKSEL1.WDT_S)
	{
		case eDRVCLK_WDTSRC_HCLK_D2048: // Hclk / 2048
		{
			u32Clk = DrvCLK_GetHclk() >> 11;
			break;
		}
		case eDRVCLK_WDTSRC_32K: // 32K
		{
			u32Clk = 32000;
			break;
		}
		case eDRVCLK_WDTSRC_12M_6M: // (12M or 6M)
		{
			u32Clk = (CLK.PLLCON.PLL_SRC == eDRVCLK_PLLSRC_12M)? 12000000 : 6000000;
			break;
		}
		case eDRVCLK_WDTSRC_24M: // 24MHz
		{
			u32Clk = 24000000;
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
#endif	// (defined(__N572F064__)||defined(__N572F065__))
