/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) Nuvoton Technology Corp. All rights reserved.                                              */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#include "Platform.h"
#include "Driver/DrvSYS.h"
#include "Driver/DrvCLK.h"

//---------------------------------------------------------------------------------------------------------
// Select the system frequency
#define __SYSTEM_CLK__		48	// Select system clock to be one of the followings
								//	48, 24, 16, 12, 8, 6, 4, 3:	for defined __HCLK_RC__ (F072 and P072)
								//  24, 12, 8, 6, 3:			for defined __HCLK_RC__ (F064 and F065)
								//	48, 24, 16, 12, 8, 6, 4, 3:	for defined __HCLK_PLL2_32K__=48 (F064, F065, F072 and P072)   
								//	48, 24, 16, 12, 8, 6, 4, 3:	for defined __HCLK_PLL2_RC__=48 (F072 and P072)	 
								//	42, 21, 14, 7, 6, 3:		for defined __HCLK_PLL2_RC__=42 or __HCLK_PLL2_32K__=42	(F072 and P072)
								//	36, 18, 12, 9, 6, 4, 3:		for defined __HCLK_PLL2_RC__=36 or __HCLK_PLL2_32K__=36	(F072 and P072)
								//	30, 15, 10, 6, 5, 3, 2:		for defined __HCLK_PLL2_RC__=30 or __HCLK_PLL2_32K__=30 (F072 and P072)

//---------------------------------------------------------------------------------------------------------
// Select one of the following options to select the clock source
#define __HCLK_RC__				// Select internal RC(24MHz or 48MHz) as clock source of HCLK
								// For F064 and F065, the RC is 24MHz
								// For F072 and P072, the RC is 48MHz

//#define __HCLK_PLL2_32K__	48	// Select external 32K oscillator as the clock source of PLL2
								// and PLL2(48MHz, 42MHz, 36MHz, 30Mhz) as the clock source of HCLK
								// The value for this option can be one of the followings:
								//	48: 				for F064, F065
								//	48, 42, 36, 30:		for F072, P072

//#define __HCLK_PLL2_RC__	48	// Select internal RC oscillator as the clock source of PLL2
								// and PLL2(48MHz, 42MHz, 36MHz, 30Mhz) as the clock source of HCLK
								// The value for this option can be one of the followings:
								//	48, 42, 36, 30:		for F072, P072
								// No support F064 and F065

//---------------------------------------------------------------------------------------------------------
// Implement inline Functons                                                                                      
//---------------------------------------------------------------------------------------------------------
#if (defined(__N572F064__)||defined(__N572F065__))
#define _PLL_STABLE_COUNT_		500000 		// about 100ms count on CPU run 24MHz to wait PLL stable
#else	// (defined(__N572_F064__)||defined(__N572_F065__))
#define _PLL_STABLE_COUNT_		500000/2	// about 100ms count on CPU run 12MHz(default when power up or reset) to wait PLL stable
#endif	// (defined(__N572_F064__)||defined(__N572_F065__))

static __inline
void SysClkConfig_NoramlWaitState()
{
	DrvSYS_SetRAMWaitState_P(DRVSYS_RAM_WAIT_STATE_0_HCLK);						// set SRAM  access with 0 wait state
#if (__SYSTEM_CLK__ > 24)
	DrvSYS_SetFlashMemoryWaitState_P(DRVSYS_FLASH_MEMORY_WAIT_STATE_1_HCLK);	// set flash access with 1 wait state
#else	// (__SYSTEM_CLK__ > 24)
	DrvSYS_SetFlashMemoryWaitState_P(DRVSYS_FLASH_MEMORY_WAIT_STATE_0_HCLK);	// set flash access with 0 wait state
#endif	// (__SYSTEM_CLK__ > 24)
}

static __inline
void SysClkConfig_UpdateFlashWaitState()
{
#if (__SYSTEM_CLK__ > 24)
	DrvSYS_SetRAMWaitState_P(DRVSYS_RAM_WAIT_STATE_1_HCLK);						// set SRAM  access with 1 wait state
#else	// (__SYSTEM_CLK__ > 24)
	DrvSYS_SetRAMWaitState_P(DRVSYS_RAM_WAIT_STATE_0_HCLK);						// set SRAM  access with 0 wait state
#endif	// (__SYSTEM_CLK__ > 24)
	DrvSYS_SetFlashMemoryWaitState_P(DRVSYS_FLASH_MEMORY_WAIT_STATE_1_HCLK);	// set flash access with 1 wait state
}

//-------------------------------------------------------------------------------------------------------------------------------------
#ifdef __HCLK_RC__
#define _SYSCLKCONFIG()			SysClkConfigRC()

static __inline
void SysClkConfigRC()
{
	//------------------------------------------------------------------
	// Procedure to configure the wait state of SDRAM&internal flash
	//------------------------------------------------------------------
	SysClkConfig_NoramlWaitState();
	
	//------------------------------------------------------------------
	// Procedures to configure the HCLK from internal RC
	//------------------------------------------------------------------

#if (defined(__N572F064__)||defined(__N572F065__))
	DrvCLK_SetClkDividerHclk((24/__SYSTEM_CLK__)-1);							// Set the divider of AHB clock source 
	DrvCLK_SetClkSrcHclk_P(eDRVCLK_HCLKSRC_24M);								// Select internal RC(24M) as the clock source of AHB
#else	// (defined(__N572F064__)||defined(__N572F065__))
	DrvCLK_SetClkDividerHclk((48/__SYSTEM_CLK__)-1);							// Set the divider of AHB clock source
	DrvCLK_SetClkSrcHclk_P(eDRVCLK_HCLKSRC_48M);								// Select internal RC(48M) as the clock source of AHB
#endif	// (defined(__N572F064__)||defined(__N572F065__))
	
	//------------------------------------------------------------------
	// Procedures to disable unnecessary oscillator
	//------------------------------------------------------------------
	DrvCLK_EnablePowerDownPll(DRVCLK_PLL1|DRVCLK_PLL2);							// Set PLL1&PLL2 into power down mode
	DrvCLK_DisableOscillator_P(eDRVCLK_XTL32K);									// Disable exteranl 32K oscillator
}

#endif // __HCLK_RC__
//-------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------
#ifdef __HCLK_PLL2_32K__
#define _SYSCLKCONFIG()		SysClkConfigPll2_32K()

static __inline
void SysClkConfigPll2_32K()
{
	UINT32 u32PllStableCount;

	//------------------------------------------------------------------
	// Procedure to configure the wait state of SDRAM
	//------------------------------------------------------------------
	SysClkConfig_NoramlWaitState();

	//------------------------------------------------------------------
	// Procedures to configue PLL1&PLL2
	//------------------------------------------------------------------
	DrvCLK_EnableOscillator_P(eDRVCLK_XTL32K);									// Enable external 32K oscillator
	DrvCLK_DisablePowerDownPll(DRVCLK_PLL1|DRVCLK_PLL2);						// Enable PLL1 & PLL2
	DrvCLK_SetClkSrcPLL(eDRVCLK_PLLSRC_PLL1);									// Select PLL1 as PLL2 input source
#if (defined(__N572F072__)||defined(__N572P072__))
	DrvCLK_SetPLLOutput((E_DRVCLK_PLLOUTPUT)((__HCLK_PLL2_32K__-30)/6));		// Select PLL2 to output 48 or 42 or 36 or 30 MHz
#endif	// (defined(__N572F072__)||defined(__N572P072__))
	for(u32PllStableCount=0;u32PllStableCount<_PLL_STABLE_COUNT_;u32PllStableCount++);	// wait for PLL stable

	//------------------------------------------------------------------
	// Procedures to configure the HCLK from PLL2
	//------------------------------------------------------------------
	DrvCLK_SetClkDividerHclk((__HCLK_PLL2_32K__/__SYSTEM_CLK__)?((__HCLK_PLL2_32K__/__SYSTEM_CLK__)-1):0);
																				// Set the divider of AHB clock source
	DrvCLK_SetClkSrcHclk_P(eDRVCLK_HCLKSRC_PLL2OUT);							// Select PLL2 output as the clock source of AHB
	
	//------------------------------------------------------------------
	// Procedures to disable unnecessary oscillator
	//------------------------------------------------------------------
	// Please not disable internal RC oscillator otherwise will cause system abnormal
}


#endif // __HCLK_PLL2_32K__
//-------------------------------------------------------------------------------------------------------------------------------------
 
//-------------------------------------------------------------------------------------------------------------------------------------
#ifdef __HCLK_PLL2_RC__
#define _SYSCLKCONFIG()		SysClkConfigPll2_RC()

static __inline
void SysClkConfigPll2_RC()
{
	UINT32 u32PllStableCount;

	//------------------------------------------------------------------
	// Procedure to configure the wait state of SDRAM
	//------------------------------------------------------------------
	SysClkConfig_NoramlWaitState();

	//------------------------------------------------------------------
	// Procedures to configue PLL1&PLL2
	//------------------------------------------------------------------
#if (defined(__N572F072__)||defined(__N572P072__))
	DrvCLK_EnableOscillator_P(eDRVCLK_OSC48M);									// Enable internal 48MHz RC oscillator
	DrvCLK_DisablePowerDownPll(DRVCLK_PLL2);									// Enable PLL2
	DrvCLK_SetClkSrcPLL(eDRVCLK_PLLSRC_48MD8);									// Select internal RC(48MHz)/8 as PLL2 input source
	DrvCLK_SetPLLOutput((E_DRVCLK_PLLOUTPUT)((__HCLK_PLL2_RC__-30)/6));		// Select PLL2 to output 48 or 42 or 36 or 30MHz
#endif 	// (defined(__N572F072__)||defined(__N572P072__)) 
	for(u32PllStableCount=0;u32PllStableCount<_PLL_STABLE_COUNT_;u32PllStableCount++);	// wait for PLL stable
	
	//------------------------------------------------------------------
	// Procedures to configure the HCLK from PLL2
	//------------------------------------------------------------------
	DrvCLK_SetClkDividerHclk((__HCLK_PLL2_RC__/__SYSTEM_CLK__)?((__HCLK_PLL2_RC__/__SYSTEM_CLK__)-1):0);
																				// Set the divider of AHB clock source
	DrvCLK_SetClkSrcHclk_P(eDRVCLK_HCLKSRC_PLL2OUT);							// Select PLL2 output as the clock source of AHB
	
	//------------------------------------------------------------------
	// Procedures to disable unnecessary oscillator
	//------------------------------------------------------------------
	DrvCLK_DisableOscillator_P(eDRVCLK_XTL32K);									// Disable external 32K oscillator
}

#endif // __HCLK_PLL2_RC__
//-------------------------------------------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------------------------------------------
// Configurztion Error Checking
#ifdef __HCLK_RC__
	#if (defined(__N572F064__)||defined(__N572F065__)) 
		#if (__SYSTEM_CLK__ > 24)
			#error "Can not configure F064 and F065 system clock over 24MHz RC!"
		#endif	// (__SYSTEM_CLK__ > 24)
		
		#define	__HCLK_SRC__	24
	#else
		#define __HCLK_SRC__	48
	#endif
#endif

#ifdef __HCLK_PLL2_32K__  
	#define __HCLK_SRC__		__HCLK_PLL2_32K__

	#if (defined(__N572F064__)||defined(__N572F065__))
		#if (__HCLK_PLL2_32K__ != 48)
			#error "Can not configure F064 and F065 PLL2 to ouput not 48MHz!"
		#endif	// (__HCLK_PLL2_32K__ != 48)
	#else	// (defined(__N572F064__)||defined(__N572F065__))
		#if ((__HCLK_PLL2_32K__!= 48)&&(__HCLK_PLL2_32K__!= 42)&&(__HCLK_PLL2_32K__!= 36)&&(__HCLK_PLL2_32K__!= 30))
			#error "Can not configure F072 and P072 PLL2 to ouput not one of 48, 42, 36, 30 MHz!"
		#endif	// ((__HCLK_PLL2_32K__!= 48)&&(__HCLK_PLL2_32K__!= 42)&&(__HCLK_PLL2_32K__!= 36)&&(__HCLK_PLL2_32K__!= 30))
	#endif	// (defined(__N572F064__)||defined(__N572F065__))
#endif

#ifdef __HCLK_PLL2_RC__
	#define __HCLK_SRC__		__HCLK_PLL2_RC__
		
	#if (defined(__N572F064__)||defined(__N572F065__))
		#error "Can not configure F064 and F065 PLL2 using 24MHz RC!"
	#else	// (defined(__N572F064__)||defined(__N572F065__))
		#if ((__HCLK_PLL2_RC__!= 48)&&(__HCLK_PLL2_RC__!= 42)&&(__HCLK_PLL2_RC__!= 36)&&(__HCLK_PLL2_RC__!= 30))
			#error "Can not configure F072 and P072 PLL2 to ouput not one of 48, 42, 36, 30 MHz!"
		#endif	// ((__HCLK_PLL2_RC__!= 48)&&(__HCLK_PLL2_RC__!= 42)&&(__HCLK_PLL2_RC__!= 36)&&(__HCLK_PLL2_RC__!= 30))
	#endif	// (defined(__N572F064__)||defined(__N572F065__))
#endif

#if (__HCLK_SRC__<__SYSTEM_CLK__)
	#error "Can not configure the system clock over selected HCLK source clock!"
#endif	// (__HCLK_SRC__<__SYSTEM_CLK__)

#if (__HCLK_SRC__>(__SYSTEM_CLK__*16))
	#error "Can not configure the system clock from selected HCLK source clock!"
#endif	// (__HCLK_SRC__>(__SYSTEM_CLK__*16))

#if (__HCLK_SRC__%__SYSTEM_CLK__)
	#warning "Can not configure the exactly system clock from selected HCLK source clock. Please check the system clock option and HCLK option!"
#endif	// (__HCLK_SRC__%__SYSTEM_CLK__)
 //-------------------------------------------------------------------------------------------------------------------------------------

