
/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.  *
 *                                                              *
 ****************************************************************/

#ifndef __DRVCLK_H__
#define __DRVCLK_H__

// Include header file
#include "Platform.h"
#include "System/SysInfra.h"
#if (defined(__N572F064__)||defined(__N572F065__))
#include "Driver/DrvCLKEx_F065.h"
#elif (defined(__N572F072__)||defined(__N572P072__))
#include "Driver/DrvCLKEx_F072.h"
#endif

#ifdef  __cplusplus
extern "C"
{
#endif

// #define Constant
#define DRVCLK_MAJOR_NUM 		2
#define DRVCLK_MINOR_NUM 		00
#define DRVCLK_BUILD_NUM 		001
#define DRVCLK_VERSION_NUM	_SYSINFRA_VERSION(DRVCLK_MAJOR_NUM, DRVCLK_MINOR_NUM, DRVCLK_BUILD_NUM)
// APIs declaration

/*---------------------------------------------------------------------------------------------------------*/
// Get CLK driver version
/*---------------------------------------------------------------------------------------------------------*/
UINT32
DrvCLK_GetVersion(void);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvCLK_GetHclk()
//
// Return:
//              Source clock of HCLK.
//
// Description:
//              Get source clock of HCLK. 
//				Note: DrvCLK_SetClkSrcPLL() must be called first if 12M/6M is selected.
/*---------------------------------------------------------------------------------------------------------*/
UINT32 
DrvCLK_GetHclk(void);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvCLK_GetClkPwm()
//
// Return:
//              Source clock of PWM
//
// Description:
//              Get source clock of of PWM timer.
//				Note: DrvCLK_SetClkSrcPLL() must be called first if 12M/6M is selected.
/*---------------------------------------------------------------------------------------------------------*/
UINT32 
DrvCLK_GetClkPwm(void);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvCLK_GetClkTmrf()
//
// Return:
//              Source clock of timerF
//
// Description:
//              Get source clock of TimerF (fix frequency) timer.
//				Note: DrvCLK_SetClkSrcPLL() must be called first if 12M/6M is selected.
/*---------------------------------------------------------------------------------------------------------*/
UINT32 
DrvCLK_GetClkTmrf(void);

/*---------------------------------------------------------------------------------------------------------*/
// Function:     
//				DrvCLK_GetSrcClkAdc()
// Parameter:
//	            void
// Return:
//              Source clock of ADC
// Description:
//              Get source clock of ADC
//				Note: DrvCLK_SetClkSrcPLL() must be called first if 12M/6M is selected.
/*---------------------------------------------------------------------------------------------------------*/
UINT32 
DrvCLK_GetSrcClkAdc(void);

/*---------------------------------------------------------------------------------------------------------*/
// Function:     
//				DrvCLK_GetClkAdc()
// Parameter:
//	            void
// Return:
//              ADC clock frequency that is calculated by ADC divider.
// Description:
//              Get clock frequency of ADC
//				Note: DrvCLK_SetClkSrcPLL() must be called first if 12M/6M is selected.
/*---------------------------------------------------------------------------------------------------------*/
static __inline
UINT32 
DrvCLK_GetClkAdc(void)
{
	return (DrvCLK_GetSrcClkAdc() / (CLK.CLKDIV.ADC_N + 1));
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvCLK_GetClkWdt()
//
// Return:
//              Source clock of WDT 
//
// Description:
//              Get source clock of WDT timer
//				Note: DrvCLK_SetClkSrcPLL() must be called first if 12M/6M is selected.
/*---------------------------------------------------------------------------------------------------------*/
UINT32 
DrvCLK_GetClkWdt(void);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvCLK_EnableWakeUpInt
//
// Parameter:
//	            u8EnableWakeUpDelay	[in] 	1: enable 64 clock cycles delay to wait oscillator clock stable, 
//											0: disable 64 clock cycles delay to wait oscillator clock stable
//
// Description:
//              Enable 'wake up interrupt' and 'delay wake up to wait for oscillator clock stable' 
/*---------------------------------------------------------------------------------------------------------*/
void
DrvCLK_EnableWakeUpInt(
	UINT8 u8EnableWakeUpDelay
);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvCLK_DisableWakeUpInt
//
// Description:
//              Disable 'wake up interrupt' and 'delay wake up to wait for oscillator clock stable' 
/*---------------------------------------------------------------------------------------------------------*/
void
DrvCLK_DisableWakeUpInt(void);


// Define inline function

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvCLK_SetPowerDownEntryCondition_P
//
// Parameter:
//	            u8PowerDownEntryCondition	[in] 	1: Chip power down when both PWR_DOWN and CPU  
//														enter power down mode. 
//													0: Chip power down when PWR_DOWN bit is set.
//
// Description:
//              Set power down entry condition to "wait CPU" or not.
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void 
DrvCLK_SetPowerDownEntryCondition_P(
	UINT16 u16PowerDownEntryCondition)
{
	CLK.PWRCON.u32Reg = (CLK.PWRCON.u32Reg&(~(CLK_PWRCON_INTSTS)))|(u16PowerDownEntryCondition<<8);
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvCLK_PowerDown_P
//
// Parameter:
//	            u8Enable			[in] 	1: Enter power down mode (PS: wait CPU sleep event if it's enabled.)
//											0: Chip operation in normal mode.
//
// Description:
//              Enter power down mode or operation in normal mde.
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvCLK_PowerDown_P(
	UINT8 u8Enable)
{
	CLK.PWRCON.u32Reg = (CLK.PWRCON.u32Reg&(~(CLK_PWRCON_INTSTS)))|(u8Enable<<7);
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvCLK_GetWakeUpIntFlag
//
// Return:
//              The wake up interrupt status. 1: the chip resumed from idle mode, 0: normal
//
// Description:
//              Get wake up interrupt status.
/*---------------------------------------------------------------------------------------------------------*/
static __inline
UINT8
DrvCLK_GetWakeUpIntFlag(void)
{
	return (UINT8)CLK.PWRCON.INTSTS;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvCLK_ClearWakeUpIntFlag
//
// Description:
//              Clear wake up interrupt status.
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvCLK_ClearWakeUpIntFlag(void)
{
	CLK.PWRCON.INTSTS = 1;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvCLK_EnableLDO33_P
//
// Description:
//              Enable LDO33.
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvCLK_EnableLDO30_P(void)
{
	CLK.PWRCON.u32Reg = (CLK.PWRCON.u32Reg&(~(CLK_PWRCON_INTSTS)))&(~CLK_PWRCON_LDO30_EN);
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvCLK_DisableLDO33_P
//
// Description:
//              Disable LDO33.
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvCLK_DisableLDO30_P(void)
{
	CLK.PWRCON.u32Reg = (CLK.PWRCON.u32Reg&(~(CLK_PWRCON_INTSTS)))|(CLK_PWRCON_LDO30_EN);
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvCLK_EnableOscillator_P
//
// Parameter:
//	            eEnabledOsc			[in] 	bitwise or to enable oscillators. 
//											    Please refer to E_DRVCLK_OSCILLATOR
//
// Description:
//              Enable oscillators.
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvCLK_EnableOscillator_P(
	E_DRVCLK_OSCILLATOR eEnabledOsc)
{
	CLK.PWRCON.u32Reg = (CLK.PWRCON.u32Reg&(~(CLK_PWRCON_INTSTS)))|eEnabledOsc;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvCLK_DisableOscillator_P
//
// Parameter:
//	            eDisabledOsc		[in] 	bitwise or to disable oscillators. 
//											Please refer to E_DRVCLK_OSCILLATOR
//
// Description:
//              Disable oscillators. 
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvCLK_DisableOscillator_P(
	E_DRVCLK_OSCILLATOR eDisabledOsc)
{
	CLK.PWRCON.u32Reg = (CLK.PWRCON.u32Reg&(~(CLK_PWRCON_INTSTS)))&(~eDisabledOsc);
}



/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvCLK_SetClkSrcHclk_P()
//
// Parameter:
//	            eClkSrc			[in] clock source selection number
//									 Please refer to E_DRVCLK_HCLKSRC
//
// Description:
//              Set clock source of HCLK
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvCLK_SetClkSrcHclk_P(
	E_DRVCLK_HCLKSRC eClkSrc)
{
	CLK.CLKSEL0.HCLK_S = eClkSrc;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvCLK_SetClkSrcPwm()
//
// Parameter:
//	            eClkSrc			[in] clock source selection number
//									 Please refer to  E_DRVCLK_PWMSRC
//
// Description:
//              Set clock source of PWM
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvCLK_SetClkSrcPwm(
	E_DRVCLK_PWMSRC eClkSrc)
{
	CLK.CLKSEL1.PWM_S = eClkSrc;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvCLK_SetClkSrcTmrf()
//
// Parameter:
//	            eClkSrc			[in] clock source selection number
//									 Please refer to E_DRVCLK_TIMERFSRC
//
// Description:
//              Set clock source of TimerF
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvCLK_SetClkSrcTmrf(
	E_DRVCLK_TIMERFSRC eClkSrc)
{
	CLK.CLKSEL1.TMRF_S = eClkSrc;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvCLK_SetClkSrcTmr2()
//
// Parameter:
//	            eClkSrc			[in] clock source selection number
//									 Please refer to E_DRVCLK_TIMERSRC
// Description:
//              Set clock source of Timer2
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvCLK_SetClkSrcTmr2(
	E_DRVCLK_TIMERSRC eClkSrc)
{
	CLK.CLKSEL1.TMR2_S = eClkSrc;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvCLK_SetClkSrcTmr1()
//
// Parameter:
//	            eClkSrc			[in] clock source selection number
//									 Please refer to E_DRVCLK_TIMERSRC
// Description:
//              Set clock source of Timer1
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvCLK_SetClkSrcTmr1(
	E_DRVCLK_TIMERSRC eClkSrc)
{
	CLK.CLKSEL1.TMR1_S = eClkSrc;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvCLK_SetClkSrcTmr0()
//
// Parameter:
//	            eClkSrc			[in] clock source selection number
//                                   Please refer to E_DRVCLK_TIMERSRC
//
// Description:
//              Set clock source of Timer0
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvCLK_SetClkSrcTmr0(
	E_DRVCLK_TIMERSRC eClkSrc)
{
	CLK.CLKSEL1.TMR0_S = eClkSrc;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvCLK_SetClkSrcAdc()
//
// Parameter:
//	            eClkSrc			[in] clock source selection number
//									 Please refer to E_DRVCLK_ADCSRC
// Description:
//              Set clock source of ADC
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvCLK_SetClkSrcAdc(
	E_DRVCLK_ADCSRC eClkSrc)
{
	CLK.CLKSEL1.ADC_S = eClkSrc;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvCLK_SetClkSrcWdt()
//
// Parameter:
//	            eClkSrc			[in] clock source selection number
//								     Please refer to E_DRVCLK_WDTSRC
// Description:
//              Set clock source of WDT
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvCLK_SetClkSrcWdt(
	E_DRVCLK_WDTSRC eClkSrc)
{
	CLK.CLKSEL1.WDT_S = eClkSrc;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvCLK_SetClkDividerAdc()
//
// Parameter:
//	            u16ClkDiverder		[in] 12 bits clock divider 
//
// Description:
//              Set clock divider of ADC
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvCLK_SetClkDividerAdc(
	UINT16 u16ClkDiverder)
{
	CLK.CLKDIV.ADC_N = u16ClkDiverder;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvCLK_SetClkDividerHclk()
//
// Parameter:
//	            u8ClkDiverder		[in] Clock divider (0~15)
//
// Description:
//               Set clock divider of HCLK. The HCLK clock frequency = (HCLK clock source frequency) / (HCLK_N+1).
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvCLK_SetClkDividerHclk(
	UINT8 u8ClkDiverder)
{
	CLK.CLKDIV.HCLK_N = u8ClkDiverder;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvCLK_GetClkTmr0()
//
// Return:
//              Clock source of timer
//
// Description:
//              Get clock source of timer 
//				Note: DrvCLK_SetClkSrcPLL() must be called first if 12M/6M is selected.
/*---------------------------------------------------------------------------------------------------------*/
static __inline
UINT32 
DrvCLK_GetClkTmr0(void)
{
	return DrvCLK_GetTimerClk(CLK.CLKSEL1.TMR0_S);
}

/*---------------------------------------------------------------------------------------------------------*/
// refer to DrvCLK_GetClkTmr0
/*---------------------------------------------------------------------------------------------------------*/
static __inline
UINT32 
DrvCLK_GetClkTmr1(void)
{
	return DrvCLK_GetTimerClk(CLK.CLKSEL1.TMR1_S);
}
/*---------------------------------------------------------------------------------------------------------*/
// refer to DrvCLK_GetClkTmr0
/*---------------------------------------------------------------------------------------------------------*/
static __inline
UINT32 
DrvCLK_GetClkTmr2(void)
{
	return DrvCLK_GetTimerClk(CLK.CLKSEL1.TMR2_S);
}


/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvCLK_EnableApbDeviceClock()
//
// Parameter:
//	            u32EnableClk			[in] bitwise OR of IPs to be enabled.
//											DRVCLK_CLK_APU (N572F064)
//											DRVCLK_CLK_ADC
//											DRVCLK_CLK_USBD
//											DRVCLK_CLK_PWM
//											DRVCLK_CLK_SPI1
//											DRVCLK_CLK_SPI0
//											DRVCLK_CLK_TMRF
//											DRVCLK_CLK_TMR2
//											DRVCLK_CLK_TMR1
//											DRVCLK_CLK_TMR0
//											DRVCLK_CLK_RTC
//											DRVCLK_CLK_WDT
//
// Description:
//              Enable APB clock of IPs.
//Note:
//				On N572F072 chip, APU clock run on AHB bus. So, APU clock enabled bit must call by
//				DrvCLK_EnableAhbDeviceClock API.
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvCLK_EnableApbDeviceClock(
	UINT32 u32EnableClk)
{
	CLK.APBCLK.u32Reg |= u32EnableClk;
}
/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvCLK_DisableApbDeviceClock()
//
// Parameter:
//	            u32DisableClk		[in] bitwise OR of IPs to be disabled.
//											DRVCLK_CLK_APU (N572F064)
//											DRVCLK_CLK_ADC
//											DRVCLK_CLK_USBD
//											DRVCLK_CLK_PWM
//											DRVCLK_CLK_SPI1
//											DRVCLK_CLK_SPI0
//											DRVCLK_CLK_TMRF
//											DRVCLK_CLK_TMR2
//											DRVCLK_CLK_TMR1
//											DRVCLK_CLK_TMR0
//											DRVCLK_CLK_RTC
//											DRVCLK_CLK_WDT
//
// Description:
//              Disable APB clock of IPs.
// Note:
//			    On N572F072 chip, APU clock run on AHB bus. So, APU clock disabled bit must call by
//				DrvCLK_DisableAhbDeviceClock API.
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvCLK_DisableApbDeviceClock(
	UINT32 u32DisableClk)
{
	CLK.APBCLK.u32Reg &= ~u32DisableClk;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvCLK_EnableAhbDeviceClock()
//
// Parameter:
//	            u8EnableClk			[in] bitwise OR of IPs to be enabled.
//											DRVCLK_CLK_ISP
//											DRVCLK_CLK_APU (N572F072)
// Description:
//              Enable or disable ISP/APU clock.
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void 
DrvCLK_EnableAhbDeviceClock( 
	UINT8 u8EnableClk)
{
	CLK.AHBCLK.u32Reg |= u8EnableClk;
}
/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvCLK_DisableAhbDeviceClock()
//
// Parameter:
//	            uDisableClk			[in] bitwise OR of IPs to be disabled.
//											DRVCLK_CLK_ISP
//											DRVCLK_CLK_APU (N572F072)
// Description:
//              Enable or disable ISP/APU clock.
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void 
DrvCLK_DisableAhbDeviceClock( 
	UINT8 uDisableClk)
{
	CLK.AHBCLK.u32Reg &= ~uDisableClk;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvCLK_EnablePowerDownPll()
//
// Parameter:
//	            u8EnablePll			[in] bitwise OR of PLLs to be enabled.
//											DRVCLK_PLL1
//											DRVCLK_PLL2
//
// Description:
//              Enable PLL1 power down function
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void 
DrvCLK_EnablePowerDownPll(
	UINT32 u32EnablePll)
{
	CLK.PLLCON.u32Reg |= u32EnablePll;
}
/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvCLK_DisablePowerDownPll()
//
// Parameter:
//	            u8DisablePll			[in] bitwise OR of PLLs to be disabled.
//											DRVCLK_PLL1
//											DRVCLK_PLL2
//
// Description:
//              Disable PLL1 power down function
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void 
DrvCLK_DisablePowerDownPll(
	UINT32 u32DisablePll)
{
	CLK.PLLCON.u32Reg &= ~u32DisablePll;
}
/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvCLK_IsEnabledApbDevice()
//
// Parameter:
//	            u32APBDevices			[in] bitwise OR of IPs.
//											DRVCLK_CLK_APU (N572F064)
//											DRVCLK_CLK_ADC
//											DRVCLK_CLK_USBD
//											DRVCLK_CLK_PWM
//											DRVCLK_CLK_SPI1
//											DRVCLK_CLK_SPI0
//											DRVCLK_CLK_TMRF
//											DRVCLK_CLK_TMR2
//											DRVCLK_CLK_TMR1
//											DRVCLK_CLK_TMR0
//											DRVCLK_CLK_RTC
//											DRVCLK_CLK_WDT
// Return:     
//				bitwise OR of IPs.	
//
// Description:
//              Check whether APB IPs clock are enabled or not.
/*---------------------------------------------------------------------------------------------------------*/
static __inline
UINT32
DrvCLK_IsEnabledApbDevice(
	UINT32	 u32APBDevices
)
{
  return (CLK.APBCLK.u32Reg&u32APBDevices);
}
/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvCLK_IsEnabledAhbDevice()
//
// Parameter:
//	            u32AHBDevices			[in] bitwise OR of IPs.
//											DRVCLK_CLK_ISP
// Return:     							   	DRVCLK_CLK_APU (N572F072)
//				bitwise OR of IPs.	
//
// Description:
//              Check whether AhB IPs clock are enabled or not.
/*---------------------------------------------------------------------------------------------------------*/
static __inline
UINT32
DrvCLK_IsEnabledAhbDevice(
	UINT32	 u32AHBDevices
)
{
  return (CLK.AHBCLK.ISP_EN&u32AHBDevices);
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvCLK_GetClkSelTmr0()
//
// Return:
//              Timer clock selection
//
// Description:
//              Get timer clock selection 
/*---------------------------------------------------------------------------------------------------------*/
static __inline
E_DRVCLK_TIMERSRC 
DrvCLK_GetClkSelTmr0(void)
{
	return (E_DRVCLK_TIMERSRC)CLK.CLKSEL1.TMR0_S;
}
/*---------------------------------------------------------------------------------------------------------*/
// refer to DrvCLK_GetClkSelTmr0
/*---------------------------------------------------------------------------------------------------------*/
static __inline
E_DRVCLK_TIMERSRC 
DrvCLK_GetClkSelTmr1(void)
{
	return (E_DRVCLK_TIMERSRC)CLK.CLKSEL1.TMR1_S;
}
/*---------------------------------------------------------------------------------------------------------*/
// refer to DrvCLK_GetClkSelTmr0
/*---------------------------------------------------------------------------------------------------------*/
static __inline
E_DRVCLK_TIMERSRC 
DrvCLK_GetClkSelTmr2(void)
{
	return (E_DRVCLK_TIMERSRC)CLK.CLKSEL1.TMR2_S;
}

#ifdef  __cplusplus
}
#endif

#endif	// __DRVCLK_H__

