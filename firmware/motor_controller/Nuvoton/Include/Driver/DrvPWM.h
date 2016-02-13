/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2010 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#ifndef __DRVPWM_H__
#define __DRVPWM_H__

#include "Platform.h"
#include "System/SysInfra.h"
#include "Driver/DrvCLK.h"
#include "Driver/DrvSYS.h"

//---------------------------------------------------------------------------------------------------------
//  Define Version number
//---------------------------------------------------------------------------------------------------------
#define DRVPWM_MAJOR_NUM	1
#define DRVPWM_MINOR_NUM	00
#define DRVPWM_BUILD_NUM	002

//---------------------------------------------------------------------------------------------------------
//  Version define with SysInfra
//---------------------------------------------------------------------------------------------------------
#define DRVPWM_VERSION_NUM     _SYSINFRA_VERSION(DRVPWM_MAJOR_NUM, DRVPWM_MINOR_NUM, DRVPWM_BUILD_NUM)

//---------------------------------------------------------------------------------------------------------
//  PWM Clcok Selector
//---------------------------------------------------------------------------------------------------------
typedef enum
{
	eDRVPWM_TIMER0	= 0,
	eDRVPWM_TIMER1	= 1,
	eDRVPWM_TIMER2	= 2,
	eDRVPWM_TIMER3	= 3,
	eDRVPWM_CAP			=0x10
} E_DRVPWM_TIMER;

typedef enum
{
	eDRVPWM_CLOCK_DIV_2 = 0,  // 1/2
	eDRVPWM_CLOCK_DIV_4,			// 1/4
	eDRVPWM_CLOCK_DIV_8,			// 1/8
	eDRVPWM_CLOCK_DIV_16,			// 1/16
	eDRVPWM_CLOCK_DIV_1				// 1
} E_DRVPWM_CLOCKDIV;

//---------------------------------------------------------------------------------------------------------
// Define PWM functions prototype
//---------------------------------------------------------------------------------------------------------
// Flow to start run a PWM output:
//		DrvPWM_Open()
//		DrvPWM_EnableDeadZone[0|1]()		// optional
//		DrvPWM_EnablePwmOutput0[0~3]()
//		DrvPWM_EnableCaptureInverter();		// optional for capture
//		DrvPWM_EnableCaptureRisingInt();	// optional for capture
//		DrvPWM_EnableCaptureFallingInt();	// optional for capture
//		DrvPWM_EnableCaptureInput();			// must for capture
//		DrvPWM_EnableCapture()						// must for capture
//		DrvPWM_StartPwmTimer()
//		DrvPWM_SetComparatorPwm[0~3]()
//		DrvPWM_EnablePwmInt();						// must for PWM
//---------------------------------------------------------------------------------------------------------
// Function:		DrvPWM_Open
// Description:
//							Enable PWM engine clock and reset PWM
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvPWM_Open()
{
	DrvCLK_EnableApbDeviceClock(DRVCLK_CLK_PWM);
	DrvSYS_ResetIP(DRVSYS_RST_PWM);
}

//---------------------------------------------------------------------------------------------------------
// Function:		DrvPWM_Close
// Description:
// 							Disable PWM engine clock and the I/O enable
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvPWM_Close()
{
	DrvSYS_ResetIP(DRVSYS_RST_PWM);
	DrvCLK_DisableApbDeviceClock(DRVCLK_CLK_PWM);
}

//---------------------------------------------------------------------------------------------------------
// Function:		DrvPWM_EnablePwm
//
// Description:	This function is used to enable/disable PWM timer function
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvPWM_EnablePwm()
{
	PWM.PCR.PEN = 1;
}

static __inline
void
DrvPWM_DisablePwm()
{
	PWM.PCR.PEN = 0;
}
//---------------------------------------------------------------------------------------------------------
// Function:		DrvPWM_ClearPwmIntFlag
//
// Description:	This function is used to clear the PWM timer interrupt flag
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvPWM_ClearPwmIntFlag()
{
	PWM.PIIR.PWMIF = 1;
}

//---------------------------------------------------------------------------------------------------------
// Function:		DrvPWM_SetPwmMode
//
// Parameters:	u8PwmMode	- [in]	1: toggle mode, 0: on shot mode
//
// Description:	This function is used to set the One Shot mode function
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvPWM_SetPwmMode(
	UINT8 u8PwmMode
)
{
	PWM.PCR.PMOD = u8PwmMode;
}

//---------------------------------------------------------------------------------------------------------
// Function:		DrvPWM_EnablePwmInverter
//
// Description:	This function is used to enable/disable the Pwm Timer inverter function
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvPWM_EnablePwmInverter()
{
	PWM.PCR.PINV = 1;
}

static __inline
void
DrvPWM_DisablePwmInverter()
{
	PWM.PCR.PINV = 0;
}

//---------------------------------------------------------------------------------------------------------
// Function:		DrvPWM_EnablePwmInt
//
// Description:	This function is used to Enable / disable the PWM timer interrupt
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvPWM_EnablePwmInt()
{
	PWM.PIER.PWMIE0 = 1;
	NVIC_EnableIRQ(PWM0_IRQn);
}

static __inline
void
DrvPWM_DisablePwmInt()
{
	PWM.PIER.PWMIE0 = 0;
	NVIC_DisableIRQ(PWM0_IRQn);
}

static __inline
void
DrvPWM_StartPwmTimer(
	UINT8 u8Mode,
	UINT8 u8Prescaler,
	E_DRVPWM_CLOCKDIV eClockDiv,
	UINT16 u16CounterLoadedValue
)
{
	DrvPWM_SetPwmMode(u8Mode);
	PWM.PPR.CP0 = u8Prescaler;
	PWM.CSR.CSR0 = eClockDiv;
	PWM.CNR0.CNR = u16CounterLoadedValue;
	DrvPWM_ClearPwmIntFlag();
	DrvPWM_EnablePwm();
}

static __inline
void
DrvPWM_SetComparatorPwm(
	UINT8	u8Index,
	UINT16 	u16PwmComparator
)
{
	if( u8Index == 0 )
		PWM.CMR0.u32Reg = u16PwmComparator;
	else if( u8Index == 1 )
		PWM.CMR1.u32Reg = u16PwmComparator;
	else if( u8Index == 2 )
		PWM.CMR2.u32Reg = u16PwmComparator;
	else if( u8Index == 3 )
		PWM.CMR3.u32Reg = u16PwmComparator;
}

static __inline
void
DrvPWM_SetComparatorPwm0(
	UINT16 u16PwmComparator
)
{
	PWM.CMR0.u32Reg = u16PwmComparator;
}

static __inline
void
DrvPWM_SetComparatorPwm1(
	UINT16 u16PwmComparator
)
{
	PWM.CMR1.u32Reg = u16PwmComparator;
}

static __inline
void
DrvPWM_SetComparatorPwm2(
	UINT16 u16PwmComparator
)
{
	PWM.CMR2.u32Reg = u16PwmComparator;
}

static __inline
void
DrvPWM_SetComparatorPwm3(
	UINT16 u16PwmComparator
)
{
	PWM.CMR3.u32Reg = u16PwmComparator;
}

//---------------------------------------------------------------------------------------------------------
// Function:		DrvPWM_EnableDeadZone0
//
// Parameters:	u8Interval	- [in]	Dead Zone Interval : 0~255
//
// Description:	This function is used to set the dead zone0 Interval and enable/disable Dead Zone0 function
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvPWM_EnableDeadZone0(
	UINT8 u8Interval
)
{
	PWM.PCR.DZEN0 = 1;
	PWM.PPR.DZI0 = u8Interval;
}

static __inline
void
DrvPWM_DisableDeadZone0()
{
	PWM.PCR.DZEN0 = 0;
}
//---------------------------------------------------------------------------------------------------------
// Function:		DrvPWM_EnableDeadZone1
//
// Parameters:	u8Interval	- [in]	Dead Zone Interval : 0~255
//
// Description:	This function is used to set the dead zone1 Interval and enable/disable Dead Zone1 function
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvPWM_EnableDeadZone1(
	UINT8 u8Interval
)
{
	PWM.PCR.DZEN1 = 1;
	PWM.PPR.DZI1 = u8Interval;
}

static __inline
void
DrvPWM_DisableDeadZone1(
	UINT8 u8Interval
)
{
	PWM.PCR.DZEN1 = 0;
}
//---------------------------------------------------------------------------------------------------------
// Function:		DrvPWM_SetTimerCounter
//
// Parameters:	u16Counter	- [in]	Timer counter : 0~65535, If 0, PWM counter/timer will be stopped
//
// Description:	This function is used to set the PWM specified timer counter
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvPWM_SetTimerCounter(
	UINT16 u16Counter
)
{
	PWM.CNR0.CNR = u16Counter;
}

//---------------------------------------------------------------------------------------------------------
// Function:		DrvPWM_GetCurrentCounter
//
// Returns:			The specified timer counter value
//
// Description:	This function is used to get the PWM specified timer counter value
//---------------------------------------------------------------------------------------------------------
static __inline
UINT16
DrvPWM_GetCurrentCounter()
{
	return PWM.PDR0.PDR;
}

//---------------------------------------------------------------------------------------------------------
// Function:		DrvPWM_GetPwmIntFlag
//
// Returns:			0:FALSE, 1:TRUE
//
// Description:	This function is used to get the PWM timer interrupt flag
//---------------------------------------------------------------------------------------------------------
static __inline
BOOL
DrvPWM_GetPwmIntFlag()
{
	return PWM.PIIR.PWMIF;
}

//---------------------------------------------------------------------------------------------------------
// Function:		DrvPWM_EnablePwmOutput0
//
// Description:	This function is used to enable/disable PWM timer0 Output function
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvPWM_EnablePwmOutput0()
{
	PWM.PCE.PWME0 = 1;
}

static __inline
void
DrvPWM_DisablePwmOutput0()
{
	PWM.PCE.PWME0 = 0;
}
//---------------------------------------------------------------------------------------------------------
// Function:		DrvPWM_EnablePwmOutput
//
// Description:	This function is used to enable/disable PWM timer1 Output function
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvPWM_EnablePwmOutput( UINT8 u8Index )
{
	PWM.PCE.u32Reg |= (1<<u8Index);
}

static __inline
void
DrvPWM_DisablePwmOutput( UINT8 u8Index )
{
	PWM.PCE.u32Reg &= ~(1<<u8Index);
}
//---------------------------------------------------------------------------------------------------------
// Function:		DrvPWM_EnablePwmOutput1
//
// Description:	This function is used to enable/disable PWM timer1 Output function
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvPWM_EnablePwmOutput1()
{
	PWM.PCE.PWME1 = 1;
}

static __inline
void
DrvPWM_DisablePwmOutput1()
{
	PWM.PCE.PWME1 = 0;
}
//---------------------------------------------------------------------------------------------------------
// Function:		DrvPWM_EnablePwmOutput2
//
// Description:	This function is used to enable/disable PWM timer2 Output function
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvPWM_EnablePwmOutput2()
{
	PWM.PCE.PWME2 = 1;
}

static __inline
void
DrvPWM_DisablePwmOutput2()
{
	PWM.PCE.PWME2 = 0;
}
//---------------------------------------------------------------------------------------------------------
// Function:		DrvPWM_EnablePwmOutput3
//
// Parameters:	bEnable	- [in]	Enable  (true) / Diasble  (!true)
//
// Description:	This function is used to enable/disable PWM timer3 Output function
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvPWM_EnablePwmOutput3()
{
	PWM.PCE.PWME3 = 1;
}

static __inline
void
DrvPWM_DisablePwmOutput3()
{
	PWM.PCE.PWME3 = 0;
}
//=========================================================================================================
//==============================================  capture  ================================================
//=========================================================================================================

//---------------------------------------------------------------------------------------------------------
// Function:		DrvPWM_EnableCapture
//
// Description:	This function is used to enable/disable PWM capture function
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvPWM_EnableCapture()
{
	PWM.CCR0.u32Reg = (PWM.CCR0.u32Reg & ~(PWM_CCR0_CAPIF0 | PWM_CCR0_CAPEN)) | PWM_CCR0_CAPEN;
}

static __inline
void
DrvPWM_DisableCapture()
{
	PWM.CCR0.u32Reg = PWM.CCR0.u32Reg & ~(PWM_CCR0_CAPIF0 | PWM_CCR0_CAPEN);
}
//---------------------------------------------------------------------------------------------------------
// Function:		DrvPWM_EnableCaptureInverter
//
// Description:	This function is used to enable/disable the Capture inverter function
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvPWM_EnableCaptureInverter()
{
	PWM.CCR0.u32Reg = (PWM.CCR0.u32Reg & ~(PWM_CCR0_CAPIF0 | PWM_CCR0_INV0)) | PWM_CCR0_INV0;
}

static __inline
void
DrvPWM_DisableCaptureInverter()
{
	PWM.CCR0.u32Reg = PWM.CCR0.u32Reg & ~(PWM_CCR0_CAPIF0 | PWM_CCR0_INV0);
}
//---------------------------------------------------------------------------------------------------------
// Function:		DrvPWM_EnableCaptureRisingInt
//
// Description: This function is used to Enable / Disable the PWM capture Rising interrupt
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvPWM_EnableCaptureRisingInt()
{
	PWM.CCR0.u32Reg = (PWM.CCR0.u32Reg & ~(PWM_CCR0_CAPIF0 | PWM_CCR0_CRLRI0 | PWM_CCR0_RL_IE0)) | PWM_CCR0_RL_IE0;
	NVIC_EnableIRQ(PWM0_IRQn);
}

static __inline
void
DrvPWM_DisableCaptureRisingInt()
{
	PWM.CCR0.u32Reg = PWM.CCR0.u32Reg & ~(PWM_CCR0_CAPIF0 | PWM_CCR0_CRLRI0 |PWM_CCR0_RL_IE0);
}
//---------------------------------------------------------------------------------------------------------
// Function:		DrvPWM_EnableCaptureFallingInt
//
// Description: This function is used to Enable / Disable the PWM capture Falling interrupt
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvPWM_EnableCaptureFallingInt()
{
	PWM.CCR0.u32Reg = (PWM.CCR0.u32Reg & ~(PWM_CCR0_CAPIF0 | PWM_CCR0_CFLRI0 |PWM_CCR0_FL_IE0)) | PWM_CCR0_FL_IE0;
	NVIC_EnableIRQ(PWM0_IRQn);
}

static __inline
void
DrvPWM_DisableCaptureFallingInt()
{
	PWM.CCR0.u32Reg = PWM.CCR0.u32Reg & ~(PWM_CCR0_CAPIF0 | PWM_CCR0_CFLRI0 |PWM_CCR0_FL_IE0);
}
//---------------------------------------------------------------------------------------------------------
// Function:		DrvPWM_EnableCaptureInput
//
// Parameters:	bEnable	- [in]	Enable  (true) / Diasble  (!true)
//
// Description:	This function is used to enable/disable PWM capture input function
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvPWM_EnableCaptureInput()
{
	PWM.PCE.CAPIE0 = 1;
}

static __inline
void
DrvPWM_DisableCaptureInput()
{
	PWM.PCE.CAPIE0 = 0;
}
/*
static __inline
void
DrvPWM_InitCapture(
	BOOL bEnableCaptureInverter,
	BOOL bEnableCaptureRisingInt,
	BOOL bEnableCaptureFallingInt)
{
	DrvPWM_EnableCaptureInverter();
	DrvPWM_EnableCaptureRisingInt();
	DrvPWM_EnableCaptureFallingInt();
	DrvPWM_EnableCaptureInput();
	DrvPWM_EnableCapture();
}
*/
//---------------------------------------------------------------------------------------------------------
// Function:		DrvPWM_GetRisingCounter
//
// Returns:			The value which latches the counter when there・s a Rising transition
//
// Description:	This function is used to get value which latches the counter when there・s a Rising transition		                   														   */
//---------------------------------------------------------------------------------------------------------
static __inline
UINT16
DrvPWM_GetRisingCounter()
{
	return PWM.CRLR0.CRLR0;
}

//---------------------------------------------------------------------------------------------------------
// Function:		DrvPWM_GetFallingCounter
//
// Returns:			The value which latches the counter when there・s a falling transition
//
// Description:	This function is used to get value which latches the counter when there・s a falling transition		                   														   */
//---------------------------------------------------------------------------------------------------------
static __inline
UINT16
DrvPWM_GetFallingCounter()
{
	return PWM.CFLR0.CFLR0;
}

//---------------------------------------------------------------------------------------------------------
// Function:		DrvPWM_GetCaptureIntFlag
//
// Returns:			0:FALSE, 1:TRUE
//
// Description:	This function is used to get the PWM capture interrupt flag
//---------------------------------------------------------------------------------------------------------
static __inline
BOOL
DrvPWM_GetCaptureIntFlag()
{
	return PWM.CCR0.CAPIF0;
}

//---------------------------------------------------------------------------------------------------------
// Function:		DrvPWM_ClearCaptureIntFlag
//
// Description:	This function is used to clear the PWM capture interrupt flag
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvPWM_ClearCaptureIntFlag()
{
	PWM.CCR0.CAPIF0 = 1;
}

//---------------------------------------------------------------------------------------------------------
// Function: 		DrvPWM_GetCaptureFallingIntFlag
//
// Returns:			Check if there・s a falling transition  0:FALSE, 1:TRUE
//
// Description: This function is used to get the falling transition interrupt flag
//---------------------------------------------------------------------------------------------------------
static __inline
BOOL
DrvPWM_GetCaptureFallingIntFlag()
{
	return PWM.CCR0.CFLRI0;
}

//---------------------------------------------------------------------------------------------------------
// Function: 		DrvPWM_GetCaptureRisingIntFlag
//
// Returns:			Check if there・s a rising transition  0:FALSE, 1:TRUE
//
// Description: This function is used to get the rising transition interrupt flag
//---------------------------------------------------------------------------------------------------------
static __inline
BOOL
DrvPWM_GetCaptureRisingIntFlag()
{
	return PWM.CCR0.CRLRI0;
}

//---------------------------------------------------------------------------------------------------------
// Function:		DrvPWM_ClearCaptureFallingIntFlag
//
// Description:	This function is used to clear the falling transition interrupt flag
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvPWM_ClearCaptureFallingIntFlag()
{
	PWM.CCR0.u32Reg = (PWM.CCR0.u32Reg & ~(PWM_CCR0_CAPIF0 | PWM_CCR0_CFLRI0));
}

//---------------------------------------------------------------------------------------------------------
// Function:		DrvPWM_ClearCaptureRisingIntFlag
//
// Description:	This function is used to clear the rising transition interrupt flag
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvPWM_ClearCaptureRisingIntFlag()
{
	PWM.CCR0.u32Reg = (PWM.CCR0.u32Reg & ~(PWM_CCR0_CAPIF0 | PWM_CCR0_CRLRI0));
}


UINT32
DrvPWM_GetVersion(void);

#endif

