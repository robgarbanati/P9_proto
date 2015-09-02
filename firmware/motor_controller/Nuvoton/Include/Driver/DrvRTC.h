/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2010 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#ifndef __DRVRTC_H__
#define __DRVRTC_H__

#include "Platform.h"
#include "Driver/DrvCLK.h"

//---------------------------------------------------------------------------------------------------------
//  Define Version number
//---------------------------------------------------------------------------------------------------------
#define DRVRTC_MAJOR_NUM 1
#define DRVRTC_MINOR_NUM 00
#define DRVRTC_BUILD_NUM 002

//---------------------------------------------------------------------------------------------------------
//  Version define with SysInfra
//---------------------------------------------------------------------------------------------------------
#define DRVRTC_VERSION_NUM    _SYSINFRA_VERSION(DRVRTC_MAJOR_NUM,DRVRTC_MINOR_NUM,DRVRTC_BUILD_NUM)

//---------------------------------------------------------------------------------------------------------
// Define for RTC Tick mode
//---------------------------------------------------------------------------------------------------------
typedef enum
{
 	eDRVRTC_TICK_4_SEC    = 0,   // 4     sec
 	eDRVRTC_TICK_1_2_SEC  = 1,   // 1/2   sec
 	eDRVRTC_TICK_1_8_SEC  = 2,   // 1/8   sec
 	eDRVRTC_TICK_1_32_SEC = 3,   // 1/32   sec
}E_DRVRTC_TICK;

//---------------------------------------------------------------------------------------------------------
// Define Function Prototype
//---------------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------------------
// Function:		DrvRTC_ClearIntFlag
//
// Description:	Clear the RTC Int Flag.
//--------------------------------------------------------------------------------------------------------------
static __inline
void
DrvRTC_ClearIntFlag()
{
	RTC.RTCR.RTIF = 1;
}

//--------------------------------------------------------------------------------------------------------------
// Function:		DrvRTC_Enable
//
// Description:	Enable the RTC.
//--------------------------------------------------------------------------------------------------------------
static __inline
void
DrvRTC_Enable()
{
	RTC.RTCR.u32Reg = RTC.RTCR.u32Reg | RTC_RTCR_RTCE;
}

//--------------------------------------------------------------------------------------------------------------
// Function:		DrvRTC_Disable
//
// Description:	Disable the RTC.
//--------------------------------------------------------------------------------------------------------------
static __inline
void
DrvRTC_Disable()
{
	RTC.RTCR.u32Reg = RTC.RTCR.u32Reg & ~(RTC_RTCR_RTCE);
}

//--------------------------------------------------------------------------------------------------------------
// Function:		DrvRTC_SetInterval
//
// Parameters:	eTick   [in]:  RTC Interval,one Tick for many secs
//												eDRVRTC_TICK_4_SEC = 0, eDRVRTC_TICK_1_2_SEC = 1, eDRVRTC_TICK_1_8_SEC = 2, eDRVRTC_TICK_1_32_SEC = 3.
//
// Description:	Set the RTC Interval.
//--------------------------------------------------------------------------------------------------------------
static __inline
void
DrvRTC_SetInterval(
	E_DRVRTC_TICK eTick
)
{
	RTC.RTCR.u32Reg = (RTC.RTCR.u32Reg & ~(RTC_RTCR_RTIF | RTC_RTCR_RTIS)) | (eTick<<3);
}

//--------------------------------------------------------------------------------------------------------------
// Function:		DrvRTC_EnableInt
//
// Description:	Enable the RTC Int.
//--------------------------------------------------------------------------------------------------------------
static __inline
void
DrvRTC_EnableInt()
{
	RTC.RTCR.u32Reg |= RTC_RTCR_RTIF | RTC_RTCR_RTIE;
	NVIC_EnableIRQ(RTC_IRQn);
}

//--------------------------------------------------------------------------------------------------------------
// Function:		DrvRTC_DisableInt
//
// Description:	Disable the RTC Int.
//--------------------------------------------------------------------------------------------------------------
static __inline
void
DrvRTC_DisableInt()
{
	RTC.RTCR.u32Reg = RTC.RTCR.u32Reg & (~RTC_RTCR_RTIE);
	NVIC_DisableIRQ(RTC_IRQn);
}

//--------------------------------------------------------------------------------------------------------------
// Function:		DrvRTC_Open
//
// Parameters:	eTick   [in]:  RTC Interval, one Tick for many secs
//												eDRVRTC_TICK_4_SEC = 0, eDRVRTC_TICK_1_2_SEC = 1, eDRVRTC_TICK_1_8_SEC = 2, eDRVRTC_TICK_1_32_SEC = 3.
//
// Description:	Open and start the RTC IP.
//--------------------------------------------------------------------------------------------------------------
static __inline
void
DrvRTC_Open(
	E_DRVRTC_TICK eTick
)
{
	DrvCLK_EnableApbDeviceClock(DRVCLK_CLK_RTC);
	DrvRTC_SetInterval(eTick);
	DrvRTC_Enable();
}

//--------------------------------------------------------------------------------------------------------------
// Function:		DrvRTC_Close
//
// Description:	Close the RTC IP.
//--------------------------------------------------------------------------------------------------------------
static __inline
void
DrvRTC_Close()
{
	DrvRTC_Disable();
	DrvCLK_DisableApbDeviceClock(DRVCLK_CLK_RTC);
}

//--------------------------------------------------------------------------------------------------------------
// Function:		DrvRTC_GetIntFlag
//
// Returns:			0:Int not occur, 1:Int occur
//
// Description:	Get RTC Int Flag.
//--------------------------------------------------------------------------------------------------------------
static __inline
BOOL
DrvRTC_GetIntFlag()
{
	return RTC.RTCR.RTIF;
}

UINT32
DrvRTC_GetVersion(void);

#endif /* __DRVRTC_H__ */

