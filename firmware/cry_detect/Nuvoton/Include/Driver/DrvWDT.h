/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2010 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#ifndef __DRVWDT_H__
#define __DRVWDT_H__

#include "Platform.h"
#include "System/SysInfra.h"
#include "Driver/DrvCLK.h"

//---------------------------------------------------------------------------------------------------------
//  Define Version number
//---------------------------------------------------------------------------------------------------------
#define DRVWDT_MAJOR_NUM 1
#define DRVWDT_MINOR_NUM 00
#define DRVWDT_BUILD_NUM 002

#define DRVWDT_VERSION_NUM    _SYSINFRA_VERSION(DRVWDT_MAJOR_NUM, DRVWDT_MINOR_NUM, DRVWDT_BUILD_NUM)

//---------------------------------------------------------------------------------------------------------
// Define WDT INTERVAL
//---------------------------------------------------------------------------------------------------------
#define DRVWDT_INTERVAL_LEVEL0 0x000         // 2^4(16) clocks
#define DRVWDT_INTERVAL_LEVEL1 0x100         // 2^6(64) APB clocks
#define DRVWDT_INTERVAL_LEVEL2 0x200         // 2^8(256) APB clocks
#define DRVWDT_INTERVAL_LEVEL3 0x300         // 2^10(1024) APB clocks
#define DRVWDT_INTERVAL_LEVEL4 0x400         // 2^12(4096) APB clocks
#define DRVWDT_INTERVAL_LEVEL5 0x500         // 2^14(16384) APB clocks
#define DRVWDT_INTERVAL_LEVEL6 0x600         // 2^16(65536) APB clocks
#define DRVWDT_INTERVAL_LEVEL7 0x700          // 2^18(262144) APB clocks

#define DRVWDT_FLAG_MASK			~(WDT_WTCR_WTWKF|WDT_WTCR_WTIF|WDT_WTCR_WTRF)	// mask wakeup/interrupt/reset flags
#define DRVWDT_LEVEL_MASK			~(WDT_WTCR_WTIS|WDT_WTCR_WTWKF|WDT_WTCR_WTIF|WDT_WTCR_WTRF)// mask intervel & wakeup/interrupt/reset flags
#define DRVWDT_ENABLE_INT_MASK		~(WDT_WTCR_WTIE|WDT_WTCR_WTWKF|WDT_WTCR_WTRF)  // mask enable_int & wakeup/reset flags
#define DRVWDT_INT_FLAG_MASK		~(WDT_WTCR_WTWKF|WDT_WTCR_WTRF)  // mask wakeup/reset flags
#define DRVWDT_ENABLE_WAKEUP_MASK	~(WDT_WTCR_WTWKE|WDT_WTCR_WTIF|WDT_WTCR_WTRF)  // mask wakeup &  interrupt/reset flags
#define DRVWDT_WAKEUP_FLAG_MASK		~(WDT_WTCR_WTIF|WDT_WTCR_WTRF)  // mask interrupt/reset flags
#define DRVWDT_ENABLE_RESET_MASK	~(WDT_WTCR_WTRE|WDT_WTCR_WTWKF|WDT_WTCR_WTIF)  // mask reset &  interrupt/wakeup flags
#define DRVWDT_RESET_FLAG_MASK		~(WDT_WTCR_WTWKF|WDT_WTCR_WTIF)  // mask interrupt/wakeup flags
//---------------------------------------------------------------------------------------------------------
// Define WDT Function Prototype
//---------------------------------------------------------------------------------------------------------
//---------------------------------------------------------------------------------------------------------
// Function:		DrvWDT_Open_P
//
// Parameters:	u16Level	- [in]	WDT Interval Level
//							bEnableWakeup	- [in]	1: Enable Wakeup, 0: Disable Wakeup
//							bEnableReset	- [in]	1: Enable Reset, 0: Disable Reset
//
// Description:	This function is used to Open WDT IP
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvWDT_Open_P(
	UINT16 u16Level,
	BOOL bEnableWakeup,
	BOOL bEnableReset
)
{
	DrvCLK_EnableApbDeviceClock(DRVCLK_CLK_WDT);

	WDT.WTCR.u32Reg = (u16Level|WDT_WTCR_WTWKF|WDT_WTCR_WTIF|WDT_WTCR_WTRF|WDT_WTCR_CLRWT|(bEnableWakeup<<4)|(bEnableReset<<1));
}

//---------------------------------------------------------------------------------------------------------
// Function:		DrvWDT_Close_P
//
// Description:	This function is used to Close WDT IP
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvWDT_Close_P()
{
    WDT.WTCR.u32Reg = (DRVWDT_INTERVAL_LEVEL7|WDT_WTCR_WTWKF|WDT_WTCR_WTIF|WDT_WTCR_WTRF|WDT_WTCR_CLRWT);	// default interval, clear int/reset/wakeup/counter bits                                           */

    DrvCLK_DisableApbDeviceClock(DRVCLK_CLK_WDT);
}

//---------------------------------------------------------------------------------------------------------
// Function:		DrvWDT_SetInterval_P
//
// Parameters:	u16Level	- [in]	WDT Interval Level
//
// Description:	This function is used to set WDT Interval
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvWDT_SetInterval_P(
	UINT16 u16Level
)
{
	WDT.WTCR.u32Reg = (WDT.WTCR.u32Reg & DRVWDT_LEVEL_MASK) | u16Level;
}

//---------------------------------------------------------------------------------------------------------
// Function:		DrvWDT_Enable_P
//
// Description:	This function is used to Enable WDT
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvWDT_Enable_P()
{
	WDT.WTCR.u32Reg = WDT.WTCR.u32Reg | WDT_WTCR_WTE;
}

//---------------------------------------------------------------------------------------------------------
// Function:		DrvWDT_Disable_P
//
// Description:	This function is used to Disable WDT
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvWDT_Disable_P()
{
	WDT.WTCR.u32Reg = WDT.WTCR.u32Reg & ~WDT_WTCR_WTE;
}

//---------------------------------------------------------------------------------------------------------
// Function:		DrvWDT_EnableInt_P
//
// Description:	This function is used to Enable WDT Int
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvWDT_EnableInt_P()
{
	// interrupt flag will be cleared here, if it's '1'.
	WDT.WTCR.u32Reg = (WDT.WTCR.u32Reg & DRVWDT_ENABLE_INT_MASK) | WDT_WTCR_WTIE;
	NVIC_EnableIRQ(WDT_IRQn);
}

//---------------------------------------------------------------------------------------------------------
// Function:		DrvWDT_DisableInt_P
//
// Description:	This function is used to Disable WDT Int
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvWDT_DisableInt_P()
{
	WDT.WTCR.u32Reg = (WDT.WTCR.u32Reg & DRVWDT_ENABLE_INT_MASK) & ~WDT_WTCR_WTIE;
	NVIC_DisableIRQ(WDT_IRQn);
}

//---------------------------------------------------------------------------------------------------------
// Function:		DrvWDT_GetIntFlag
//
// Returns:			0:Int not occur, 1:Int occur and delay some time to reset
//
// Description:	This function is used to Get WDT Int Flag Status
//---------------------------------------------------------------------------------------------------------
static __inline
BOOL
DrvWDT_GetIntFlag()
{
	return WDT.WTCR.WTIF;
}

//---------------------------------------------------------------------------------------------------------
// Function:		DrvWDT_ClearIntFlag_P
//
// Description:	This function is used to Clear WDT Int Flag
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvWDT_ClearIntFlag_P()
{
	WDT.WTCR.u32Reg = (WDT.WTCR.u32Reg & DRVWDT_INT_FLAG_MASK) | WDT_WTCR_WTIF;
}

//---------------------------------------------------------------------------------------------------------
// Function:		DrvWDT_EnableWakeup_P
//
// Description:	This function is used to Enable WDT Wakeup function
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvWDT_EnableWakeup_P()
{
	// wakeup flag will be cleared here, if it's '1'.
	WDT.WTCR.u32Reg = (WDT.WTCR.u32Reg & DRVWDT_ENABLE_WAKEUP_MASK) | WDT_WTCR_WTWKE;
}

//---------------------------------------------------------------------------------------------------------
// Function:		DrvWDT_DisableWakeup_P
//
// Description:	This function is used to Disable WDT Wakeup function
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvWDT_DisableWakeup_P()
{
	WDT.WTCR.u32Reg = (WDT.WTCR.u32Reg & DRVWDT_ENABLE_WAKEUP_MASK) & ~WDT_WTCR_WTWKE;
}

//---------------------------------------------------------------------------------------------------------
// Function:		DrvWDT_GetWakeupFlag
//
// Returns:			0:Wakeup not occur, 1:Wakeup occur by WDT
//
// Description:	This function is used to Get WDT Wakeup Flag Status
//---------------------------------------------------------------------------------------------------------
static __inline
BOOL
DrvWDT_GetWakeupFlag()
{
	return WDT.WTCR.WTWKF;
}

//---------------------------------------------------------------------------------------------------------
// Function:		DrvWDT_ClearWakeupFlag_P
//
// Description:	This function is used to Clear WDT Wakeup Flag
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvWDT_ClearWakeupFlag_P()
{
	WDT.WTCR.u32Reg = (WDT.WTCR.u32Reg & DRVWDT_WAKEUP_FLAG_MASK) | WDT_WTCR_WTWKF;
}

//---------------------------------------------------------------------------------------------------------
// Function:		DrvWDT_EnableReset_P
//
// Description:	This function is used to Enable WDT Reset function
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvWDT_EnableReset_P()
{
	// reset flag will be cleared here, if it's '1'.
	WDT.WTCR.u32Reg = (WDT.WTCR.u32Reg & DRVWDT_ENABLE_RESET_MASK) | WDT_WTCR_WTRE;
}

//---------------------------------------------------------------------------------------------------------
// Function:		DrvWDT_DisableReset_P
//
// Description:	This function is used to Disable WDT Reset function
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvWDT_DisableReset_P()
{
	WDT.WTCR.u32Reg = (WDT.WTCR.u32Reg & DRVWDT_ENABLE_RESET_MASK) & ~WDT_WTCR_WTRE;
}

//---------------------------------------------------------------------------------------------------------
// Function:		DrvWDT_GetResetFlag
//
// Returns:			0:Reset not occur, 1:Reset occur by WDT
//
// Description:	This function is used to Get WDT Reset Flag Status
//---------------------------------------------------------------------------------------------------------
static __inline
BOOL
DrvWDT_GetResetFlag()
{
	return WDT.WTCR.WTRF;
}

//---------------------------------------------------------------------------------------------------------
// Function:		DrvWDT_ClearResetFlag_P
//
// Description:	This function is used to Clear WDT Reset Flag
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvWDT_ClearResetFlag_P()
{
	WDT.WTCR.u32Reg = (WDT.WTCR.u32Reg & DRVWDT_RESET_FLAG_MASK) | WDT_WTCR_WTRF;
}

//---------------------------------------------------------------------------------------------------------
// Function:		DrvWDT_ClearCount_P
//
// Description:	This function is used to Clear WDT Count
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvWDT_ClearCount_P()
{
	WDT.WTCR.u32Reg = (WDT.WTCR.u32Reg & DRVWDT_FLAG_MASK) | WDT_WTCR_CLRWT;
}

//---------------------------------------------------------------------------------------------------------
// Function:		DrvWDT_GetVersion
//
// Description:	Get the version number of WDT driver.
//---------------------------------------------------------------------------------------------------------
UINT32
DrvWDT_GetVersion(void);

#endif	// __DRVWDT_H__

