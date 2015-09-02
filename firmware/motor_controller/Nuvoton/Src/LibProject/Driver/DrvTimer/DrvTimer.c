/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2010 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/* Include related header files                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
#include "Driver/DrvTimer.h"

/*---------------------------------------------------------------------------------------------------------*/
// Get Timer driver version
/*---------------------------------------------------------------------------------------------------------*/
UINT32
DrvTimer_GetVersion()
{
	return DRVTIMER_VERSION_NUM;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:
//				DrvTimer_WaitMicrosecondTmr0()
// Parameter:
//				u32TimeLen		[in]	The wait time in microsecond time unit.
//
// Description:
//				Wait a duration in microsecond time unit given by caller.
// Note:
//		1. 32K timer clock source is not allowed
//		2. Timer should be free for waiting count down.
//      3. Cannot wait over 65536 microseconds.
/*---------------------------------------------------------------------------------------------------------*/
void
DrvTimer_WaitMicrosecondTmr0(
	UINT32	u32TimeLen
)
{
	UINT16 u16TimerTotalCount = (UINT16)(u32TimeLen * DrvCLK_GetClkTmr0() / 1000000);
	DrvTimer_OpenTmr0(DRVTIMER_START_COUNTING | DRVTIMER_CONTINUOUS_COUNTING_MODE,65535);

	while(u16TimerTotalCount > DrvTimer_GetClkCountTmr0());

	DrvTimer_CloseTmr0();
}
/*---------------------------------------------------------------------------------------------------------*/
// refer to DrvTimer_WaitMicrosecondTmr0
/*---------------------------------------------------------------------------------------------------------*/
void
DrvTimer_WaitMicrosecondTmr1(
	UINT32	u32TimeLen
)
{
	UINT16 u16TimerTotalCount = u32TimeLen * DrvCLK_GetClkTmr1() / 1000000;
	DrvTimer_OpenTmr1(DRVTIMER_START_COUNTING | DRVTIMER_CONTINUOUS_COUNTING_MODE, 65535, 0);

	while(u16TimerTotalCount > DrvTimer_GetClkCountTmr1());

	DrvTimer_CloseTmr1();
}
/*---------------------------------------------------------------------------------------------------------*/
// refer to DrvTimer_WaitMicrosecondTmr0
/*---------------------------------------------------------------------------------------------------------*/
void
DrvTimer_WaitMicrosecondTmr2(
	UINT32	u32TimeLen
)
{
	UINT16 u16TimerTotalCount = u32TimeLen * DrvCLK_GetClkTmr2() / 1000000;
	DrvTimer_OpenTmr2(DRVTIMER_START_COUNTING | DRVTIMER_CONTINUOUS_COUNTING_MODE,65535);

	while(u16TimerTotalCount > DrvTimer_GetClkCountTmr2());

	DrvTimer_CloseTmr2();
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:
//				DrvTimer_WaitMillisecondTmr0()
// Parameter:
//				u32TimeLen		[in]	The wait time in Millisecond time unit.
//
// Description:
//				Wait a duration in Millisecond time unit given by caller.
// Note:
//		1. Timer should be free for waiting count down.
/*---------------------------------------------------------------------------------------------------------*/
void
DrvTimer_WaitMillisecondTmr0(
	UINT32	u32TimeLen
)
{
	UINT16 u16TimerCount,u16PreviousTimerCount=0;
	UINT16 u16PeriodicCount = (UINT16)(DrvCLK_GetClkTmr0() / 1000);		// counter for 1ms ==> 32 or 6000 or 12000 or 24000 or 48000
	DrvTimer_OpenTmr0(DRVTIMER_START_COUNTING | DRVTIMER_PERIODIC_MODE,u16PeriodicCount);

	while(1)
	{
		u16TimerCount = DrvTimer_GetClkCountTmr0();
		if (u16TimerCount < u16PreviousTimerCount)
		{
			u32TimeLen--;
			if (u32TimeLen == 0)
				break;
		}
		u16PreviousTimerCount = u16TimerCount;
	}
	DrvTimer_CloseTmr0();
}

/*---------------------------------------------------------------------------------------------------------*/
// refer to DrvTimer_WaitMillisecondTmr0
/*---------------------------------------------------------------------------------------------------------*/
void
DrvTimer_WaitMillisecondTmr1(
	UINT32	u32TimeLen
)
{
	UINT16 u16TimerCount,u16PreviousTimerCount=0;
	UINT16 u16PeriodicCount = DrvCLK_GetClkTmr1() / 1000;		// counter for 1ms ==> 32 or 6000 or 12000 or 24000 or 48000
	DrvTimer_OpenTmr1(DRVTIMER_START_COUNTING | DRVTIMER_PERIODIC_MODE,u16PeriodicCount, 0);

	while(1)
	{
		u16TimerCount = DrvTimer_GetClkCountTmr1();
		if (u16TimerCount < u16PreviousTimerCount)
		{
			u32TimeLen--;
			if (u32TimeLen == 0)
				break;
		}
		u16PreviousTimerCount = u16TimerCount;
	}
	DrvTimer_CloseTmr1();
}
/*---------------------------------------------------------------------------------------------------------*/
// refer to DrvTimer_WaitMillisecondTmr0
/*---------------------------------------------------------------------------------------------------------*/
void
DrvTimer_WaitMillisecondTmr2(
	UINT32	u32TimeLen
)
{
	UINT16 u16TimerCount,u16PreviousTimerCount=0;
	UINT16 u16PeriodicCount = DrvCLK_GetClkTmr2() / 1000;		// counter for 1ms ==> 32 or 6000 or 12000 or 24000 or 48000
	DrvTimer_OpenTmr2(DRVTIMER_START_COUNTING | DRVTIMER_PERIODIC_MODE,u16PeriodicCount);

	while(1)
	{
		u16TimerCount = DrvTimer_GetClkCountTmr2();
		if (u16TimerCount < u16PreviousTimerCount)
		{
			u32TimeLen--;
			if (u32TimeLen == 0)
				break;
		}
		u16PreviousTimerCount = u16TimerCount;
	}
	DrvTimer_CloseTmr2();
}

