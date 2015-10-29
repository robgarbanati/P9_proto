/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.  *
 *                                                              *
 ****************************************************************/

#ifndef __DRVTIMER_H__
#define __DRVTIMER_H__

// Include header file
#include "Platform.h"
#include "System/SysInfra.h"
#include "Driver/DrvCLK.h"
#include "Driver/DrvSYS.h"

// #define Constant
#define DRVTIMER_MAJOR_NUM 1
#define DRVTIMER_MINOR_NUM 10
#define DRVTIMER_BUILD_NUM 001
#define DRVTIMER_VERSION_NUM     _SYSINFRA_VERSION(DRVTIMER_MAJOR_NUM, DRVTIMER_MINOR_NUM, DRVTIMER_BUILD_NUM)

// #define Error Code
// none

// timer control constent
#define DRVTIMER_START_COUNTING					0x40000000
#define DRVTIMER_ENABLE_INTERRUPT				0x20000000

// timer mode setting
#define DRVTIMER_ONE_SHOT_MODE					0x0
#define DRVTIMER_PERIODIC_MODE					0x08000000
#define DRVTIMER_CONTINUOUS_COUNTING_MODE		0x18000000

// IR carrier
#define DRVTIMER_IR_CARRIER_OUTPUT_ENABLE		0x2
#define	DRVTIMER_NON_CARRIER_STATE				0x1	// IROUT keep high when IRCEN is 0(disable IR carrier out).

// APIs declaration

/*---------------------------------------------------------------------------------------------------------*/
// Get Timer driver version
/*---------------------------------------------------------------------------------------------------------*/
UINT32
DrvTimer_GetVersion(void);

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
);

/*---------------------------------------------------------------------------------------------------------*/
// refer to DrvTimer_WaitMicrosecondTmr1
/*---------------------------------------------------------------------------------------------------------*/
void
DrvTimer_WaitMicrosecondTmr1(
	UINT32	u32TimeLen
);

/*---------------------------------------------------------------------------------------------------------*/
// refer to DrvTimer_WaitMicrosecondTmr2
/*---------------------------------------------------------------------------------------------------------*/
void
DrvTimer_WaitMicrosecondTmr2(
	UINT32	u32TimeLen
);

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
);

/*---------------------------------------------------------------------------------------------------------*/
// refer to DrvTimer_WaitMillisecondTmr1
/*---------------------------------------------------------------------------------------------------------*/
void
DrvTimer_WaitMillisecondTmr1(
	UINT32	u32TimeLen
);

/*---------------------------------------------------------------------------------------------------------*/
// refer to DrvTimer_WaitMillisecondTmr2
/*---------------------------------------------------------------------------------------------------------*/
void
DrvTimer_WaitMillisecondTmr2(
	UINT32	u32TimeLen
);

// inline function declaration

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvTimer_OpenTmr0
// Parameter:
// 				u32OpenOption		[in] 	Open options, that is composed with bitw ise or and will be set
//											to TCSR register.
//											ex: (DRVTIMER_START_COUNTING|DRVTIMER_ENABLEINTERRUPT|
//												DRVTIMER_PERIOD_MODE|DRVTIMER_RESET_PRESCALE_COUNTER|150)
//
//											DRVTIMER_START_COUNTING (0x40000000, CEN)
//												: Timer will start count if it's set.
//											DRVTIMER_ENABLE_INTERRUPT (0x20000000, TIE)
//												: Enable tmer interrupt if it's set
//											DRVTIMER_ONE_SHOT_MODE (0x0, MODE, defult) :
//												or DRVTIMER_PERIODIC_MODE (0x08000000, MODE))
//												or DRVTIMER_CONTINUOUS_COUNTING_MODE (0x18000000, MODE)
//												: set operation mode
//											0~255 (PRESCALE)
//
//				u16CompareValue		[in]	Timer compare value (0~65536, but 0x0 and 0x1 are not allowed)
//
// Description:
//               Enable and start timer.
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvTimer_OpenTmr0(
	UINT32 u32OpenOption,
	UINT16 u16CompareValue
)
{
	DrvCLK_EnableApbDeviceClock(DRVCLK_CLK_TMR0);
	DrvSYS_ResetIP(DRVSYS_RST_TMR0);
	TMR0.TCMPR.TCMP = u16CompareValue;
	TMR0.TCSR.u32Reg = u32OpenOption;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvTimer_OpenTmr1
// Parameter:
// 				u32OpenOption		[in] 	refer to DrvTimer_OpenTmr0()
//				u16CompareValue		[in]	refer to DrvTimer_OpenTmr0()
//				u32IROption			[in]	IR option, that is composed with bitwaise or and will be set
//											to IRCAR register.
//											DRVTIMER_IR_CARRIER_OUTPUT_ENABLE
//											DRVTIMER_NON_CARRIER_STATE
//
// Description:
//               refer to DrvTimer_OpenTmr0()
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvTimer_OpenTmr1(
	UINT32 u32OpenOption,
	UINT16 u16CompareValue,
	UINT32 u32IROption
)
{
	DrvCLK_EnableApbDeviceClock(DRVCLK_CLK_TMR1);
	DrvSYS_ResetIP(DRVSYS_RST_TMR1);
	TMRIR.IRCAR.u32Reg = u32IROption;
	TMR1.TCMPR.TCMP = u16CompareValue;
	TMR1.TCSR.u32Reg = u32OpenOption;
}

/*---------------------------------------------------------------------------------------------------------*/
// refer to DrvTimer_OpenTmr2()
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvTimer_OpenTmr2(
	UINT32 u32OpenOption,
	UINT16 u16CompareValue
)
{
	DrvCLK_EnableApbDeviceClock(DRVCLK_CLK_TMR2);
	DrvSYS_ResetIP(DRVSYS_RST_TMR2);
	TMR2.TCMPR.TCMP = u16CompareValue;
	TMR2.TCSR.u32Reg = u32OpenOption;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:
//				DrvTimer_OpenTmrf()
// Parameter:
//				u32EnableInt		[in]	Enable interrupt or not. 1: enable, 0: disable.
//
// Description:
//				Enable and start fix frequency timer.
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvTimer_OpenTmrf()
{
	DrvCLK_EnableApbDeviceClock(DRVCLK_CLK_TMRF);
	DrvSYS_ResetIP(DRVSYS_RST_TMRF);
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:
//				DrvTimer_CloseTmr0
//
// Description:
//				Stop timer
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvTimer_CloseTmr0()
{
	DrvSYS_ResetIP(DRVSYS_RST_TMR0);
	DrvCLK_DisableApbDeviceClock(DRVCLK_CLK_TMR0);
}

/*---------------------------------------------------------------------------------------------------------*/
// refer to DrvTimer_CloseTmr1()
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvTimer_CloseTmr1()
{
	DrvSYS_ResetIP(DRVSYS_RST_TMR1);
	TMRIR.IRCAR.u32Reg = 0;
	DrvCLK_DisableApbDeviceClock(DRVCLK_CLK_TMR1);
}

/*---------------------------------------------------------------------------------------------------------*/
// refer to DrvTimer_CloseTmr2()
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvTimer_CloseTmr2()
{
	DrvSYS_ResetIP(DRVSYS_RST_TMR2);
	DrvCLK_DisableApbDeviceClock(DRVCLK_CLK_TMR2);
}

/*---------------------------------------------------------------------------------------------------------*/
// refer to DrvTimer_CloseTmrf()
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvTimer_CloseTmrf()
{
	DrvSYS_ResetIP(DRVSYS_RST_TMRF);
	DrvCLK_DisableApbDeviceClock(DRVCLK_CLK_TMRF);
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:
//				DrvTimer_EnableIntTmr0()
//
// Description:
//				Enable timer interrupt
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvTimer_EnableIntTmr0()
{
	TMR0.TCSR.TIE =1 ; 				//for 072 name is TMIE
	NVIC_EnableIRQ(TMR0_IRQn);
}

/*---------------------------------------------------------------------------------------------------------*/
// refer to DrvTimer_EnableIntTmr1()
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvTimer_EnableIntTmr1()
{
	TMR1.TCSR.TIE =1 ; 				//for 072 name is TMIE
	NVIC_EnableIRQ(TMR1_IRQn);
}

/*---------------------------------------------------------------------------------------------------------*/
// refer to DrvTimer_EnableIntTmr2()
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvTimer_EnableIntTmr2()
{
	TMR2.TCSR.TIE =1 ; 				//for 072 name is TMIE
	NVIC_EnableIRQ(TMR2_IRQn);
}

/*---------------------------------------------------------------------------------------------------------*/
// refer to DrvTimer_EnableIntTmrf()
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvTimer_EnableIntTmrf()
{
	TMRF.TFIR.u32Reg = (TMRF.TFIR.u32Reg & ~BIT0) | BIT1 ;
	NVIC_EnableIRQ(TMRF_IRQn);
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:
//				DrvTimer_DisableIntTmr0()
//
// Description:
//				Disable timer interrupt.
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvTimer_DisableIntTmr0()
{
	TMR0.TCSR.TIE = 0 ; 				//for 072 name is TMIE
	NVIC_DisableIRQ(TMR0_IRQn);
}

/*---------------------------------------------------------------------------------------------------------*/
// refer to DrvTimer_DisableIntTmr1()
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvTimer_DisableIntTmr1()
{
	TMR1.TCSR.TIE = 0 ; 				//for 072 name is TMIE
	NVIC_DisableIRQ(TMR1_IRQn);
}

/*---------------------------------------------------------------------------------------------------------*/
// refer to DrvTimer_DisableIntTmr2()
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvTimer_DisableIntTmr2()
{
	TMR2.TCSR.TIE = 0 ; 				//for 072 name is TMIE
	NVIC_DisableIRQ(TMR2_IRQn);
}

/*---------------------------------------------------------------------------------------------------------*/
// refer to DrvTimer_DisableIntTmrf()
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvTimer_DisableIntTmrf()
{
	TMRF.TFIR.u32Reg = TMRF.TFIR.u32Reg & ~(BIT0|BIT1) ;
	NVIC_DisableIRQ(TMRF_IRQn);
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:
//				DrvTimer_GetIntFlagTmr0()
//
// Return:
//				Interrupt flag. 1: time out and flag is set, 0: is not yet timer out.
// Description:
//				Check if timer interrupt flag is set or not.
/*---------------------------------------------------------------------------------------------------------*/
static __inline
UINT8
DrvTimer_GetIntFlagTmr0()
{
	return (UINT8) TMR0.TISR.TIF; 				//for 072 name is TMIF
}
/*---------------------------------------------------------------------------------------------------------*/
// refer to DrvTimer_GetIntFlagTmr1()
/*---------------------------------------------------------------------------------------------------------*/
static __inline
UINT8
DrvTimer_GetIntFlagTmr1()
{
	return (UINT8) TMR1.TISR.TIF; 				//for 072 name is TMIF
}
/*---------------------------------------------------------------------------------------------------------*/
// refer to DrvTimer_GetIntFlagTmr2()
/*---------------------------------------------------------------------------------------------------------*/
static __inline
UINT8
DrvTimer_GetIntFlagTmr2()
{
	return (UINT8) TMR2.TISR.TIF; 				//for 072 name is TMIF
}
/*---------------------------------------------------------------------------------------------------------*/
// refer to DrvTimer_GetIntFlagTmrf()
/*---------------------------------------------------------------------------------------------------------*/
static __inline
UINT8
DrvTimer_GetIntFlagTmrf()
{
	return (UINT8) TMRF.TFIR.TFIF;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:
//				DrvTimer_ClearIntFlagTmr0()
//
// Description:
//				Clear interrupt flag. So we can check it's time out again by checking the flag.
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvTimer_ClearIntFlagTmr0()
{
	TMR0.TISR.TIF = 1; 				//for 072 name is TMIF
}
/*---------------------------------------------------------------------------------------------------------*/
// refer to DrvTimer_ClearIntFlagTmr1()
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvTimer_ClearIntFlagTmr1()
{
	TMR1.TISR.TIF = 1; 				//for 072 name is TMIF
}
/*---------------------------------------------------------------------------------------------------------*/
// refer to DrvTimer_ClearIntFlagTmr2()
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvTimer_ClearIntFlagTmr2()
{
	TMR2.TISR.TIF = 1; 				//for 072 name is TMIF
}
/*---------------------------------------------------------------------------------------------------------*/
// refer to DrvTimer_ClearIntFlagTmrf()
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvTimer_ClearIntFlagTmrf()
{
	TMRF.TFIR.TFIF = 1;
}

static __inline
void
DrvTimer_SetCompareValueTmr0(
	UINT16 u16CompareValue
)
{
	TMR0.TCMPR.TCMP = u16CompareValue;
}

static __inline
void
DrvTimer_SetCompareValueTmr1(
	UINT16 u16CompareValue
)
{
	TMR1.TCMPR.TCMP = u16CompareValue;
}

static __inline
void
DrvTimer_SetCompareValueTmr2(
	UINT16 u16CompareValue
)
{
	TMR2.TCMPR.TCMP = u16CompareValue;
}
/*---------------------------------------------------------------------------------------------------------*/
// Function:
//				DrvTimer_GetClkCountTmr0()
//
// Return:
//				Current timer clock counter value.
// Description:
//				Get current timer clock counter value.
/*---------------------------------------------------------------------------------------------------------*/
static __inline
UINT16
DrvTimer_GetClkCountTmr0()
{
	return (UINT16)TMR0.TDR.TDR;
}
/*---------------------------------------------------------------------------------------------------------*/
// refer to DrvTimer_GetClkCountTmr1()
/*---------------------------------------------------------------------------------------------------------*/
static __inline
UINT16
DrvTimer_GetClkCountTmr1()
{
	return (UINT16)TMR1.TDR.TDR;
}
/*---------------------------------------------------------------------------------------------------------*/
// refer to DrvTimer_GetClkCountTmr2()
/*---------------------------------------------------------------------------------------------------------*/
static __inline
UINT16
DrvTimer_GetClkCountTmr2()
{
	return (UINT16)TMR2.TDR.TDR;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:
//				DrvTimer_EnableTmr0()
// Parameter:
//				u8Enable		[in]	Enable or disable timer counting. 1: enable, 0: disable
//
// Description:
//				Enable or disable timer counting.
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvTimer_EnableTmr0()
{
	TMR0.TCSR.CEN = 1; 				//for 072 name is TCEN
}

static __inline
void
DrvTimer_DisableTmr0()
{
	TMR0.TCSR.CEN = 0; 				//for 072 name is TCEN
}
/*---------------------------------------------------------------------------------------------------------*/
// refer to DrvTimer_EnableTmr1
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvTimer_EnableTmr1()
{
	TMR1.TCSR.CEN = 1; 				//for 072 name is TCEN
}

static __inline
void
DrvTimer_DisableTmr1()
{
	TMR1.TCSR.CEN = 0; 				//for 072 name is TCEN
}
/*---------------------------------------------------------------------------------------------------------*/
// refer to DrvTimer_EnableTmr2
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvTimer_EnableTmr2()
{
	TMR2.TCSR.CEN = 1; 				//for 072 name is TCEN
}

static __inline
void
DrvTimer_DisableTmr2()
{
	TMR2.TCSR.CEN = 0; 				//for 072 name is TCEN
}
/*---------------------------------------------------------------------------------------------------------*/
// Function:
//				DrvTimer_ResetCounterTmr0()
//
// Return:
//				void
// Description:
//				Reset current timer counter.
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvTimer_ResetCounterTmr0()
{
	TMR0.TCSR.CRST = 1;
}
/*---------------------------------------------------------------------------------------------------------*/
// refer to DrvTimer_ResetCounterTmr1
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvTimer_ResetCounterTmr1()
{
	TMR1.TCSR.CRST = 1;
}
/*---------------------------------------------------------------------------------------------------------*/
// refer to DrvTimer_ResetCounterTmr2
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvTimer_ResetCounterTmr2()
{
	TMR2.TCSR.CRST = 1;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:
//				DrvTimer_IsActiveTmr0()
//
// Return:
//				Active or not. 1: active, 0: not active.
// Description:
//				Check timer is active or not.
/*---------------------------------------------------------------------------------------------------------*/
static __inline
UINT8
DrvTimer_IsActiveTmr0()
{
	return (UINT8)TMR0.TCSR.CACT;
}
/*---------------------------------------------------------------------------------------------------------*/
// refer to DrvTimer_IsActiveTmr1
/*---------------------------------------------------------------------------------------------------------*/
static __inline
UINT8
DrvTimer_IsActiveTmr1()
{
	return (UINT8)TMR1.TCSR.CACT;
}
/*---------------------------------------------------------------------------------------------------------*/
// refer to DrvTimer_IsActiveTmr2
/*---------------------------------------------------------------------------------------------------------*/
static __inline
UINT8
DrvTimer_IsActiveTmr2()
{
	return (UINT8)TMR2.TCSR.CACT;
}

#endif	// __DRVTIMER_H__

