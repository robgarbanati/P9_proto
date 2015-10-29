/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2010 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#ifndef __DRVINT_H__
#define __DRVINT_H__

#include "Platform.h"
#include "System/SysInfra.h"
#include "Driver/DrvCLK.h"

//---------------------------------------------------------------------------------------------------------
//  Define Version number
//---------------------------------------------------------------------------------------------------------
#define DRVINT_MAJOR_NUM 1
#define DRVINT_MINOR_NUM 10
#define DRVINT_BUILD_NUM 001

#define DRVINT_VERSION_NUM    _SYSINFRA_VERSION(DRVINT_MAJOR_NUM, DRVINT_MINOR_NUM, DRVINT_BUILD_NUM)

#define DRVINT_WDT_MASK 	BIT0
#define DRVINT_APU_MASK 	BIT1
#define DRVINT_ADC_MASK		BIT2
#define DRVINT_EXINT_MASK	BIT3
#define DRVINT_USB_MASK		BIT4
#define DRVINT_TMR0_MASK	BIT5
#define DRVINT_TMR1_MASK	BIT6
#define DRVINT_TMR2_MASK	BIT7
#define DRVINT_GPAB_MASK	BIT8
#define DRVINT_SPI0_MASK	BIT9
#define DRVINT_PWM0_MASK	BIT10
#define DRVINT_SPI1_MASK	BIT11
#define DRVINT_TMRF_MASK	BIT12
#define DRVINT_RTC_MASK		BIT13
#define DRVINT_PWRWU_MASK	BIT14

typedef enum
{
	eDRVINT_WDT_INT 	= 0,
	eDRVINT_APU_INT 	= 1,
	eDRVINT_ADC_INT		= 2,
	eDRVINT_EXINT		= 3,
	eDRVINT_USB_INT		= 4,
	eDRVINT_TMR0_INT	= 5,
	eDRVINT_TMR1_INT	= 6,
	eDRVINT_TMR2_INT	= 7,
	eDRVINT_GPAB_INT	= 8,
	eDRVINT_SPI0_INT	= 9,
	eDRVINT_PWM0_INT	= 10,
	eDRVINT_SPI1_INT	= 11,
	eDRVINT_TMRF_INT	= 12,
	eDRVINT_RTC_INT		= 13,
	eDRVINT_PWRWU_INT	= 14
}E_DRVINT_SRC;
//---------------------------------------------------------------------------------------------------------
// Function:		DrvINT_GetMutiIntSrc
//
// Parameters:	eIntSrc  [in]  the interrupt source channel
//											eDRVINT_WDT_INT 	0
//											eDRVINT_APU_INT 	1
//											eDRVINT_ADC_INT		2
//											eDRVINT_EXINT			3
//											eDRVINT_USB_INT		4
//											eDRVINT_TMR0_INT	5
//											eDRVINT_TMR1_INT	6
//											eDRVINT_TMR2_INT	7
//											eDRVINT_GPAB_INT	8
//											eDRVINT_SPI0_INT	9
//											eDRVINT_PWM0_INT	10
//											eDRVINT_SPI1_INT	11
//											eDRVINT_TMRF_INT	12
//											eDRVINT_RTC_INT		13
//											eDRVINT_PWRWU_INT	14
// Returns:			the interrupt Multiplexer source identify status
//
// Description:	Get the interrupt Multiplexer source identify status.
//---------------------------------------------------------------------------------------------------------
static __inline
UINT32
DrvINT_GetMutiIntSrc(
	E_DRVINT_SRC eIntSrc
)
{
	return SYSINT.IRQN_SRC[eIntSrc].INT_SRC;
}

//---------------------------------------------------------------------------------------------------------
// Function:		DrvINT_SetNMISrcInt
//
// Parameters:	eIntSrc  [in]  the interrupt source channel
//											eDRVINT_WDT_INT 	0
//											eDRVINT_APU_INT 	1
//											eDRVINT_ADC_INT		2
//											eDRVINT_EXINT			3
//											eDRVINT_USB_INT		4
//											eDRVINT_TMR0_INT	5
//											eDRVINT_TMR1_INT	6
//											eDRVINT_TMR2_INT	7
//											eDRVINT_GPAB_INT	8
//											eDRVINT_SPI0_INT	9
//											eDRVINT_PWM0_INT	10
//											eDRVINT_SPI1_INT	11
//											eDRVINT_TMRF_INT	12
//											eDRVINT_RTC_INT		13
//											eDRVINT_PWRWU_INT	14
//
// Description:	Select the NMI interrupt source.
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvINT_SetNMISrcInt(
	E_DRVINT_SRC eIntSrc
)
{
	SYSINT.NMI_SEL.NMI_SEL = eIntSrc;
}

//---------------------------------------------------------------------------------------------------------
// Function:		DrvINT_EnableIntTestMode_P
//
// Description:	Enable interrupt test mode.
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvINT_EnableIntTestMode_P()
{
	SYSINT.NMI_SEL.INT_TEST = 1;
}

//---------------------------------------------------------------------------------------------------------
// Function:		DrvINT_DisableIntTestMode_P
//
// Description:	Disable interrupt test mode.
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvINT_DisableIntTestMode_P()
{
	SYSINT.NMI_SEL.INT_TEST = 0;
}
//---------------------------------------------------------------------------------------------------------
// Function:		DrvINT_GetMcuIrqFlag
//
// Parameters:	ui6McuIrqMask  [in]  the Bitwise of MCU IRQ source
//											DRVINT_WDT_MASK 		BIT0
//											DRVINT_APU_MASK 		BIT1
//											DRVINT_ADC_MASK		BIT2
//											DRVINT_EXINT_MASK	BIT3
//											DRVINT_USB_MASK		BIT4
//											DRVINT_TMR0_MASK		BIT5
//											DRVINT_TMR1_MASK		BIT6
//											DRVINT_TMR2_MASK		BIT7
//											DRVINT_GPAB_MASK		BIT8
//											DRVINT_SPI0_MASK		BIT9
//											DRVINT_PWM0_MASK		BIT10
//											DRVINT_SPI1_MASK		BIT11
//											DRVINT_TMRF_MASK		BIT12
//											DRVINT_RTC_MASK		BIT13
//											DRVINT_PWRWU_MASK	BIT14
// Returns:			an interrupt is asserted(true), an interrupt is not asserted(false)
//
// Description:	Get the MCU IRQ source status.
//---------------------------------------------------------------------------------------------------------
static __inline
BOOL
DrvINT_GetMcuIrqFlag(
	UINT16	u16McuIrqMask
)
{
	return SYSINT.MCU_IRQ.MCU_IRQ & u16McuIrqMask;
}

//---------------------------------------------------------------------------------------------------------
// Function:		DrvINT_SetMcuIrqFlag
//
// Parameters:	ui6McuIrqMask  [in]  the Bitwise of MCU IRQ source
//											DRVINT_WDT_MASK 		BIT0
//											DRVINT_APU_MASK 		BIT1
//											DRVINT_ADC_MASK		BIT2
//											DRVINT_EXINT_MASK	BIT3
//											DRVINT_USB_MASK		BIT4
//											DRVINT_TMR0_MASK		BIT5
//											DRVINT_TMR1_MASK		BIT6
//											DRVINT_TMR2_MASK		BIT7
//											DRVINT_GPAB_MASK		BIT8
//											DRVINT_SPI0_MASK		BIT9
//											DRVINT_PWM0_MASK		BIT10
//											DRVINT_SPI1_MASK		BIT11
//											DRVINT_TMRF_MASK		BIT12
//											DRVINT_RTC_MASK		BIT13
//											DRVINT_PWRWU_MASK	BIT14
// Returns:			void
//
// Description:	Set the MCU IRQ flag to generate an interrupt to MCU in test mode
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvINT_SetMcuIrqFlag(
	UINT16	u16McuIrqMask
)
{
	if (!DrvINT_GetMcuIrqFlag(u16McuIrqMask))
	{
		SYSINT.MCU_IRQ.MCU_IRQ = u16McuIrqMask;	// set flag to generate an interrupt in test mode
	}
}
//---------------------------------------------------------------------------------------------------------
// Function:		DrvINT_ClaerMcuIrqFlag
//
// Parameters:	ui6McuIrqMask  [in]  the Bitwise of MCU IRQ source
//											DRVINT_WDT_MASK 		BIT0
//											DRVINT_APU_MASK 		BIT1
//											DRVINT_ADC_MASK		BIT2
//											DRVINT_EXINT_MASK	BIT3
//											DRVINT_USB_MASK		BIT4
//											DRVINT_TMR0_MASK		BIT5
//											DRVINT_TMR1_MASK		BIT6
//											DRVINT_TMR2_MASK		BIT7
//											DRVINT_GPAB_MASK		BIT8
//											DRVINT_SPI0_MASK		BIT9
//											DRVINT_PWM0_MASK		BIT10
//											DRVINT_SPI1_MASK		BIT11
//											DRVINT_TMRF_MASK		BIT12
//											DRVINT_RTC_MASK		BIT13
//											DRVINT_PWRWU_MASK	BIT14
// Returns:			void
//
// Description:	Clear the MCU IRQ source flag.
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvINT_ClearMcuIrqFlag(
	UINT16	u16McuIrqMask
)
{
	if(DrvINT_GetMcuIrqFlag(u16McuIrqMask) == u16McuIrqMask)
	{
		SYSINT.MCU_IRQ.MCU_IRQ = u16McuIrqMask;
	}
}


UINT32
DrvINT_GetVersion(void);

#endif	// __DRVINT_H__

