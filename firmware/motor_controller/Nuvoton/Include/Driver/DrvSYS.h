/********************************************************************
 *																	*
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.	*
 *																	*
 ********************************************************************/

#ifndef __DRVSYS_H__
#define __DRVSYS_H__

// --------------------------------------------------------------
// Include Header Files
// --------------------------------------------------------------
#include "Platform.h"
#include "System/SysInfra.h"

#ifdef  __cplusplus
extern "C"
{
#endif

// #define Constant

// --------------------------------------------------------------
// Version Definition and Error Code
// --------------------------------------------------------------
#define	DRVSYS_MAJOR_NUM 2
#define	DRVSYS_MINOR_NUM 00
#define	DRVSYS_BUILD_NUM 001

#define DRVSYS_VERSION_NUM	_SYSINFRA_VERSION(DRVSYS_MAJOR_NUM, DRVSYS_MINOR_NUM, DRVSYS_BUILD_NUM)

//E_DRVSYS_ERR_UNLOCK_FAIL		Unlock key address failed
//E_DRVSYS_ERR_LOCK_FAIL		Lock key address failed
//E_DRVSYS_ERR_ARGUMENT		Wrong Argument
//E_DRVSYS_ERR_SRC_SEL			Wrong clock selection
//E_DRVSYS_ERR_CLK_DIV			Wrong divider selection

#define E_DRVSYS_ERR_UNLOCK_FAIL	_SYSINFRA_ERRCODE(TRUE, MODULE_ID_DRVSYS, 1)
#define E_DRVSYS_ERR_LOCK_FAIL		_SYSINFRA_ERRCODE(TRUE, MODULE_ID_DRVSYS, 2)
#define E_DRVSYS_ERR_ARGUMENT		_SYSINFRA_ERRCODE(TRUE, MODULE_ID_DRVSYS, 3)
#define E_DRVSYS_ERR_SRC_SEL		_SYSINFRA_ERRCODE(TRUE, MODULE_ID_DRVSYS, 4)
#define E_DRVSYS_ERR_CLK_DIV		_SYSINFRA_ERRCODE(TRUE, MODULE_ID_DRVSYS, 5)

// System reset source
#define DRVSYS_RST_SRC_POR		SYS_RSTSRC_RSTS_POR
#define DRVSYS_RST_SRC_PAD		SYS_RSTSRC_RSTS_PAD
#define DRVSYS_RST_SRC_WDT		SYS_RSTSRC_RSTS_WDT
#define DRVSYS_RST_SRC_LVR		SYS_RSTSRC_RSTS_LVR
#if defined(__N572F064__)||defined(__N572F065__)
#define DRVSYS_RST_SRC_MCU		SYS_RSTSRC_RSTS_MCU
#endif
#define DRVSYS_RST_SRC_PMU		SYS_RSTSRC_RSTS_PMU

#define DRVSYS_FLASH_MEMORY_WAIT_STATE_1_HCLK	0
#define DRVSYS_FLASH_MEMORY_WAIT_STATE_0_HCLK	1

#define DRVSYS_RAM_WAIT_STATE_1_HCLK	0
#define DRVSYS_RAM_WAIT_STATE_0_HCLK	1

#define DRVSYS_VOLTAGE_DETECTOR_ENABLE			SYS_VDCR_VD_EN
#define DRVSYS_VOLTAGE_DETECTOR_THRESHOLD_3V	SYS_VDCR_VD_VL

#define	DRVSYS_GPIO_SCHMITT_TG_0	BIT16
#define	DRVSYS_GPIO_SCHMITT_TG_1	BIT17
#define	DRVSYS_GPIO_SCHMITT_TG_2	BIT18
#define	DRVSYS_GPIO_SCHMITT_TG_3	BIT19
#define	DRVSYS_GPIO_SCHMITT_TG_4	BIT20
#define	DRVSYS_GPIO_SCHMITT_TG_5	BIT21
#define	DRVSYS_GPIO_SCHMITT_TG_6	BIT22
#define	DRVSYS_GPIO_SCHMITT_TG_7	BIT23
#define	DRVSYS_GPIO_SCHMITT_TG_8	BIT24
#define	DRVSYS_GPIO_SCHMITT_TG_9	BIT25
#define	DRVSYS_GPIO_SCHMITT_TG_10	BIT26
#define	DRVSYS_GPIO_SCHMITT_TG_11	BIT27
#define	DRVSYS_GPIO_SCHMITT_TG_12	BIT28
#define	DRVSYS_GPIO_SCHMITT_TG_13	BIT29
#define	DRVSYS_GPIO_SCHMITT_TG_14	BIT30
#define	DRVSYS_GPIO_SCHMITT_TG_15	BIT31

#define DRVSYS_GPIOA_MF0_SPI0_2ND_CHIP_SEL_OUT	BIT0
#define DRVSYS_GPIOA_MF1_SPI0_1ST_CHIP_SEL_OUT	BIT1
#define DRVSYS_GPIOA_MF2_SPI0_CLOCK_OUT			BIT2
#define DRVSYS_GPIOA_MF3_SPI0_DATA_IN			BIT3
#define DRVSYS_GPIOA_MF4_SPI0_DATA_OUT			BIT4
#define DRVSYS_GPIOA_MF5_TMR0_COUNTER_EXT_IN	BIT5
#define DRVSYS_GPIOA_MF6_EXT_INTERRUPT_IN		BIT6
#define DRVSYS_GPIOA_MF7_ADC_EXT_TG_IN			BIT7
#define DRVSYS_GPIOA_MF8_ADC_CHANNEL0_IN		BIT8
#define DRVSYS_GPIOA_MF9_ADC_CHANNEL1_IN		BIT9
#define DRVSYS_GPIOA_MF10_ADC_CHANNEL2_IN		BIT10
#define DRVSYS_GPIOA_MF11_ADC_CHANNEL3_IN		BIT11
#define DRVSYS_GPIOA_MF12_ADC_CHANNEL4_IN		BIT12
#define DRVSYS_GPIOA_MF13_ADC_CHANNEL5_IN		BIT13
#define DRVSYS_GPIOA_MF14_ADC_CHANNEL6_IN		BIT14
#define DRVSYS_GPIOA_MF15_ADC_CHANNEL7_IN		BIT15


#define DRVSYS_GPIOB_MF0_SPI1_2ND_CHIP_SEL_OUT	BIT0
#define DRVSYS_GPIOB_MF1_SPI1_1ST_CHIP_SEL_OUT	BIT1
#define DRVSYS_GPIOB_MF2_SPI1_CLOCK_OUT			BIT2
#define DRVSYS_GPIOB_MF3_SPI1_DATA_IN			BIT3
#define DRVSYS_GPIOB_MF4_SPI1_DATA_OUT			BIT4
#define DRVSYS_GPIOB_MF8_PWM_OUT_0				BIT8
#define DRVSYS_GPIOB_MF9_PWM_OUT_1				BIT9
#define DRVSYS_GPIOB_MF10_PWM_OUT_2				BIT10
#define DRVSYS_GPIOB_MF11_PWM_OUT_3				BIT11
#define DRVSYS_GPIOB_MF12_PWM_TMR_CAPTURE_IN	BIT12
#define DRVSYS_GPIOB_MF13_IR_CARRIER_OUT		BIT13
#define DRVSYS_GPIOB_MF14_TMR1_COUNTER_EXT_IN	BIT14
#define DRVSYS_GPIOB_MF15_TMR2_COUNTER_EXT_IN	BIT15

//---------------------------------------------------------------
// IP Reset (IPRSTC2)
//---------------------------------------------------------------

#define DRVSYS_RST_GPIO		SYS_IPRSTC2_GPIO_RST
#define DRVSYS_RST_TMR0		SYS_IPRSTC2_TMR0_RST
#define DRVSYS_RST_TMR1		SYS_IPRSTC2_TMR1_RST
#define DRVSYS_RST_TMR2		SYS_IPRSTC2_TMR2_RST
#define DRVSYS_RST_APU		SYS_IPRSTC2_APU_RST
#define DRVSYS_RST_TMRF		SYS_IPRSTC2_TMRF_RST
#define DRVSYS_RST_SPI0		SYS_IPRSTC2_SPI0_RST
#define DRVSYS_RST_SPI1		SYS_IPRSTC2_SPI1_RST
#define DRVSYS_RST_PWM		SYS_IPRSTC2_PWM_RST
#if defined(__N572F065__)
#define DRVSYS_RST_USBD		SYS_IPRSTC2_USBD_RST
#endif
#define DRVSYS_RST_ADC		SYS_IPRSTC2_ADC_RST

// APIs declaration
//---------------------------------------------------------------------------------------------------------
// Function: DrvSYS_ClearPORDisableCode_P
//
// Parameters:
//
// Returns:
//               None
//
// Description:
//               Clear POR disable code to enable power on reset
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvSYS_ClearPORDisableCode_P(void)
{
	SYS.PORCR.u32Reg = 0;
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvSYS_ClearRstSrc
//
// Parameters:
//               u8RstSrc      		[in]  bitwise or to indicate the reset source to be cleared
//											DRVSYS_RST_SRC_POR
//											DRVSYS_RST_SRC_PAD
//											DRVSYS_RST_SRC_WDT
//											DRVSYS_RST_SRC_LVR
//											DRVSYS_RST_SRC_MCU	, N572F072 removed this option
//											DRVSYS_RST_SRC_PMU
//
// Description:
//               Clear the indicating reset source
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvSYS_ClearRstSrc(
	UINT8 u8RstSrc
)
{
	SYS.RSTSRC.u32Reg &= ~u8RstSrc;
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvSYS_DisableLowVoltageReset_P
//
// Description:
//               Disable low voltage reset.
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvSYS_DisableLowVoltageReset_P(void)
{
	SYS.VDCR.LVR_EN = 0;
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvSYS_DisableMultifunctionGpioa
//
// Parameters:
//				u16Multifunction	[in]	Bitwise or to disable GPIOA multifunction selection
//												DRVSYS_GPIOA_MF0_SPI0_2ND_CHIP_SEL_OUT
//												DRVSYS_GPIOA_MF1_SPI0_1ST_CHIP_SEL_OUT
//												DRVSYS_GPIOA_MF2_SPI0_CLOCK_OUT
//												DRVSYS_GPIOA_MF3_SPI0_DATA_IN
//												DRVSYS_GPIOA_MF4_SPI0_DATA_OUT
//												DRVSYS_GPIOA_MF5_TMR0_COUNTER_EXT_IN
//												DRVSYS_GPIOA_MF6_EXT_INTERRUPT_IN
//												DRVSYS_GPIOA_MF7_ADC_EXT_TG_IN
//												DRVSYS_GPIOA_MF8_ADC_CHANNEL0_IN
//												DRVSYS_GPIOA_MF9_ADC_CHANNEL1_IN
//												DRVSYS_GPIOA_MF10_ADC_CHANNEL2_IN
//												DRVSYS_GPIOA_MF11_ADC_CHANNEL3_IN
//												DRVSYS_GPIOA_MF12_ADC_CHANNEL4_IN
//												DRVSYS_GPIOA_MF13_ADC_CHANNEL5_IN
//												DRVSYS_GPIOA_MF14_ADC_CHANNEL6_IN
//												DRVSYS_GPIOA_MF15_ADC_CHANNEL7_IN
//
// Description:
//              Disable GPIOA multifunction pin selection.
//				If it's disabled, it will be GPIO pin.
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvSYS_DisableMultifunctionGpioa(
	UINT16 u16Multifunction
)
{
	SYS.GPA_MFP.u32Reg &= ~u16Multifunction;
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvSYS_DisableMultifunctionGpiob
//
// Parameters:
//				u16Multifunction	[in]	Bitwise or to disable GPIOB multifunction selection
//												DRVSYS_GPIOB_MF0_SPI1_2ND_CHIP_SEL_OUT
//												DRVSYS_GPIOB_MF1_SPI1_1ST_CHIP_SEL_OUT
//												DRVSYS_GPIOB_MF2_SPI1_CLOCK_OUT
//												DRVSYS_GPIOB_MF3_SPI1_DATA_IN
//												DRVSYS_GPIOB_MF4_SPI1_DATA_OUT
//												DRVSYS_GPIOB_MF8_PWM_OUT_0
//												DRVSYS_GPIOB_MF9_PWM_OUT_1
//												DRVSYS_GPIOB_MF10_PWM_OUT_2
//												DRVSYS_GPIOB_MF11_PWM_OUT_3
//												DRVSYS_GPIOB_MF12_PWM_TMR_CAPTURE_IN
//												DRVSYS_GPIOB_MF13_IR_CARRIER_OUT
//												DRVSYS_GPIOB_MF14_TMR1_COUNTER_EXT_IN
//												DRVSYS_GPIOB_MF15_TMR2_COUNTER_EXT_IN
//
// Description:
//              Disable GPIOB multifunction pin selection.
//				If it's disabled, it will be GPIO pin.
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvSYS_DisableMultifunctionGpiob(
	UINT16 u16Multifunction
)
{
	SYS.GPB_MFP.u32Reg &= ~u16Multifunction;
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvSYS_DisableSchmittTriggerGpioa
//
// Parameters:
//				u16DisableIO			[in]	Bitwise or to disable GPIOA I/O cell input schmitt triggle function
// 												DRVSYS_GPIO_SCHMITT_TG_0
// 												DRVSYS_GPIO_SCHMITT_TG_1
// 												DRVSYS_GPIO_SCHMITT_TG_2
// 												DRVSYS_GPIO_SCHMITT_TG_3
// 												DRVSYS_GPIO_SCHMITT_TG_4
// 												DRVSYS_GPIO_SCHMITT_TG_5
// 												DRVSYS_GPIO_SCHMITT_TG_6
// 												DRVSYS_GPIO_SCHMITT_TG_7
// 												DRVSYS_GPIO_SCHMITT_TG_8
// 												DRVSYS_GPIO_SCHMITT_TG_9
// 												DRVSYS_GPIO_SCHMITT_TG_10
// 												DRVSYS_GPIO_SCHMITT_TG_11
// 												DRVSYS_GPIO_SCHMITT_TG_12
// 												DRVSYS_GPIO_SCHMITT_TG_13
// 												DRVSYS_GPIO_SCHMITT_TG_14
// 												DRVSYS_GPIO_SCHMITT_TG_15
//
// Description:
//              Disable GPIOA I/O cell input schmitt triggle function
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvSYS_DisableSchmittTriggerGpioa(
	UINT32 u32DisableIO
)
{
	SYS.GPA_MFP.u32Reg &= ~u32DisableIO;
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvSYS_DisableSchmittTriggerGpiob
//
// Parameters:
//				u16DisableIO			[in]	Bitwise or to disable GPIOB I/O cell input schmitt triggle function
// 												DRVSYS_GPIO_SCHMITT_TG_0
// 												DRVSYS_GPIO_SCHMITT_TG_1
// 												DRVSYS_GPIO_SCHMITT_TG_2
// 												DRVSYS_GPIO_SCHMITT_TG_3
// 												DRVSYS_GPIO_SCHMITT_TG_4
// 												DRVSYS_GPIO_SCHMITT_TG_5
// 												DRVSYS_GPIO_SCHMITT_TG_6
// 												DRVSYS_GPIO_SCHMITT_TG_7
// 												DRVSYS_GPIO_SCHMITT_TG_8
// 												DRVSYS_GPIO_SCHMITT_TG_9
// 												DRVSYS_GPIO_SCHMITT_TG_10
// 												DRVSYS_GPIO_SCHMITT_TG_11
// 												DRVSYS_GPIO_SCHMITT_TG_12
// 												DRVSYS_GPIO_SCHMITT_TG_13
// 												DRVSYS_GPIO_SCHMITT_TG_14
// 												DRVSYS_GPIO_SCHMITT_TG_15
//
// Description:
//              Disable GPIOB I/O cell input schmitt triggle function
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvSYS_DisableSchmittTriggerGpiob(
	UINT32 u32DisableIO
)
{
	SYS.GPB_MFP.u32Reg &= ~u32DisableIO;
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvSYS_DisableVoltageDetector_P
//
// Parameters:
//				void
// Returns:
//               None
//
// Description:
//               Disable voltage detector.
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvSYS_DisableVoltageDetector_P(void)
{
	SYS.VDCR.VD_EN = 0;
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvSYS_EnableLowVoltageReset_P
//
// Description:
//               Enable low voltage reset.
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvSYS_EnableLowVoltageReset_P(void)
{
	SYS.VDCR.LVR_EN = 1;
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvSYS_EnableMultifunctionGpioa
//
// Parameters:
//				u16Multifunction	[in]	Bitwise or to select GPIOA multifunction selection
//												DRVSYS_GPIOA_MF0_SPI0_2ND_CHIP_SEL_OUT
//												DRVSYS_GPIOA_MF1_SPI0_1ST_CHIP_SEL_OUT
//												DRVSYS_GPIOA_MF2_SPI0_CLOCK_OUT
//												DRVSYS_GPIOA_MF3_SPI0_DATA_IN
//												DRVSYS_GPIOA_MF4_SPI0_DATA_OUT
//												DRVSYS_GPIOA_MF5_TMR0_COUNTER_EXT_IN
//												DRVSYS_GPIOA_MF6_EXT_INTERRUPT_IN
//												DRVSYS_GPIOA_MF7_ADC_EXT_TG_IN
//												DRVSYS_GPIOA_MF8_ADC_CHANNEL0_IN
//												DRVSYS_GPIOA_MF9_ADC_CHANNEL1_IN
//												DRVSYS_GPIOA_MF10_ADC_CHANNEL2_IN
//												DRVSYS_GPIOA_MF11_ADC_CHANNEL3_IN
//												DRVSYS_GPIOA_MF12_ADC_CHANNEL4_IN
//												DRVSYS_GPIOA_MF13_ADC_CHANNEL5_IN
//												DRVSYS_GPIOA_MF14_ADC_CHANNEL6_IN
//												DRVSYS_GPIOA_MF15_ADC_CHANNEL7_IN
//
// Description:
//              GPIOA multifunction pin selection.
//				If it's not selected as multifunction pin, it will be GPIO pin.
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvSYS_EnableMultifunctionGpioa(
	UINT16 u16Multifunction
)
{
	SYS.GPA_MFP.u32Reg |= u16Multifunction;
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvSYS_EnableMultifunctionGpiob
//
// Parameters:
//				u16Multifunction	[in]	Bitwise or to select GPIOB multifunction selection
//												DRVSYS_GPIOB_MF0_SPI1_2ND_CHIP_SEL_OUT
//												DRVSYS_GPIOB_MF1_SPI1_1ST_CHIP_SEL_OUT
//												DRVSYS_GPIOB_MF2_SPI1_CLOCK_OUT
//												DRVSYS_GPIOB_MF3_SPI1_DATA_IN
//												DRVSYS_GPIOB_MF4_SPI1_DATA_OUT
//												DRVSYS_GPIOB_MF8_PWM_OUT_0
//												DRVSYS_GPIOB_MF9_PWM_OUT_1
//												DRVSYS_GPIOB_MF10_PWM_OUT_2
//												DRVSYS_GPIOB_MF11_PWM_OUT_3
//												DRVSYS_GPIOB_MF12_PWM_TMR_CAPTURE_IN
//												DRVSYS_GPIOB_MF13_IR_CARRIER_OUT
//												DRVSYS_GPIOB_MF14_TMR1_COUNTER_EXT_IN
//												DRVSYS_GPIOB_MF15_TMR2_COUNTER_EXT_IN
//
// Description:
//              GPIOB multifunction pin selection.
//				If it's not selected as multifunction pin, it will be GPIO pin.
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvSYS_EnableMultifunctionGpiob(
	UINT16 u16Multifunction
)
{
	SYS.GPB_MFP.u32Reg |= u16Multifunction;
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvSYS_EnableSchmittTriggerGpioa
//
// Parameters:
//				u16EnableIO			[in]	Bitwise or to enable GPIOA I/O cell input schmitt triggle function
// 												DRVSYS_GPIO_SCHMITT_TG_0
// 												DRVSYS_GPIO_SCHMITT_TG_1
// 												DRVSYS_GPIO_SCHMITT_TG_2
// 												DRVSYS_GPIO_SCHMITT_TG_3
// 												DRVSYS_GPIO_SCHMITT_TG_4
// 												DRVSYS_GPIO_SCHMITT_TG_5
// 												DRVSYS_GPIO_SCHMITT_TG_6
// 												DRVSYS_GPIO_SCHMITT_TG_7
// 												DRVSYS_GPIO_SCHMITT_TG_8
// 												DRVSYS_GPIO_SCHMITT_TG_9
// 												DRVSYS_GPIO_SCHMITT_TG_10
// 												DRVSYS_GPIO_SCHMITT_TG_11
// 												DRVSYS_GPIO_SCHMITT_TG_12
// 												DRVSYS_GPIO_SCHMITT_TG_13
// 												DRVSYS_GPIO_SCHMITT_TG_14
// 												DRVSYS_GPIO_SCHMITT_TG_15
//
// Description:
//              Enable GPIOA I/O cell input schmitt triggle function
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvSYS_EnableSchmittTriggerGpioa(
	UINT32 u32EnableIO
)
{
	SYS.GPA_MFP.u32Reg |= u32EnableIO;
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvSYS_EnableVoltageDetector_P
//
// Parameters:
//				u8VoltageDetectorThreshold	[in] Voltage detector threshold. 1: 3.0V, 0:2.7V
// Returns:
//               None
//
// Description:
//               Enable voltage detector.
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvSYS_EnableVoltageDetector_P(
	UINT8 u8VoltageDetectorThreshold)
{
	SYS.VDCR.VD_VL = u8VoltageDetectorThreshold;
	SYS.VDCR.VD_EN = 1;
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvSYS_GpiobEnableSchmittTrigger
//
// Parameters:
//				u16EnableIO			[in]	Bitwise or to enable GPIOB I/O cell input schmitt triggle function
// 												DRVSYS_GPIO_SCHMITT_TG_0
// 												DRVSYS_GPIO_SCHMITT_TG_1
// 												DRVSYS_GPIO_SCHMITT_TG_2
// 												DRVSYS_GPIO_SCHMITT_TG_3
// 												DRVSYS_GPIO_SCHMITT_TG_4
// 												DRVSYS_GPIO_SCHMITT_TG_5
// 												DRVSYS_GPIO_SCHMITT_TG_6
// 												DRVSYS_GPIO_SCHMITT_TG_7
// 												DRVSYS_GPIO_SCHMITT_TG_8
// 												DRVSYS_GPIO_SCHMITT_TG_9
// 												DRVSYS_GPIO_SCHMITT_TG_10
// 												DRVSYS_GPIO_SCHMITT_TG_11
// 												DRVSYS_GPIO_SCHMITT_TG_12
// 												DRVSYS_GPIO_SCHMITT_TG_13
// 												DRVSYS_GPIO_SCHMITT_TG_14
// 												DRVSYS_GPIO_SCHMITT_TG_15
//
// Description:
//              Enable GPIOB I/O cell input schmitt triggle function
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvSYS_EnableSchmittTriggerGpiob(
	UINT32 u32EnableIO
)
{
	SYS.GPB_MFP.u32Reg |= u32EnableIO;
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvSYS_GetRstSrc
//
// Returns:
//               reset source that is bitwise or of following bits.
//					DRVSYS_RST_SRC_POR
//					DRVSYS_RST_SRC_PAD
//					DRVSYS_RST_SRC_WDT
//					DRVSYS_RST_SRC_LVR
//					DRVSYS_RST_SRC_MCU	, N572F072 removed this option
//					DRVSYS_RST_SRC_PMU
//
// Description:
//               Get reset source from last operation
//---------------------------------------------------------------------------------------------------------
static __inline
UINT8
DrvSYS_GetRstSrc(void)
{
	return (UINT8)SYS.RSTSRC.u32Reg;
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvSYS_GetVersion
//
// Returns:
//               The DrvSYS_ version number
//
// Description:
//               This function is used to get the DrvSYS_ version number
//---------------------------------------------------------------------------------------------------------
UINT32
DrvSYS_GetVersion(void);

//---------------------------------------------------------------------------------------------------------
// Function: DrvSYS_GetVoltageDetectorOutput
//
// Returns:
//               1: lower than threshold
//               0: higher than threshold.
//
// Description:
//               Get voltage detector result is higher or lower threshold.
//---------------------------------------------------------------------------------------------------------
static __inline
UINT8
DrvSYS_GetVoltageDetectorOutput(void)
{
	return (UINT8) SYS.VDCR.VD_OUT;
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvSYS_UnLockKeyReg
//
// Parameters:
//				None
//
// Returns:
//				None
// Description:
//				Unlock protected registers
//---------------------------------------------------------------------------------------------------------
void DrvSYS_UnlockKeyReg(void);

//---------------------------------------------------------------------------------------------------------
// Function: DrvSYS_LockKeyReg
//
// Returns:
//
// Description:
//               Lock protected registers
//---------------------------------------------------------------------------------------------------------
void DrvSYS_LockKeyReg(void);

//---------------------------------------------------------------------------------------------------------
// Function: DrvSYS_ReadProductID
//
// Returns:
//               Product ID
//
// Description:
//               Read Product ID
//---------------------------------------------------------------------------------------------------------
static __inline
UINT32
DrvSYS_ReadProductID(void)
{
	return SYS.PDID;
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvSYS_ResetChip_P
//
// Parameters:
//               None
//
// Returns:
//               None
//
// Description:
//               Reset whole chip
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvSYS_ResetChip_P(void)
{
	SYS.IPRSTC1.CHIP_RST = 1;
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvSYS_ResetCPU_P
//
// Parameters:
//               None
//
// Returns:
//               None
//
// Description:
//               Reset CPU
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvSYS_ResetCPU_P(void)
{
	SYS.IPRSTC1.CPU_RST = 1;
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvSYS_ResetIP
//
// Parameters:
//               eIpRst  - 			[in] bitwise or to reset IPs
//											DRVSYS_RST_GPIO
//											DRVSYS_RST_TMR0
//											DRVSYS_RST_TMR1
//											DRVSYS_RST_TMR2
//											DRVSYS_RST_APU
//											DRVSYS_RST_TMRF
//											DRVSYS_RST_SPI0
//											DRVSYS_RST_SPI1
//											DRVSYS_RST_PWM
//											DRVSYS_RST_USBD	 , N572F072 removed USB device
//											DRVSYS_RST_ADC
//
// Description:
//               Reset corresponding IPs
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvSYS_ResetIP(
	UINT32 u32IpRst
)
{
	SYS.IPRSTC2.u32Reg =  u32IpRst;	// reset IPs
	SYS.IPRSTC2.u32Reg = 0;			// set "0" to release from reset state
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvSYS_SetFlashMemoryWaitState_P
//
// Parameters:
//               u8WaitState		[in] wait state setting.
//								   			DRVSYS_FLASH_MEMORY_WAIT_STATE_1_HCLK
//								   			DRVSYS_FLASH_MEMORY_WAIT_STATE_0_HCLK
//
// Returns:
//               None
//
// Description:
//               Set flash memory wait state.
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvSYS_SetFlashMemoryWaitState_P(
	UINT8 u8WaitState)
{
	SYS.IPRSTC1.CPUWS = u8WaitState;
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvSYS_SetPORDisableCode_P
//
// Parameters:
//
// Returns:
//               None
//
// Description:
//               Set POR disable code to diable power on reset for prevent reset by power noise.
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvSYS_SetPORDisableCode_P(void)
{
	SYS.PORCR.u32Reg = 0x5aa5;
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvSYS_SetRamWaitState_P
//
// Parameters:
//               u8WaitState		[in] wait state setting.
//								   			DRVSYS_RAM_WAIT_STATE_1_HCLK
//								   			DRVSYS_RAM_WAIT_STATE_0_HCLK
//
// Returns:
//               None
//
// Description:
//               Ser RAM wait state.
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvSYS_SetRAMWaitState_P(
	UINT8 u8WaitState)
{
	SYS.IPRSTC1.RAMWS = u8WaitState;
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvSYS_LockKeyReg
//
// Returns:
//
// Description:
//               Lock protected registers
//---------------------------------------------------------------------------------------------------------
static __inline
void 
DrvSYS_LockKeyReg(void)
{
	SYS.REGLOCKADDR.u32Reg = 0x00;
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvSYS_IsUnLockKeyReg
//
// Returns:
//
// Description:
//               Check key register whether lock
//---------------------------------------------------------------------------------------------------------
static __inline
BOOL
DrvSYS_IsUnLockKeyReg(void)
{
	return (SYS.REGLOCKADDR.REGUNLOCK);
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvSYS_ReadROMMAPData
//
// Parameters: 	 u8Sel				[in] Select ROMMAP, 0:ROMMAP0;1:ROMMAP1.
//
// Returns:
//				 Image data of ROMMAP according to u8Sel selection.
// Description:
//               Read ROMMAP image data.
//---------------------------------------------------------------------------------------------------------
static __inline
UINT32
DrvSYS_ReadROMMAPData(
	UINT8	u8Sel
)
{
   return *((PUINT32)((UINT32)&SYS.IMGMAP0+u8Sel*4));
}

#if (defined (__N572F072__) || defined (__N572P072__))
//---------------------------------------------------------------------------------------------------------
// Function: DrvSYS_EnableGPIOHSTransition
//
// Parameters: 	 
//
// Returns:
//				 
// Description:
//               Enable GPIOA pin0~pin4 high speed transition for SPI0 up to 48MHz. This API only support
//				 N572F072.
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvSYS_EnableGPIOHSTransition(void)
{
   SYS.GPIO_HS.u32Reg = 0x1f;
}
//---------------------------------------------------------------------------------------------------------
// Function: DrvSYS_DisableGPIOHSTransition
//
// Parameters: 	 
//
// Returns:
//				 
// Description:
//               GPIOA pin0~pin4 normal speed transition for SPI0 less than 15MHz. This API only support
//				 N572F072.
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvSYS_DisableGPIOHSTransition(void)
{
   SYS.GPIO_HS.u32Reg = 0x0;
}
#endif

#endif

#ifdef  __cplusplus
}
#endif

