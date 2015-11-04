/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2010 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#ifndef __DRVGPIO_H__
#define __DRVGPIO_H__

// Include header file

#include "Platform.h"
#include "System/SysInfra.h"

//---------------------------------------------------------------------------------------------------------
//  Define Version number
//---------------------------------------------------------------------------------------------------------
#define DRVGPIO_MAJOR_NUM	3
#define DRVGPIO_MINOR_NUM	30
#define DRVGPIO_BUILD_NUM	1

//--------------------------------------------------------------------------------------------------------
//  Version define with SysInfra
//--------------------------------------------------------------------------------------------------------
#define DRVGPIO_VERSION_NUM     _SYSINFRA_VERSION(DRVGPIO_MAJOR_NUM, DRVGPIO_MINOR_NUM, DRVGPIO_BUILD_NUM)

// #define Constant

// define IO mode for all pins
#define DRVGPIO_IOMODE_PIN0_IN				0x0
#define DRVGPIO_IOMODE_PIN0_OUT				0x1
#define DRVGPIO_IOMODE_PIN0_OPEN_DRAIN		0x2
#define DRVGPIO_IOMODE_PIN0_QUASI			0x3

#define DRVGPIO_IOMODE_PIN1_IN				0x0
#define DRVGPIO_IOMODE_PIN1_OUT				0x4
#define DRVGPIO_IOMODE_PIN1_OPEN_DRAIN		0x8
#define DRVGPIO_IOMODE_PIN1_QUASI			0xC

#define DRVGPIO_IOMODE_PIN2_IN				0x00
#define DRVGPIO_IOMODE_PIN2_OUT				0x10
#define DRVGPIO_IOMODE_PIN2_OPEN_DRAIN		0x20
#define DRVGPIO_IOMODE_PIN2_QUASI			0x30

#define DRVGPIO_IOMODE_PIN3_IN				0x00
#define DRVGPIO_IOMODE_PIN3_OUT				0x40
#define DRVGPIO_IOMODE_PIN3_OPEN_DRAIN		0x80
#define DRVGPIO_IOMODE_PIN3_QUASI			0xC0

#define DRVGPIO_IOMODE_PIN4_IN				0x000
#define DRVGPIO_IOMODE_PIN4_OUT				0x100
#define DRVGPIO_IOMODE_PIN4_OPEN_DRAIN		0x200
#define DRVGPIO_IOMODE_PIN4_QUASI			0x300

#define DRVGPIO_IOMODE_PIN5_IN				0x000
#define DRVGPIO_IOMODE_PIN5_OUT				0x400
#define DRVGPIO_IOMODE_PIN5_OPEN_DRAIN		0x800
#define DRVGPIO_IOMODE_PIN5_QUASI			0xC00

#define DRVGPIO_IOMODE_PIN6_IN				0x0000
#define DRVGPIO_IOMODE_PIN6_OUT				0x1000
#define DRVGPIO_IOMODE_PIN6_OPEN_DRAIN		0x2000
#define DRVGPIO_IOMODE_PIN6_QUASI			0x3000

#define DRVGPIO_IOMODE_PIN7_IN				0x0000
#define DRVGPIO_IOMODE_PIN7_OUT				0x4000
#define DRVGPIO_IOMODE_PIN7_OPEN_DRAIN		0x8000
#define DRVGPIO_IOMODE_PIN7_QUASI			0xC000

#define DRVGPIO_IOMODE_PIN8_IN				0x00000
#define DRVGPIO_IOMODE_PIN8_OUT				0x10000
#define DRVGPIO_IOMODE_PIN8_OPEN_DRAIN		0x20000
#define DRVGPIO_IOMODE_PIN8_QUASI			0x30000

#define DRVGPIO_IOMODE_PIN9_IN				0x00000
#define DRVGPIO_IOMODE_PIN9_OUT				0x40000
#define DRVGPIO_IOMODE_PIN9_OPEN_DRAIN		0x80000
#define DRVGPIO_IOMODE_PIN9_QUASI			0xC0000

#define DRVGPIO_IOMODE_PIN10_IN				0x000000
#define DRVGPIO_IOMODE_PIN10_OUT			0x100000
#define DRVGPIO_IOMODE_PIN10_OPEN_DRAIN		0x200000
#define DRVGPIO_IOMODE_PIN10_QUASI			0x300000

#define DRVGPIO_IOMODE_PIN11_IN				0x000000
#define DRVGPIO_IOMODE_PIN11_OUT			0x400000
#define DRVGPIO_IOMODE_PIN11_OPEN_DRAIN		0x800000
#define DRVGPIO_IOMODE_PIN11_QUASI			0xC00000

#define DRVGPIO_IOMODE_PIN12_IN				0x0000000
#define DRVGPIO_IOMODE_PIN12_OUT			0x1000000
#define DRVGPIO_IOMODE_PIN12_OPEN_DRAIN		0x2000000
#define DRVGPIO_IOMODE_PIN12_QUASI			0x3000000

#define DRVGPIO_IOMODE_PIN13_IN				0x0000000
#define DRVGPIO_IOMODE_PIN13_OUT			0x4000000
#define DRVGPIO_IOMODE_PIN13_OPEN_DRAIN		0x8000000
#define DRVGPIO_IOMODE_PIN13_QUASI			0xC000000

#define DRVGPIO_IOMODE_PIN14_IN				0x00000000
#define DRVGPIO_IOMODE_PIN14_OUT			0x10000000
#define DRVGPIO_IOMODE_PIN14_OPEN_DRAIN		0x20000000
#define DRVGPIO_IOMODE_PIN14_QUASI			0x30000000

#define DRVGPIO_IOMODE_PIN15_IN				0x00000000
#define DRVGPIO_IOMODE_PIN15_OUT			0x40000000
#define DRVGPIO_IOMODE_PIN15_OPEN_DRAIN		0x80000000
#define DRVGPIO_IOMODE_PIN15_QUASI			0xC0000000

// define pin mask
#define DRVGPIO_PIN_0		BIT0
#define DRVGPIO_PIN_1		BIT1
#define DRVGPIO_PIN_2		BIT2
#define DRVGPIO_PIN_3		BIT3
#define DRVGPIO_PIN_4		BIT4
#define DRVGPIO_PIN_5		BIT5
#define DRVGPIO_PIN_6		BIT6
#define DRVGPIO_PIN_7		BIT7
#define DRVGPIO_PIN_8		BIT8
#define DRVGPIO_PIN_9		BIT9
#define DRVGPIO_PIN_10		BIT10
#define DRVGPIO_PIN_11		BIT11
#define DRVGPIO_PIN_12		BIT12
#define DRVGPIO_PIN_13		BIT13
#define DRVGPIO_PIN_14		BIT14
#define DRVGPIO_PIN_15		BIT15


#define DrvGPIO_DisableIoDigitalInputPath(pGPIO,u32Pins)	DrvPGIO_DisableInputPin(pGPIO,u32Pins)
#define DrvGPIO_EnableIoDigitalInputPath(pGPIO,u32Pins)		DrvGPIO_EnableInputPin(pGPIO,u32Pins)
#define DrvGPIO_GetIoDigitalInputPath(pGPIO,u32Pins)		DrvGPIO_CheckDisabledInputPin(pGPIO,u32Pins) 
#define DrvGPIO_GetOutputWriteMask(pGPIO,u32Pins)			DrvGPIO_CheckOutputWriteMask(pGPIO,u32Pins)
#define DrvGPIO_GetOutputData(pGPIO,u32Pins)				DrvGPIO_GetOutputPinValue(pGPIO,u32Pins)
#define DrvGPIO_GetPinValue(pGPIO,u32Pins) 					DrvGPIO_GetInputPinValue(pGPIO,u32Pins)


// APIs declaration

UINT32
DrvGPIO_GetVersion(void);

// define inline functions

//---------------------------------------------------------------------------------------------------------
// Function:     DrvGPIO_SetIOMode
//
// Parameter:
//				 pGPIO			[in]	Set GPIO port "GPIOA" "GPIOB" address
//	             u32Mode		[in]	bitwise or to set IO mode for IO pins. 'n' is the pin number.
//										Note: One mode for one pin, and bitwise or all pins of the same GPIO.
//										ex: "DRVGPIO_IOMODE_PIN0_IN | DRVGPIO_IOMODE_PIN3_OUT | DRVGPIO_IOMODE_PIN6_QUASI
//											DRVGPIO_IOMODE_PINn_IN
//											DRVGPIO_IOMODE_PINn_OUT
//											DRVGPIO_IOMODE_PINn_OPEN_DRAIN
//											DRVGPIO_IOMODE_PINn_QUASI
// Description:
//               Set GPIO IO mode for IO pins.
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvGPIO_SetIOMode(
	GPIO_T* pGPIO,
	UINT32 u32IoMode
)
{
	pGPIO->PMD.u32Reg = u32IoMode;
}

//---------------------------------------------------------------------------------------------------------
// Function:     DrvGPIO_SetIOModeEx
//
// Parameter:
//				 pGPIO			[in]	Set GPIO port "GPIOA" "GPIOB" address
//	             u32IOMode		[in]	bitwise or to set IO mode for IO pins. 'n' is the pin number.
//										Note: One mode for one pin, and bitwise or all pins of the same GPIO.
//										ex: "DRVGPIO_IOMODE_PIN0_IN | DRVGPIO_IOMODE_PIN3_OUT | DRVGPIO_IOMODE_PIN6_QUASI
//											DRVGPIO_IOMODE_PINn_IN
//											DRVGPIO_IOMODE_PINn_OUT
//											DRVGPIO_IOMODE_PINn_OPEN_DRAIN
//											DRVGPIO_IOMODE_PINn_QUASI
//	             u16Pins		[in]	bitwise or the pins. 'n' is the pin number.
//											DRVGPIO_PIN_n
// Description:
//               Set GPIO IO mode for IO pins.
//---------------------------------------------------------------------------------------------------------
void
DrvGPIO_SetIOModeEx(
	GPIO_T* pGPIO,
	UINT32 u32IoMode,
	UINT16 u16Pins
);

//---------------------------------------------------------------------------------------------------------
// Function:     DrvPGIO_DisableInputPin
//
// Parameter:
//				 pGPIO			[in]	Set GPIO port "GPIOA" "GPIOB" address
//	             u32Pins		[in]	bitwise or the pins. 'n' is the pin number.
//											DRVGPIO_PIN_n
// Description:
//               Disable IO digital input option (digital input tied to low).
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvPGIO_DisableInputPin(
	GPIO_T* pGPIO,
	UINT32 u32Pins
)
{
	pGPIO->OFFD.u32Reg |= (u32Pins << 16);
}

//---------------------------------------------------------------------------------------------------------
// Function:     DrvGPIO_CheckDisabledInputPin
//
// Parameter:
//				 pGPIO			[in]	Set GPIO port "GPIOA" "GPIOB" address
//	             u16Pins		[in]	bitwise or the pins. 'n' is the pin number.
//											DRVGPIO_PIN_n
// Returns:
//               IO digital input option status of indicated GPIO output pins.
// Description:
//               Check IO digital input option status.
//---------------------------------------------------------------------------------------------------------
static __inline
UINT16
DrvGPIO_CheckDisabledInputPin(
	GPIO_T* pGPIO,
	UINT16 u16Pins
)
{
	return (UINT16)(pGPIO->OFFD.u32Reg & u16Pins);
}

//---------------------------------------------------------------------------------------------------------
// Function:     DrvGPIO_EnableInputPin
//
// Parameter:
//				 pGPIO			[in]	Set GPIO port "GPIOA" "GPIOB" address
//	             u32Pins		[in]	bitwise or the pins. 'n' is the pin number.
//											DRVGPIO_PIN_n
// Description:
//               Enable IO digital input option.
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvGPIO_EnableInputPin(
	GPIO_T* pGPIO,
	UINT32 u32Pins
)
{
	pGPIO->OFFD.u32Reg &= ~(u32Pins<<16);
}

//---------------------------------------------------------------------------------------------------------
// Function:     DrvGPIO_SetOutputData
//
// Parameter:
//				 pGPIO			[in]	Set GPIO port "GPIOA" "GPIOB" address
//	             u16Data		[in]	16 bits data to coresponding output pins
// Description:
//               Set output data of output pins of GPIO.
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvGPIO_SetOutputData(
	GPIO_T* pGPIO,
	UINT16 u16Data
)
{
	pGPIO->DOUT.u32Reg = u16Data;
}

//---------------------------------------------------------------------------------------------------------
// Function:     DrvGPIO_SetOutputDataEx
//
// Parameter:
//				 pGPIO			[in]	Set GPIO port "GPIOA" "GPIOB" address
//	             u16Data		[in]	16 bits data to coresponding output pins,output low and nonchanged is 0, output high is 1.
//				 u16Pins		[in]	bitwise or the pins. 'n' is the pin number.
//											DRVGPIO_PIN_n
// Description:
//               Set output data of output pins of GPIO.
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvGPIO_SetOutputDataEx(
	GPIO_T* pGPIO,
	UINT16 u16Data,
	UINT16 u16Pins
)
{
	pGPIO->DOUT.u32Reg = (pGPIO->DOUT.u32Reg&(~u16Pins))|(u16Data&u16Pins);
}
//---------------------------------------------------------------------------------------------------------
// Function:     DrvGPIO_SetOutputBit
//
// Parameter:
//				 pGPIO			[in]	Set GPIO port "GPIOA" "GPIOB" address
//	             u16Pins		[in]	bitwise or the pins. 'n' is the pin number.
//											DRVGPIO_PIN_n
// Description:
//               Set output bits of output pins of GPIO.
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvGPIO_SetOutputBit(
	GPIO_T* pGPIO,
	UINT16 u16Pins
)
{
	pGPIO->DOUT.u32Reg |= u16Pins;
}

//---------------------------------------------------------------------------------------------------------
// Function:     void DrvGPIO_ClearOutputBit
//
// Parameter:
//				 pGPIO			[in]	Set GPIO port "GPIOA" "GPIOB" address
//	             u16Pins		[in]	bitwise or the pins. 'n' is the pin number.
//											DRVGPIO_PIN_n
// Description:
//               Clear output bits of output pins of GPIO.
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvGPIO_ClearOutputBit(
	GPIO_T* pGPIO,
	UINT16 u16Pins
)
{
	pGPIO->DOUT.u32Reg &= ~u16Pins;
}

//---------------------------------------------------------------------------------------------------------
// Function:     DrvGPIO_GetOutputPinValue
//
// Parameter:
//				 pGPIO			[in]	Set GPIO port "GPIOA" "GPIOB" address
//	             u16Pins		[in]	bitwise or the pins. 'n' is the pin number.
//											DRVGPIO_PIN_n
// Returns:
//               Output value of indicated GPIO output pins.
// Description:
//               Get output value of indicated output pins of GPIO.
//---------------------------------------------------------------------------------------------------------
static __inline
UINT16
DrvGPIO_GetOutputPinValue(
	GPIO_T* pGPIO,
	UINT16 u16Pins
)
{
	return (UINT16)(pGPIO->DOUT.u32Reg & u16Pins);
}

//---------------------------------------------------------------------------------------------------------
// Function:     DrvGPIO_SetOutputWriteMask
//
// Parameter:
//				 pGPIO			[in]	Set GPIO port "GPIOA" "GPIOB" address
//	             u16Pins		[in]	bitwise or the pins. 'n' is the pin number.
//											DRVGPIO_PIN_n
// Description:
//               Set write mask to indicated output pins of GPIO.
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvGPIO_SetOutputWriteMask(
	GPIO_T* pGPIO,
	UINT16 u16Pins
)
{
	pGPIO->DMASK.u32Reg |= u16Pins;
}

//---------------------------------------------------------------------------------------------------------
// Function:     DrvGPIO_CheckOutputWriteMask
//
// Parameter:
//				 pGPIO			[in]	Set GPIO port "GPIOA" "GPIOB" address
//	             u16Pins		[in]	bitwise or the pins. 'n' is the pin number.
//											DRVGPIO_PIN_n
// Returns:
//               write mask status of indicated GPIO output pins.
// Description:
//               Check write mask status of indicated output pins of GPIO.
//---------------------------------------------------------------------------------------------------------
static __inline
UINT16
DrvGPIO_CheckOutputWriteMask(
	GPIO_T* pGPIO,
	UINT16 u16Pins
)
{
	return (UINT16)(pGPIO->DMASK.u32Reg & u16Pins);
}

//---------------------------------------------------------------------------------------------------------
// Function:     DrvGPIO_ClearOutputWriteMask
//
// Parameter:
//				 pGPIO			[in]	Set GPIO port "GPIOA" "GPIOB" address
//	             u16Pins		[in]	bitwise or the pins. 'n' is the pin number.
//											DRVGPIO_PIN_n
// Description:
//               Clear write mask to indicated output pins of GPIO.
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvGPIO_ClearOutputWriteMask(
	GPIO_T* pGPIO,
	UINT16 u16Pins
)
{
	pGPIO->DMASK.u32Reg &= ~u16Pins;
}

//---------------------------------------------------------------------------------------------------------
// Function:     DrvGPIO_GetInputPinValue
//
// Parameter:
//				 pGPIO			[in]	Set GPIO port "GPIOA" "GPIOB" address
//	             u16Pins		[in]	bitwise or the pins. 'n' is the pin number.
//											DRVGPIO_PIN_n
// Returns:
//               GPIO actual pin value of the indicated pins.
/// Description:
//               Get GPIO actual pin value of the indicated pins.
//---------------------------------------------------------------------------------------------------------
static __inline
UINT16
DrvGPIO_GetInputPinValue(
	GPIO_T* pGPIO,
	UINT16 u16Pins
)
{
	return (UINT16)(pGPIO->PIN.u32Reg & u16Pins);
}

//---------------------------------------------------------------------------------------------------------
// Function:     DrvGPIO_SetIntModeEdgeTrigger
//
// Parameter:
//				 pGPIO			[in]	Set GPIO port "GPIOA" "GPIOB" address
//	             u16Pins		[in]	bitwise or the pins. 'n' is the pin number.
//											DRVGPIO_PIN_n
// Description:
//               Set edge trigger interrupt mode to indicated GPIO pins.
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvGPIO_SetIntModeEdgeTrigger(
	GPIO_T* pGPIO,
	UINT16 u16Pins
)
{
	pGPIO->IMD.u32Reg &= ~u16Pins;
}

//---------------------------------------------------------------------------------------------------------
// Function:     DrvGPIO_SetIntModeLevelTrigger
//
// Parameter:
//				 pGPIO			[in]	Set GPIO port "GPIOA" "GPIOB" address
//	             u16Pins		[in]	bitwise or the pins. 'n' is the pin number.
//											DRVGPIO_PIN_n
// Description:
//               Set level trigger interrupt mode to indicated GPIO pins.
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvGPIO_SetIntModeLevelTrigger(
	GPIO_T* pGPIO,
	UINT16 u16Pins
)
{
	pGPIO->IMD.u32Reg |= u16Pins;
}

//---------------------------------------------------------------------------------------------------------
// Function:     DrvGPIO_EnableRisingHighInt
//
// Parameter:
//				 pGPIO			[in]	Set GPIO port "GPIOA" "GPIOB" address
//	             u16Pins		[in]	bitwise or the pins. 'n' is the pin number.
//											DRVGPIO_PIN_n
// Description:
//               Enable rising or high level interrupt to indicated GPIO pins.
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvGPIO_EnableRisingHighInt(
	GPIO_T* pGPIO,
	UINT16 u16Pins)
{
	pGPIO->IEN.u32Reg |= ((UINT32)u16Pins << 16);
}

//---------------------------------------------------------------------------------------------------------
// Function:     DrvGPIO_DisableRisingHighInt
//
// Parameter:
//				 pGPIO			[in]	Set GPIO port "GPIOA" "GPIOB" address
//	             u16Pins		[in]	bitwise or the pins. 'n' is the pin number.
//											DRVGPIO_PIN_n
// Returns:
//               None
// Description:
//               Disable rising or high level interrupt to indicated GPIO pins.
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvGPIO_DisableRisingHighInt(
	GPIO_T* pGPIO,
	UINT16 u16Pins
)
{
	pGPIO->IEN.u32Reg &= ~((UINT32)u16Pins << 16);
}

//---------------------------------------------------------------------------------------------------------
// Function:     DrvGPIO_EnableFallingLowInt
//
// Parameter:
//				 pGPIO			[in]	Set GPIO port "GPIOA" "GPIOB" address
//	             u16Pins		[in]	bitwise or the pins. 'n' is the pin number.
//											DRVGPIO_PIN_n
// Description:
//               Enable falling or low level interrupt to indicated GPIO pins.
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvGPIO_EnableFallingLowInt(
	GPIO_T* pGPIO,
	UINT16 u16Pins
)
{
	pGPIO->IEN.u32Reg |= ((UINT32)u16Pins);
}

//---------------------------------------------------------------------------------------------------------
// Function:     DrvGPIO_DisableFallingLowInt
//
// Parameter:
//				 pGPIO			[in]	Set GPIO port "GPIOA" "GPIOB" address
//	             u16Pins		[in]	bitwise or the pins. 'n' is the pin number.
//											DRVGPIO_PIN_n
// Description:
//               Disable falling or low level interrupt to indicated GPIO pins.
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvGPIO_DisableFallingLowInt(
	GPIO_T* pGPIO,
	UINT16 u16Pins
)
{
	pGPIO->IEN.u32Reg &= ~((UINT32)u16Pins);
}

//---------------------------------------------------------------------------------------------------------
// Function:     DrvGPIO_SetRisingInt
//
// Parameter:
//				 pGPIO			[in]	Set GPIO port "GPIOA" "GPIOB" address
//	             u16Pins		[in]	bitwise or the pins. 'n' is the pin number.
//											DRVGPIO_PIN_n
// Description:
//               Set rising edge trigger interrupt mode to indicate GPIO pins.
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvGPIO_SetRisingInt(
	GPIO_T* pGPIO,
	UINT16 u16Pins,
	BOOL bEnable
)
{
	pGPIO->IMD.u32Reg &= ~u16Pins;

	if(bEnable)
		pGPIO->IEN.u32Reg |= ((UINT32)u16Pins << 16);
	else
		pGPIO->IEN.u32Reg &= ~((UINT32)u16Pins << 16);
}

//---------------------------------------------------------------------------------------------------------
// Function:     DrvGPIO_SetFallingInt
//
// Parameter:
//				 pGPIO			[in]	Set GPIO port "GPIOA" "GPIOB" address
//	             u16Pins		[in]	bitwise or the pins. 'n' is the pin number.
//											DRVGPIO_PIN_n
// Description:
//               Set falling edge trigger interrupt mode to indicate GPIO pins.
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvGPIO_SetFallingInt(
	GPIO_T* pGPIO,
	UINT16 u16Pins,
	BOOL bEnable
)
{
	pGPIO->IMD.u32Reg &= ~u16Pins;

	if(bEnable)
		pGPIO->IEN.u32Reg |= ((UINT32)u16Pins);
	else
		pGPIO->IEN.u32Reg &= ~((UINT32)u16Pins);
}

//---------------------------------------------------------------------------------------------------------
// Function:     DrvGPIO_SetHighInt
//
// Parameter:
//				 pGPIO			[in]	Set GPIO port "GPIOA" "GPIOB" address
//	             u16Pins		[in]	bitwise or the pins. 'n' is the pin number.
//											DRVGPIO_PIN_n
// Description:
//               Set high level trigger interrupt mode to indicate GPIO pins.
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvGPIO_SetHighInt(
	GPIO_T* pGPIO,
	UINT16 u16Pins,
	BOOL bEnable
)
{
	if(bEnable)
	{
		pGPIO->IMD.u32Reg |= u16Pins;
		pGPIO->IEN.u32Reg |= ((UINT32)u16Pins << 16);
	}
	else
	{
		pGPIO->IEN.u32Reg &= ~((UINT32)u16Pins << 16);
		pGPIO->IMD.u32Reg &= ~u16Pins;
	}
}

//---------------------------------------------------------------------------------------------------------
// Function:     DrvGPIO_SetLowInt
//
// Parameter:
//				 pGPIO			[in]	Set GPIO port "GPIOA" "GPIOB" address
//	             u16Pins		[in]	bitwise or the pins. 'n' is the pin number.
//											DRVGPIO_PIN_n
// Description:
//               Set low level trigger interrupt mode to indicate GPIO pins.
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvGPIO_SetLowInt(
	GPIO_T* pGPIO,
	UINT16 u16Pins,
	BOOL bEnable
)
{
	if(bEnable)
	{
		pGPIO->IMD.u32Reg |= u16Pins;
		pGPIO->IEN.u32Reg |= ((UINT32)u16Pins);
	}
	else
	{
		pGPIO->IEN.u32Reg &= ~((UINT32)u16Pins);
		pGPIO->IMD.u32Reg &= ~u16Pins;
	}
}

//---------------------------------------------------------------------------------------------------------
// Function:     DrvGPIO_GetIntFlag
//
// Parameter:
//				 pGPIO			[in]	Set GPIO port "GPIOA" "GPIOB" address
//	             u16Pins		[in]	bitwise or the pins. 'n' is the pin number.
//											DRVGPIO_PIN_n
// Returns:
//               bitwise or of pending interrupt trigger source of indicated pins.
// Description:
//               Get interrupt trigger source of indicated GPIO pins.
//---------------------------------------------------------------------------------------------------------
static __inline
UINT16
DrvGPIO_GetIntFlag(
	GPIO_T* pGPIO,
	UINT16 u16Pins
)
{
	return (UINT16)(pGPIO->ISRC.u32Reg & u16Pins);
}

//---------------------------------------------------------------------------------------------------------
// Function:     DrvGPIO_ClearIntFlag
//
// Parameter:
//				 pGPIO			[in]	Set GPIO port "GPIOA" "GPIOB" address
//	             u16Pins		[in]	bitwise or the pins. 'n' is the pin number.
//											DRVGPIO_PIN_n
// Description:
//               Clear interrupt flag of indicated pins of GPIO.
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvGPIO_ClearIntFlag(
	GPIO_T* pGPIO,
	UINT16 u16Pins
)
{
	pGPIO->ISRC.u32Reg = u16Pins;	// write 1: to clear, 0: no action
}

//---------------------------------------------------------------------------------------------------------
// Function:     DrvGPIO_EnableQuasiPins
//
// Parameter:
//				 pGPIO			[in]	Set GPIO port "GPIOA" "GPIOB" address
//	             u32Pins		[in]	bitwise or the pins. 'n' is the pin number.
//											DRVGPIO_PIN_n
// Description:
//               Enable quasi-input pins which are configured by calling DrvGPIO_SetIOModeEx() or DrvGPIO_SetIOMode().
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvGPIO_EnableQuasiPins(GPIO_T* pGPIO, UINT32 u32Pins)
{
	pGPIO->DOUT.u32Reg |= u32Pins;
}

#endif	// __DRVGPIO_H__
