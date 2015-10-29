/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2010 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#include "Driver/DrvGPIO.h"

//---------------------------------------------------------------------------------------------------------
// Function:		DrvGPIO_GetVersion
//
// Description:	This function is used to return the version number of GPIO driver.
//---------------------------------------------------------------------------------------------------------
UINT32
DrvGPIO_GetVersion()
{
   return DRVGPIO_VERSION_NUM;
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
)
{
	register UINT32 u32PinsMask = 0, u32PinMask = 3;

    while(u16Pins)
    {
        if ( u16Pins&1 )
            u32PinsMask |= u32PinMask;
        u16Pins >>= 1;
		u32PinMask <<= 2;
    }
    pGPIO->PMD.u32Reg = (pGPIO->PMD.u32Reg&(~u32PinsMask))|(u32IoMode&u32PinsMask);
}
