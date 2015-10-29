/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2010 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#if (defined(__N572F064__)||defined(__N572F065__))
#include "Driver/DrvSPI.h"
//--------------------------------------------------------------------------------------------------------------
// Function:		DrvSPI_Open
//
// Parameters:	u32SpiHandler   [in]:  set SPI chaneel
//																			DRVSPI_SPI0_HANDLER: SPI0 base. DRVSPI_SPI1_HANDLER: SPI1 base.
//
// Description:	Configure the SPI port.
//--------------------------------------------------------------------------------------------------------------
void
DrvSPI_Open(
	UINT32 u32SpiHandler,
	UINT32 u32OpenFlag,
	UINT16 u16Divider
)
{
	UINT32 u32SysClkID;

	if( u32SpiHandler == DRVSPI_SPI0_HANDLER )
		u32SysClkID = DRVCLK_CLK_SPI0;
	else
		u32SysClkID = DRVCLK_CLK_SPI1;
	DrvCLK_EnableApbDeviceClock(u32SysClkID);
	DrvSYS_ResetIP(u32SysClkID);
	((SPI_T*)u32SpiHandler)->CNTRL.u32Reg = u32OpenFlag|SPI_CNTRL_IF;		// Clear interrupt flag
	((SPI_T*)u32SpiHandler)->DIVIDER.u32Reg = u16Divider;
}

void
DrvSPI_Close(
	UINT32 u32Handler
)
{
	UINT32 u32SysClkID;
	IRQn_Type eNvicID;

	if( u32Handler == SPI0_BASE )
	{
		u32SysClkID = DRVCLK_CLK_SPI0;
		eNvicID = SPI0_IRQn;
	}
	else
	{
		u32SysClkID = DRVCLK_CLK_SPI1;
		eNvicID = SPI1_IRQn;
	}
	NVIC_DisableIRQ(eNvicID);
	DrvCLK_DisableApbDeviceClock(u32SysClkID);
}

//-------------------------------------------------------------------------------------------------------------------
// Function:		DrvSPI_EnableInt
//
// Description:	Enable the SPI interrupt.
//-------------------------------------------------------------------------------------------------------------------
void
DrvSPI_EnableInt(
	UINT32 u32SpiHandler
)
{
	IRQn_Type eIRQ;

	((SPI_T*)u32SpiHandler)->CNTRL.u32Reg = (((SPI_T*)u32SpiHandler)->CNTRL.u32Reg & (~SPI_CNTRL_IF)) | SPI_CNTRL_IE;
	eIRQ = (u32SpiHandler==SPI0_BASE)?SPI0_IRQn:SPI1_IRQn;
	NVIC_EnableIRQ(eIRQ);
}

//-------------------------------------------------------------------------------------------------------------------
// Function:		DrvSPI_DisableInt
//
// Description:	Disable the SPI interrupt.
//-------------------------------------------------------------------------------------------------------------------
void
DrvSPI_DisableInt(
	UINT32 u32SpiHandler
)
{
	IRQn_Type eIRQ;

	((SPI_T*)u32SpiHandler)->CNTRL.u32Reg = ((SPI_T*)u32SpiHandler)->CNTRL.u32Reg & (~(SPI_CNTRL_IF|SPI_CNTRL_IE));
	eIRQ = (u32SpiHandler==SPI0_BASE)?SPI0_IRQn:SPI1_IRQn;
	NVIC_DisableIRQ(eIRQ);
}

//-------------------------------------------------------------------------------------------------------------------
// Function:		DrvSPI_GetVersion
//
// Returns:			Version number.
//
// Description:	Get the version number of SPI driver.
//-------------------------------------------------------------------------------------------------------------------
UINT32
DrvSPI_GetVersion()
{
	return DRVSPI_VERSION_NUM;
}
#endif // (defined(__N572F064__)||defined(__N572F065__))
