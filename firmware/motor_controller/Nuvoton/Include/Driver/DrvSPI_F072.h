/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#ifndef __DRVSPI_H__
#define __DRVSPI_H__

#include "Platform.h"
#include "System/SysInfra.h"
#include "Driver/DrvCLK.h"
#include "Driver/DrvSYS.h"

#ifdef  __cplusplus
extern "C"
{
#endif

//---------------------------------------------------------------------------------------------------------
// Macro, type and constant definitions
//---------------------------------------------------------------------------------------------------------
#define DRVSPI_MAJOR_NUM 3
#define DRVSPI_MINOR_NUM 0
#define DRVSPI_BUILD_NUM 0

#define DRVSPI_VERSION_NUM    _SYSINFRA_VERSION(DRVSPI_MAJOR_NUM,DRVSPI_MINOR_NUM,DRVSPI_BUILD_NUM)

//---------------------------------------------------------------------------------------------------------
// SPI Setting
//---------------------------------------------------------------------------------------------------------
#define DRVSPI_SPI0_HANDLER		SPI0_BASE
#define DRVSPI_SPI1_HANDLER		SPI1_BASE

#define _DRVSPI_DIVIDER(AhbClk, SpiClk)		(((AhbClk)/(SpiClk))?(((AhbClk)/(SpiClk))-1):0)
//#define _DRVSPI_SPI1_DIVIDER(AhbClk, SpiClk)		(((AhbClk)/(SpiClk*2))?(((AhbClk)/(SpiClk*2))-1):0)

#define _DRVSPI_SLEEP(s)		((((s)-2)&0xf)<<12)			// s = 2~17 sleep cycles between 2 transfer
#define _DRVSPI_DATA_BITS(b)	(((b)&0x1f)<<3)				// b = 1~32 bits for each transfer

#define DRVSPI_ENDIAN_BIG		0							// Transfer/Receive word(or half word) in big endian
#define DRVSPI_ENDIAN_LITTLE	SPI_CNTRL_BYTE_ENDIAN		// Transfer/Receive word(or half word) in little endian
#define DRVSPI_IDEL_CLK_HIGH	SPI_CNTRL_CLKP				// The CLK pin keep high when idel
#define DRVSPI_IDEL_CLK_LOW		0							// The CLK pin keep low  when idel
#define DRVSPI_LSB_FISRT		SPI_CNTRL_LSB				// Transfer/Receive from least signification bit
#define DRVSPI_MSB_FIRST		0							// Transfer/Receive from most signification bit
#define DRVSPI_TX_1DATA			0							// Transfer/Receive 1 data each time
#define DRVSPI_TX_2DATA			(1<<8)						// Transfer/Receive 2 data each time
#define DRVSPI_TX_NEGEDGE		SPI_CNTRL_TX_NEG			// Transfer 1 bit on the falling edge of SPI clock
#define DRVSPI_TX_POSEDGE		0							// Transfer 1 bit on the rising  edge of SPI clock
#define DRVSPI_RX_NEGEDGE		SPI_CNTRL_RX_NEG			// Receive 1 bit on the falling  edge of SPI clock
#define DRVSPI_RX_POSEDGE		0							// Receive 1 bit on the rising   edge of SPI clock
#define DRVSPI_SPI0_CTIM0		0				// The SPI0 Coarse timing no delayed
#define DRVSPI_SPI0_CTIM1		0x3			// The SPI0 Coarse timing delayed 1 half SPI0_CLK clock cycle
#define DRVSPI_SPI0_CTIM2		0x1			// The SPI0 Coarse timing delayed 2 half SPI0_CLK clock cycle
#define DRVSPI_SPI0_CTIM3		0x2			// The SPI0 Coarse timing delayed 3 half SPI0_CLK clock cycle
#define DRVSPI_SPI0_FTIM0		0x3			// The SPI0 Fine timing no delayed
#define DRVSPI_SPI0_FTIM1		0x2			// The SPI0 Fine timing delayed 1 delay chains time
#define DRVSPI_SPI0_FTIM2		0x1			// The SPI0 Fine timing delayed 2 delay chains time
#define DRVSPI_SPI0_FTIM3		0			// The SPI0 Fine timing delayed 3 delay chains time
#define DRVSPI_SLAVE_LEVELTRIG		SPI_SSR_SS_LTRIG		// The SPI1 Slave Select Level Trigger
#define DRVSPI_SLAVE_EDGETRIG		0		// The SPI1 Slave Select Edge Trigger

typedef enum
{
	eDRVSPI_SLAVE_NONE	= 0,
	eDRVSPI_SLAVE_1		= 1,
	eDRVSPI_SLAVE_2		= 2
} E_DRVSPI_DEVICE;

//---------------------------------------------------------------------------------------------------------
// Define Function Prototypes
//---------------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------------------
// Function:		DrvSPI_Open
//
// Parameters:	u32SpiHandler   [in]:  set SPI chaneel
//																			DRVSPI_SPI0_HANDLER: SPI0 base. DRVSPI_SPI1_HANDLER: SPI1 base.
//
// Description:	Configure the SPI port.
//--------------------------------------------------------------------------------------------------------------
UINT32
DrvSPI_Open(
	UINT32 u32SpiHandler,
	UINT32 u32OpenFlag,
	UINT16 u16Divider
);

//---------------------------------------------------------------------------------------------------------
// Function:		DrvSPI_Close
//
// Description: Close the SPI module and disable the SPI interrupt.
//---------------------------------------------------------------------------------------------------------
void DrvSPI_Close(
	UINT32 u32SpiHandler
);

//-------------------------------------------------------------------------------------------------------------------
// Function:		DrvSPI_EnableInt
//
// Description:	Enable the SPI interrupt.
//-------------------------------------------------------------------------------------------------------------------
void DrvSPI_EnableInt(
	UINT32 u32SpiHandler
);

//-------------------------------------------------------------------------------------------------------------------
// Function:		DrvSPI_DisableInt
//
// Description:	Disable the SPI interrupt.
//-------------------------------------------------------------------------------------------------------------------
void
DrvSPI_DisableInt(
	UINT32 u32SpiHandler
);

//--------------------------------------------------------------------------------------------------------------
// Function:		DrvSPI_SetDivider
//
// Parameters:	u32SpiHandler   [in]:  set SPI chaneel
//																			DRVSPI_SPI0_HANDLER: SPI0 base. DRVSPI_SPI1_HANDLER: SPI1 base.
//
// Description:	Configure the SPI Divider.
//--------------------------------------------------------------------------------------------------------------
static __inline
void
DrvSPI_SetDivider(
	UINT32 u32SpiHandler,
	UINT16 u16Divider
)
{
	if(u32SpiHandler == DRVSPI_SPI1_HANDLER)
		u16Divider = (u16Divider>>1);
	((SPI_T*)u32SpiHandler)->DIVIDER.u32Reg = u16Divider;
}

static __inline
UINT16
DrvSPI_GetDivider(
	UINT32 u32SpiHandler,
	UINT16 u16Divider
)
{
	return (UINT16)((SPI_T*)u32SpiHandler)->DIVIDER.u32Reg;
}

//---------------------------------------------------------------------------------------------------------
// Function:		DrvSPI_GetSPIClock
//
// Returns:			SPI Working Clock
//
// Description:	This function is used to get the SPI Working Clock
//---------------------------------------------------------------------------------------------------------
static __inline
UINT32
DrvSPI_GetSPIClock(
	UINT32 u32SpiHandler
)
{
	if(u32SpiHandler == DRVSPI_SPI1_HANDLER)
		return (DrvCLK_GetHclk()/(((((SPI_T*)u32SpiHandler)->DIVIDER.u32Reg)+1)*2));
	else
		return (DrvCLK_GetHclk()/((((SPI_T*)u32SpiHandler)->DIVIDER.u32Reg)+1));
}
//---------------------------------------------------------------------------------------------------------
// Function:		DrvSPI_GetIntFlag
//
// Returns:			0:FALSE, 1:TRUE
//
// Description:	This function is used to get the SPI interrupt flag
//---------------------------------------------------------------------------------------------------------
static __inline
BOOL
DrvSPI_GetIntFlag(
	UINT32 u32SpiHandler
)
{
	return ((SPI_T*)u32SpiHandler)->CNTRL.IF; //for 072 name is SPIIF
}

//---------------------------------------------------------------------------------------------------------
// Function:		DrvSPI_ClearIntFlag
//
// Description:	This function is used to clear the SPI interrupt flag
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvSPI_ClearIntFlag(
	UINT32 u32SpiHandler
)
{
	((SPI_T*)u32SpiHandler)->CNTRL.IF = 1; //for 072 name is SPIIF
}

//-------------------------------------------------------------------------------------------------------------------
// Function:		DrvSPI_SetGo
//
// Description:	Set the GO_BUSY bit to trigger a SPI data trasfer.
//-------------------------------------------------------------------------------------------------------------------
static __inline
void
DrvSPI_SetGo(
	UINT32 u32SpiHandler
)
{
	((SPI_T*)u32SpiHandler)->CNTRL.u32Reg = (((SPI_T*)u32SpiHandler)->CNTRL.u32Reg&(~SPI_CNTRL_IF)) | SPI_CNTRL_GO_BUSY; //for 072 name is SPIIF
}

//-------------------------------------------------------------------------------------------------------------------
// Function:		DrvSPI_GetBusy
//
// Returns:			TRUE: SPI is busy, FALSE: SPI is not in busy
//
// Description:	Check the busy status of the SPI port.
//-------------------------------------------------------------------------------------------------------------------
static __inline
BOOL
DrvSPI_GetBusy(
	UINT32 u32SpiHandler
)
{

	return ((SPI_T*)u32SpiHandler)->CNTRL.GO_BUSY;
}

//---------------------------------------------------------------------------------------------------------
// Function:		DrvSPI_SetBitLen
//
// Parameters:	u8BitLength [in]: Specify the bit length (1~32 bits)
//
// Description:	Set the bit length of SPI transfer.
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvSPI_SetBitLen(
	UINT32 u32SpiHandler,
	UINT8 u8BitLength
)
{
	((SPI_T*)u32SpiHandler)->CNTRL.u32Reg =
		(((SPI_T*)u32SpiHandler)->CNTRL.u32Reg & (~(SPI_CNTRL_IF|SPI_CNTRL_TX_BIT_LEN))) | ((u8BitLength&0x1f)<<3) ; //for 072 name is SPIIF
}

//----------------------------------------------------------------------------------------------------------
// Function: DrvSPI_SetDataCount
//
// Parameters:	u8DataCount  [in]:	1: Transfer/Receive 1 data each time.
//																	2: Transfer/Receive 2 data each time.
//
// Description: Set SPI Transfer/Receive data count each time.
//----------------------------------------------------------------------------------------------------------

static __inline
void
DrvSPI_SetDataCount(
	UINT32 u32SpiHandler,
	UINT8 u8DataCount
)
{
	((SPI_T*)u32SpiHandler)->CNTRL.u32Reg =
		(((SPI_T*)u32SpiHandler)->CNTRL.u32Reg & (~(SPI_CNTRL_IF|SPI_CNTRL_TX_NUM))) | ((u8DataCount-1)<<8) ; //for 072 name is SPIIF
}

//----------------------------------------------------------------------------------------------------------
// Function: DrvSPI_SetDataConfig
//
// Parameters:	u8BitLength [in]: Specify the bit length (1~32 bits).
//
// Description: Configuration SPI Data transfer status.
//----------------------------------------------------------------------------------------------------------

static __inline
void
DrvSPI_SetDataConfig(
	UINT32 u32SpiHandler,
	UINT8 u8DataCount,
	UINT8 u8BitLength
)
{
	((SPI_T*)u32SpiHandler)->CNTRL.u32Reg =
		(((SPI_T*)u32SpiHandler)->CNTRL.u32Reg & (~(SPI_CNTRL_IF|SPI_CNTRL_TX_NUM|SPI_CNTRL_TX_BIT_LEN))) | ((u8DataCount-1)<<8) | ((u8BitLength&0x1f)<<3) ; //for 072 name is SPIIF
}

static __inline
void
DrvSPI_SetDataLittleEndian(
	UINT32 u32SpiHandler,
	BOOL bLittleEndian
)
{
	((SPI_T*)u32SpiHandler)->CNTRL.u32Reg =
		(((SPI_T*)u32SpiHandler)->CNTRL.u32Reg & (~(SPI_CNTRL_IF|SPI_CNTRL_BYTE_ENDIAN))) | (bLittleEndian<<20); //for 072 name is SPIIF
}

//----------------------------------------------------------------------------------------------------------
// Function: DrvSPI_SlaveSelect
//
// Parameters:	bAutoSlaveSelect  [in]: Set the Automatic Slave Slect.
//							bActiveHigh	[in]: Set the Device active status
//
// Description: Configuration SPI slave select function and set the specified slave select pin.
//----------------------------------------------------------------------------------------------------------

static __inline
void
DrvSPI_SlaveSelect(
	UINT32 u32SpiHandler,
	BOOL bAutoSlaveSelect,
	BOOL bActiveHigh
)
{
	((SPI_T*)u32SpiHandler)->SSR.u32Reg = (bAutoSlaveSelect << 3) + (bActiveHigh << 2);
}

//----------------------------------------------------------------------------------------------------------
// Function: DrvSPI_SelectSlave
//
// Parameters:	eDevice  [in]: Select the slave select pin which will be used.
//
// Description: Set the specified slave select pin.
//----------------------------------------------------------------------------------------------------------
static __inline
void
DrvSPI_SelectSlave(
	UINT32 u32SpiHandler,
	E_DRVSPI_DEVICE eDevice
)
{
	((SPI_T*)u32SpiHandler)->SSR.SSPC = eDevice;
}

//-------------------------------------------------------------------------------------------------------------------
// Function:		DrvSPI_SingleReadData0
//
// Return:	The data got from the SPI Rx.
//
// Description:	Read data from SPI Rx registers.
//-------------------------------------------------------------------------------------------------------------------
static __inline
UINT32
DrvSPI_SingleReadData0(
	UINT32 u32SpiHandler
)
{
	return ((SPI_T*)u32SpiHandler)->RX[0];
}

static __inline
UINT32
DrvSPI_SingleReadData1(
	UINT32 u32SpiHandler
)
{
	return ((SPI_T*)u32SpiHandler)->RX[1];
}

//-------------------------------------------------------------------------------------------------------------------
// Function:		DrvSPI_SingleWriteData0
//
// Parameters:	u32Data  [in]:  The data which will be transmitted through the SPI bus.
//
// Description:	Write data to SPI bus.
//-------------------------------------------------------------------------------------------------------------------
static __inline
void
DrvSPI_SingleWriteData0(
	UINT32 u32SpiHandler,
	UINT32 u32Data
)
{
	((SPI_T*)u32SpiHandler)->TX[0] = u32Data;
//	((SPI_T*)u32SpiHandler)->TX[0] = 0xff;
}

static __inline
void
DrvSPI_SingleWriteData1(
	UINT32 u32SpiHandler,
	UINT32 u32Data
)
{
	((SPI_T*)u32SpiHandler)->TX[1] = u32Data;
}
//-------------------------------------------------------------------------------------------------------------------
// Function:		DrvSPI_BurstReadData
//
// Parameters:	pu32Buf   [out]: Store the data got from the SPI bus.
//
// Description:	Read two words of data from SPI Rx registers
//-------------------------------------------------------------------------------------------------------------------
static __inline
void
DrvSPI_BurstReadData(
	UINT32 u32SpiHandler,
	UINT32 *pu32Buf
)
{
	pu32Buf[0] = ((SPI_T*)u32SpiHandler)->RX[0];
	pu32Buf[1] = ((SPI_T*)u32SpiHandler)->RX[1];
}

//-------------------------------------------------------------------------------------------------------------------
// Function:		DrvSPI_BurstWriteData
//
// Parameters:	pu32Buf   [in]:  Store the data which will be transmitted through the SPI bus.
//
// Description:	Write two words of data to SPI bus
//-------------------------------------------------------------------------------------------------------------------
static __inline
void
DrvSPI_BurstWriteData(
	UINT32 u32SpiHandler,
	UINT32 *pu32Buf
)
{
	((SPI_T*)u32SpiHandler)->TX[0] = pu32Buf[0];
	((SPI_T*)u32SpiHandler)->TX[1] = pu32Buf[1];
}

//for SPI1 Slave mode
static __inline
void
DrvSPI_SPI1_SetSlaveMode(
	BOOL bSlaveMode
)
{
	((SPI_T*)DRVSPI_SPI1_HANDLER)->CNTRL.u32Reg =
		(((SPI_T*)DRVSPI_SPI1_HANDLER)->CNTRL.u32Reg & (~(SPI_CNTRL_IF|SPI_CNTRL_SLAVE))) | (bSlaveMode<<18); //for 072 name is SPIIF
}

static __inline
void
DrvSPI_SPI1_LevelTriggerInSlave(
	BOOL bLevelTrigger
)
{
	((SPI_T*)DRVSPI_SPI1_HANDLER)->SSR.SS_LTRIG = bLevelTrigger;
}

static __inline
BOOL
DrvSPI_SPI1_GetLevelTriggerFlag()
{
	return ((SPI_T*)DRVSPI_SPI1_HANDLER)->SSR.LTRIG_FLAG;
}

//for SPI0 high speed fine tune
static __inline
void
DrvSPI_SPI0_ReceiveTimingTune_P(
	UINT8 u8CoarseTune,
	UINT8 u8FineTune
)
{
	((SPI_T*)DRVSPI_SPI0_HANDLER)->SPI0_RCLK.u32Reg = ((u8CoarseTune&0x3) << 2) + (u8FineTune&0x3);
}

UINT32
DrvSPI_GetVersion(void);

#ifdef  __cplusplus
}
#endif

#endif
