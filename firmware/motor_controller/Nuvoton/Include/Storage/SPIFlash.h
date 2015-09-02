/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.  *
 *                                                              *
 ****************************************************************/

#ifndef __SPIFLASH_H__
#define __SPIFLASH_H__

// Include header file
#include "Platform.h"
#include "Driver/DrvSPI.h"
#include "System/SysInfra.h"


#ifdef  __cplusplus
extern "C"
{
#endif

// Version number
#define SPIFLASH_MAJOR_NUM	2
#define SPIFLASH_MINOR_NUM	1
#define SPIFLASH_BUILD_NUM	1
#define SPIFLASH_VERSION_NUM	_SYSINFRA_VERSION(SPIFLASH_MAJOR_NUM, SPIFLASH_MINOR_NUM, SPIFLASH_BUILD_NUM)

// ------------------------------------------------------------------------------
// Define the Error Code
// ------------------------------------------------------------------------------
// E_SPIFLASH_BUSY				Read/Write Data Busy
#define E_SPIFLASH_BUSY         _SYSINFRA_ERRCODE(TRUE, MODULE_ID_SPIFLASH, 1)

// SPI Flash Status
#define SPIFLASH_SPR			0x80	// Status Register Protect
#define SPIFLASH_R				0x40	// Reserved Bit
#define SPIFLASH_TB				0x20	// Top / Bottom Block Protect
#define SPIFLASH_BP2			0x10	// Block Protect Bit 2
#define SPIFLASH_BP1			0x08	// Block Protect Bit 1
#define SPIFLASH_BP0			0x04	// Block Protect Bit 0
#define SPIFLASH_WEL			0x02	// Write Enable Latch
#define SPIFLASH_BUSY			0x01	// BUSY
#define SPIFLASH_BP				(SPIFLASH_TB|SPIFLASH_BP2|SPIFLASH_BP1|SPIFLASH_BP0)

// SPI Flash Command
#define SPIFLASH_ZERO			0x00
#define SPIFLASH_DUMMY			0xFF
#define SPIFLASH_WRITE_ENABLE	0x06
#define SPIFLASH_WRITE_DISABLE	0x04
#define SPIFLASH_READ_STATUS	0x05
#define SPIFLASH_WRITE_STATUS	0x01
#define SPIFLASH_READ_DATA		0x03 // Using fast read to replace normal read
#define SPIFLASH_FAST_READ		0x0B
#define SPIFLASH_FAST_RD_DUAL	0x3B
#define SPIFLASH_PAGE_PROGRAM	0x02
#define SPIFLASH_64K_ERASE		0xD8
#define SPIFLASH_4K_ERASE		0x20
#define SPIFLASH_32K_ERASE		0x52
#define SPIFLASH_CHIP_ERASE		0xC7
#define SPIFLASH_POWER_DOWN		0xB9
#define SPIFLASH_RELEASE_PD_ID	0xAB
#define SPIFLASH_DEVICE_ID		0x90
#define SPIFLASH_JEDEC_ID		0x9F


#define SPIFLASH_PAGE_SIZE	256

// Upper or lower portion of flash
#define SPIFLASH_TOP		TRUE		// Upper portion
#define SPIFLASH_BOTTOM		FALSE		// Lower portion

// Size of block protect
#define SPIFLASH_BLOCK_ALL	0x7			// all of flash
#define SPIFLASH_BLOCK_1_2	0x6			// 1/2 of flash
#define SPIFLASH_BLOCK_1_4	0x5			// 1/4 of flash
#define SPIFLASH_BLOCK_1_8	0x4			// 1/8 of flash
#define SPIFLASH_BLOCK_1_16	0x3			// 1/16 of flash
#define SPIFLASH_BLOCK_1_32	0x2			// 1/32 of flash
#define SPIFLASH_BLOCK_1_64	0x1			// 1/64 of flash
#define SPIFLASH_BLOCK_NONE	0x0

#define SPIFLASH_FLAG_ERASE_64K			0x01
#define SPIFLASH_FLAG_ERASE_4K			0x02
#define SPIFLASH_FLAG_ERASE_32K			0x04
#define SPIFLASH_FLAG_DUAL_SPI			0x08
#define SPIFLASH_FLAG_WINBOND			0x10
#define SPIFLASH_FLAG_MXIC				0x20
#define SPIFLASH_FLAG_ATMEL				0x40

//---------------------------------------------------------------
// SPI Flash NB State 
//---------------------------------------------------------------
typedef enum{
	eSPIFLASH_IDLE_STATE = 0,
	eSPIFLASH_READ_SUCCESS,
	eSPIFLASH_READ_BUSY,
	eSPIFLASH_WRITE_SUCCESS,
	eSPIFLASH_WRITE_DATABUSY,
	eSPIFLASH_WRITE_ENDBUSY
} E_SPIFLASH_STATE;

typedef struct
{
	UINT32  u32SpiHandler;				// SPI access handler
	UINT32  u32FlashSize;				// SPI memory size
	E_DRVSPI_DEVICE   eSlaveDevice;
	UINT8   u8Flag;
} S_SPIFLASH_HANDLER;

// APIs declaration
void
SPIFlash_SendRecOneData(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	UINT32 u32Data,
	UINT8  u8DataLen
);

void
SPIFlash_Erase(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	UINT32 u32CmdAddr,
	UINT32 u32AddIncreament,
	UINT16 u16EraseCount
);

UINT32
SPIFlash_GetVersion(void);

void
SPIFlash_Open(
	UINT32 u32DrvSpiHandler,
	E_DRVSPI_DEVICE eDrvSlaveDevice,
	UINT16 u16SpiDivider,
	S_SPIFLASH_HANDLER *psSpiFlashHandler
);

static __inline
void
SPIFlash_Close(
	S_SPIFLASH_HANDLER *psSpiFlashHandler
)
{
	DrvSPI_Close(psSpiFlashHandler->u32SpiHandler);
}

#if (defined (__N572F072__) || defined (__N572P072__))
static __inline
UINT32
SPIFlash_GetSPIClock(
   UINT32 u32SpiHandler
)
{
	return DrvSPI_GetSPIClock(u32SpiHandler);
}
#endif

void
SPIFlash_GetChipInfo(
	S_SPIFLASH_HANDLER *psSpiFlashHandler
);

void
SPIFlash_ChipWriteEnable(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	BOOL bEnableWrite
);

void
SPIFlash_GlobalProtect(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	BOOL bEnableGlobalProtect
);

void
SPIFlash_BlockWriteProtect(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	UINT8 u8BlockSize
);

UINT8
SPIFlash_ReadStatusReg(
	S_SPIFLASH_HANDLER *psSpiFlashHandler
);

void
SPIFlash_WriteStatusReg(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	UINT8 u8Status
);

void
SPIFlash_Write(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	UINT32 u32Addr,
	PUINT8 pau8Data,
	UINT32 u32DataLen
);

void
SPIFlash_WritePage(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	UINT32 u32PageAddr,
	PUINT8 pau8Data
);

static __inline
void
SPIFlash_Erase64K(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	UINT8  u8IndexOf64K,
	UINT16 u16EraseCount
)
{
	SPIFlash_Erase(psSpiFlashHandler, ((UINT32)SPIFLASH_64K_ERASE<<24)|(u8IndexOf64K<<16), (1<<16), u16EraseCount );
}

static __inline
void
SPIFlash_Erase4K(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	UINT16 u16IndexOf4K,
	UINT16 u16EraseCount
)
{
	SPIFlash_Erase(psSpiFlashHandler, (SPIFLASH_4K_ERASE<<24)|(u16IndexOf4K<<12), (1<<12), u16EraseCount );
}

static __inline
void
SPIFlash_Erase32K(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	UINT16 u16IndexOf32K,
	UINT16 u16EraseCount
)
{
	SPIFlash_Erase(psSpiFlashHandler, (SPIFLASH_32K_ERASE<<24)|(u16IndexOf32K<<15), (1<<15), u16EraseCount );
}

void
SPIFlash_EraseChip(
	S_SPIFLASH_HANDLER *psSpiFlashHandler
);

void
SPIFlash_Read(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	UINT32 u32ByteAddr,
	PUINT8 pau8Data,
	UINT32 u32DataLen
);

void
SPIFlash_BurstRead(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	UINT32 u32ByteAddr,
	PUINT8 pau8Data,
	UINT32 u32DataLen
);

void
SPIFlash_PowerDown(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	BOOL	bEnable
);

void
SPIFlash_WaitReady(
	S_SPIFLASH_HANDLER *psSpiFlashHandler
);

// none blocking releate APIs
void
SPIFlash_EraseStart(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	UINT32 u32CmdAddr
);

static __inline
void
SPIFlash_Erase64KStart(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	UINT8  u8IndexOf64K
)
{
	SPIFlash_EraseStart(psSpiFlashHandler, ((UINT32)SPIFLASH_64K_ERASE<<24)|(u8IndexOf64K<<16));
}

static __inline
void
SPIFlash_Erase4KStart(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	UINT16 u16IndexOf4K
)
{
	SPIFlash_EraseStart(psSpiFlashHandler, (SPIFLASH_4K_ERASE<<24)|(u16IndexOf4K<<12));
}

static __inline
void
SPIFlash_Erase32KStart(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	UINT16 u16IndexOf32K
)
{
	SPIFlash_EraseStart(psSpiFlashHandler, (SPIFLASH_32K_ERASE<<24)|(u16IndexOf32K<<15) );
}

void
SPIFlash_EraseChipStart(
	S_SPIFLASH_HANDLER *psSpiFlashHandler
);

BOOL
SPIFlash_CheckBusy(
	S_SPIFLASH_HANDLER *psSpiFlashHandler
);

void
SPIFlash_WriteStart(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	UINT32 u32ByteAddr
);

void
SPIFlash_WriteEnd(
	S_SPIFLASH_HANDLER *psSpiFlashHandler
);

UINT32
SPIFlash_WriteData(
	UINT32 u32SpiHandler,
	UINT32 u32SPIAddr,
	PUINT8 pau8Data,
	UINT32 u32DataLen
);

void
SPIFlash_WriteDataAlign(
	UINT32 u32SpiHandler,
	PUINT8 pau8Data
);

void
SPIFlash_ReadStart(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	UINT32 u32ByteAddr
);

void
SPIFlash_ReadEnd(
	S_SPIFLASH_HANDLER *psSpiFlashHandler
);

void
SPIFlash_ReadData(
	UINT32 u32SpiHandler,
	PUINT8 pau8Data,
	UINT32 u32DataLen
);

void
SPIFlash_ReadDataAlign(
	UINT32 u32SpiHandler,
	PUINT8 pau8Data,
	UINT32 u32DataLen
);

E_SPIFLASH_STATE
SPIFlash_GetState_NBK(void);

void SPIFlash_ISR_NBK(void);

ERRCODE
SPIFlash_WritePage_NBK(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	UINT32 u32PageAddr,
	PUINT8 pau8Data
);

ERRCODE
SPIFlash_BurstRead_NBK(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	UINT32 u32ByteAddr,
	PUINT8 pau8Data,
	UINT32 u32DataLen
);

#ifdef  __cplusplus
}
#endif

#endif	// __SPIFLASH_H__

