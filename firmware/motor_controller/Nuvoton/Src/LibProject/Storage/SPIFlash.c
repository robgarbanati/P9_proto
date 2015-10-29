/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved. *
 *                                                              *
 ****************************************************************/
#include "Driver/DrvSPI.h"
#include "Driver/DrvSYS.h"
#include "Storage/SPIFlash.h"
#include <string.h>

// Device ID for support flash type list
#define SPIFLASH_W25P		0x10			// W25P series
#define SPIFLASH_W25X		0x40			// W25X series
#define SPIFLASH_W25Q		0x50			// W25Q series
// W25P series
#define SPIFLASH_W25P10		0x10			// 1M-bits
#define SPIFLASH_W25P20		0x11			// 2M-bits
#define SPIFLASH_W25P40		0x12			// 4M-bits
#define SPIFLASH_W25P80		0x13			// 8M-bits
#define SPIFLASH_W25P16		0x14			// 16M-bits
#define SPIFLASH_W25P32		0x15			// 32M-bits

// W25X series
#define SPIFLASH_W25X10		0x40			// 1M-bits
#define SPIFLASH_W25X20		0x41			// 2M-bits
#define SPIFLASH_W25X40		0x42			// 4M-bits
#define SPIFLASH_W25X80		0x43			// 8M-bits
#define SPIFLASH_W25X16		0x44			//16M-bits
#define SPIFLASH_W25X32		0x45			//32M-bits
#define SPIFLASH_W25X64		0x46			//64M-bits
// W25Q series
#define SPIFLASH_W25Q10		0x50			// 1M-bits
#define SPIFLASH_W25Q20		0x51			// 2M-bits
#define SPIFLASH_W25Q40		0x52			// 4M-bits
#define SPIFLASH_W25Q80		0x53			// 8M-bits
#define SPIFLASH_W25Q16		0x54			//16M-bits
#define SPIFLASH_W25Q32		0x55			//32M-bits
#define SPIFLASH_W25Q64		0x56			//64M-bits
#define SPIFLASH_W25Q128	0x57			//128M-bits

volatile PUINT8 g_pau8Data;
volatile UINT32 g_u32DataLen=0;
volatile E_SPIFLASH_STATE g_eSPIFlashState = eSPIFLASH_IDLE_STATE;
S_SPIFLASH_HANDLER g_psSpiFlashHandler;

void
SPIFlash_SendRecOneData(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	UINT32 u32Data,
	UINT8  u8DataLen
)
{
	UINT32 u32SpiHandler = psSpiFlashHandler->u32SpiHandler;

	// Active chip select
	DrvSPI_SelectSlave(u32SpiHandler, psSpiFlashHandler->eSlaveDevice);

	DrvSPI_SetDataConfig(u32SpiHandler, 1, u8DataLen);
	DrvSPI_SingleWriteData0(u32SpiHandler, u32Data);
	DrvSPI_SetGo(u32SpiHandler);
	while( DrvSPI_GetBusy(u32SpiHandler) );

	// Inactive chip select
	DrvSPI_SelectSlave(u32SpiHandler, eDRVSPI_SLAVE_NONE);
}

BOOL
SPIFlash_CheckBusy(
	S_SPIFLASH_HANDLER *psSpiFlashHandler
)
{
	return (SPIFlash_ReadStatusReg(psSpiFlashHandler)& SPIFLASH_BUSY);
}

void
SPIFlash_WaitReady(
	S_SPIFLASH_HANDLER *psSpiFlashHandler
)
{
	while( SPIFlash_ReadStatusReg(psSpiFlashHandler)& SPIFLASH_BUSY );
/*
	UINT8 u8Status;
	UINT32 u32SpiHandler = psSpiFlashHandler->u32SPIHandler;

	// Active chip select
	DrvSPI_SelectSlave(u32SpiHandler, psSpiFlashHandler->u8SlaveDevice);

	DrvSPI_SetDataConfig(u32SpiHandler, 1, 16);
	DrvSPI_SingleWriteData0(u32SpiHandler, SPIFLASH_READ_STATUS<<8);
	do
	{
		DrvSPI_SetGo(u32SpiHandler);
		while( DrvSPI_GetBusy(u32SpiHandler) );
		if ( ((u8Status=DrvSPI_SingleReadData0(u32SpiHandler)) & SPIFLASH_BUSY) == 0 )
			break;
		DrvSPI_SetBitLen(u32SpiHandler, 8);
	}
	while((u8Status & SPIFLASH_BUSY));

	// Inactive chip select
	DrvSPI_SelectSlave(u32SpiHandler, DRVSPI_SLAVE_NONE);
*/
}

UINT8
SPIFlash_ReadStatusReg(
	S_SPIFLASH_HANDLER *psSpiFlashHandler
)
{
	SPIFlash_SendRecOneData(psSpiFlashHandler,SPIFLASH_READ_STATUS<<8, 16);
	return (UINT8)DrvSPI_SingleReadData0(psSpiFlashHandler->u32SpiHandler);
}

void
SPIFlash_WriteStatusReg(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	UINT8 u8Status
)
{
	SPIFlash_ChipWriteEnable(psSpiFlashHandler, TRUE);

	SPIFlash_SendRecOneData(psSpiFlashHandler,(SPIFLASH_WRITE_STATUS<<8)|u8Status, 16);
	SPIFlash_WaitReady(psSpiFlashHandler);
}


UINT32
SPIFlash_GetVersion(void)
{
	return SPIFLASH_VERSION_NUM;
}

void
SPIFlash_Open(
	UINT32 u32DrvSpiHandler,
	E_DRVSPI_DEVICE eDrvSlaveDevice,
	UINT16 u16SpiDivider,
	S_SPIFLASH_HANDLER *psSpiFlashHandler
)
{
	psSpiFlashHandler->eSlaveDevice = eDrvSlaveDevice;
	psSpiFlashHandler->u32SpiHandler = u32DrvSpiHandler;

#if (defined (__N572F072__) || defined (__N572P072__))
	if (u32DrvSpiHandler == DRVSPI_SPI0_HANDLER)// Enable high speed pins
	   DrvSYS_EnableGPIOHSTransition();
#endif

	DrvSPI_Open(
		u32DrvSpiHandler,
		DRVSPI_ENDIAN_BIG|DRVSPI_MSB_FIRST|DRVSPI_IDEL_CLK_HIGH|DRVSPI_TX_1DATA|DRVSPI_TX_NEGEDGE|DRVSPI_RX_POSEDGE|
		_DRVSPI_SLEEP(3)|_DRVSPI_DATA_BITS(8),
		u16SpiDivider);
	psSpiFlashHandler->u32FlashSize = 0;
	psSpiFlashHandler->u8Flag = 0;
	// Inactive all slave devices
	DrvSPI_SelectSlave(psSpiFlashHandler->u32SpiHandler, eDRVSPI_SLAVE_NONE);
}

void
SPIFlash_GetChipInfo(
	S_SPIFLASH_HANDLER *psSpiFlashHandler
)
{
	UINT8  u8DeviceID, u8ManufacID, u8Flag=0;
	UINT32 u32SpiHandler = psSpiFlashHandler->u32SpiHandler;
	UINT32 u32Value;
	UINT8  u8MemType;
	BOOL   bW25P_SPIFlash=FALSE;
   
	// W25P serious do not support JEDEC ID command
	// therefore, use Get ID command for W25P serious
	// Active chip select
	DrvSPI_SelectSlave(u32SpiHandler, psSpiFlashHandler->eSlaveDevice);

	DrvSPI_SetDataConfig(u32SpiHandler, 2, 24);
	DrvSPI_SingleWriteData0(u32SpiHandler, ((UINT32)SPIFLASH_DEVICE_ID<<16));
	DrvSPI_SingleWriteData1(u32SpiHandler, 0);
	DrvSPI_SetGo(u32SpiHandler);
	while( DrvSPI_GetBusy(u32SpiHandler) );
	u32Value = DrvSPI_SingleReadData1(u32SpiHandler);
	u8DeviceID = (UINT8)u32Value;
	u8ManufacID = (UINT8)(u32Value>>8);
	// Inactive all slave devices
	DrvSPI_SelectSlave(u32SpiHandler, eDRVSPI_SLAVE_NONE);

	// Get JEDEC ID command to detect W25X,W25Q MXIC and ATmel series
	// Only W25P serious not support JEDEC ID command
	SPIFlash_SendRecOneData(psSpiFlashHandler, (UINT32)SPIFLASH_JEDEC_ID<<24, 32);
	u32Value = DrvSPI_SingleReadData0(u32SpiHandler);
	if ( ((u32Value>>16)&0xff) == u8ManufacID )
	{
		u8DeviceID = ((UINT8)u32Value) - 1;
		u8MemType  = (UINT8)(u32Value>>8);

		if( ((u32Value>>16)&0xff) == 0xef )		
		{
			u8Flag = SPIFLASH_FLAG_WINBOND;			
			u8DeviceID += u8MemType;

			if ( u8MemType == 0x20 ) // W25P80,W25P32,W25P16
				bW25P_SPIFlash = TRUE;
		}
		else
		{
			u8Flag = SPIFLASH_FLAG_MXIC;
			u8DeviceID += 0x30;
		}
	}
	else if( ((u32Value>>16)&0xff) == 0x1f ) // Atmel SPIFlash
	{		
		u8DeviceID = (((u32Value>>8)&0x1f) - 2 );
		u8Flag = SPIFLASH_FLAG_ATMEL;		
	}

	if ( (u8DeviceID == 0) || ((u8DeviceID & 0xf) > 7) )
	{
		psSpiFlashHandler->u32FlashSize = 0;
		return;
	}
	psSpiFlashHandler->u32FlashSize = 1024*1024/8<<(u8DeviceID&0xf);

	u8DeviceID &= 0xf0;

	if(bW25P_SPIFlash)
	{
		psSpiFlashHandler->u8Flag = u8Flag|SPIFLASH_FLAG_ERASE_64K;	
		return;
	}

	if ( u8DeviceID == SPIFLASH_W25X )
		u8Flag = u8Flag|SPIFLASH_FLAG_ERASE_64K|SPIFLASH_FLAG_ERASE_4K;
	else if ( u8DeviceID == SPIFLASH_W25Q )
		u8Flag = u8Flag|SPIFLASH_FLAG_ERASE_64K|SPIFLASH_FLAG_ERASE_4K|SPIFLASH_FLAG_ERASE_32K;
	else if ( u8DeviceID == SPIFLASH_W25P )
		u8Flag = u8Flag|SPIFLASH_FLAG_ERASE_64K;
	else if( ((u32Value>>16)&0xff) == 0x1f ) // Atmel SPIFlash
		u8Flag = u8Flag|SPIFLASH_FLAG_ERASE_64K|SPIFLASH_FLAG_ERASE_4K|SPIFLASH_FLAG_ERASE_32K;
	else
	{
		psSpiFlashHandler->u32FlashSize = 0;
		return;
	}
	psSpiFlashHandler->u8Flag = u8Flag;
}

void
SPIFlash_ChipWriteEnable(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	BOOL bEnableWrite
)
{
	UINT8 u8Cmd;

	if ( bEnableWrite == TRUE )
		u8Cmd = SPIFLASH_WRITE_ENABLE;
	else
		u8Cmd = SPIFLASH_WRITE_DISABLE;

	SPIFlash_SendRecOneData(psSpiFlashHandler, u8Cmd, 8 );
	SPIFlash_WaitReady(psSpiFlashHandler);
}

void
SPIFlash_GlobalProtect(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	BOOL bEnableGlobalProtect
)
{
	UINT8 u8Status=0,u8Flag = ((psSpiFlashHandler->u8Flag)&0xf0);

	if( (u8Flag == SPIFLASH_FLAG_WINBOND) || (u8Flag == SPIFLASH_FLAG_MXIC) )
	{
		u8Status = SPIFlash_ReadStatusReg(psSpiFlashHandler);

		if(bEnableGlobalProtect)
			u8Status |= 0x1c; // Set 1 to BP0,BP1,BP2 in Status Register 
		else
		{
			if(u8Status&0x20) // Check BP3/TB bit (some flash have it) 
				u8Status &= 0xc3; // Set 0 to BP0,BP1,BP2,BP3/TB in Status Register
			else
				u8Status &= 0xe3; // Set 0 to BP0,BP1,BP2 in Status Register
		}

		SPIFlash_WriteStatusReg(psSpiFlashHandler, u8Status);
	}
	else if ( u8Flag == SPIFLASH_FLAG_ATMEL )
	{
		if(bEnableGlobalProtect)
			u8Status = 0x3c;	// Set [7:0] 0X1111XX to Write Status Register (X is ignore bit )
		else
			u8Status = 0x00;	// Set [7:0] 0X0000XX to Write Status Register
				
		SPIFlash_WriteStatusReg(psSpiFlashHandler, u8Status);
	}
}

void
SPIFlash_BlockWriteProtect(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	UINT8 u8BlockSize
)
{
	UINT8 u8Status;

	u8Status = SPIFlash_ReadStatusReg(psSpiFlashHandler);
	u8Status = (u8Status&(~SPIFLASH_BP))|u8BlockSize;
	SPIFlash_WriteStatusReg(psSpiFlashHandler, u8Status);
}

void
SPIFlash_Erase(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	UINT32 u32CmdAddr,
	UINT32 u32AddIncreament,
	UINT16 u16EraseCount
)
{
	while(u16EraseCount--)
	{
		SPIFlash_EraseStart(psSpiFlashHandler,u32CmdAddr);
		u32CmdAddr += u32AddIncreament;

		// Wait erase complete
		SPIFlash_WaitReady(psSpiFlashHandler);
	}
}

void
SPIFlash_EraseChip(
	S_SPIFLASH_HANDLER *psSpiFlashHandler
)
{
	SPIFlash_EraseChipStart(psSpiFlashHandler);
	// Wait erase complete
	SPIFlash_WaitReady(psSpiFlashHandler);
}

void
SPIFlash_PowerDown(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	BOOL	bEnable
)
{
	UINT8 u8Cmd;

	if ( bEnable )
		u8Cmd = SPIFLASH_POWER_DOWN;
	else
		u8Cmd = SPIFLASH_RELEASE_PD_ID;

	SPIFlash_SendRecOneData(psSpiFlashHandler,u8Cmd,8);
}

void
SPIFlash_WriteStart(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	UINT32 u32ByteAddr
)
{
	UINT32 u32SpiHandler = psSpiFlashHandler->u32SpiHandler;

	SPIFlash_ChipWriteEnable(psSpiFlashHandler, TRUE);

	// Active chip select
	DrvSPI_SelectSlave(u32SpiHandler, psSpiFlashHandler->eSlaveDevice);

	// Send write page command
	DrvSPI_SetDataConfig(u32SpiHandler, 1, 32);
	DrvSPI_SingleWriteData0(u32SpiHandler, ((UINT32)SPIFLASH_PAGE_PROGRAM<<24)|u32ByteAddr);
	DrvSPI_SetGo(u32SpiHandler);
	while( DrvSPI_GetBusy(u32SpiHandler) );
	DrvSPI_SetDataLittleEndian(u32SpiHandler, TRUE);
}

void
SPIFlash_WriteEnd(
	S_SPIFLASH_HANDLER *psSpiFlashHandler
)
{
	// Inactive all slave devices
	DrvSPI_SelectSlave(psSpiFlashHandler->u32SpiHandler, eDRVSPI_SLAVE_NONE);
	DrvSPI_SetDataLittleEndian(psSpiFlashHandler->u32SpiHandler, FALSE);
}

void
SPIFlash_WritePage(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	UINT32 u32PageAddr,
	PUINT8 pau8Data
)
{
	//SPIFlash_WritePage_NBK(psSpiFlashHandler, u32PageAddr, pau8Data);
	
	SPIFlash_WriteStart(psSpiFlashHandler, u32PageAddr);
	SPIFlash_WriteDataAlign(psSpiFlashHandler->u32SpiHandler, pau8Data);
	SPIFlash_WriteEnd(psSpiFlashHandler);

	SPIFlash_WaitReady(psSpiFlashHandler);
}

ERRCODE
SPIFlash_WritePage_NBK(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	UINT32 u32PageAddr,
	PUINT8 pau8Data
)
{	
	if(g_eSPIFlashState != eSPIFLASH_IDLE_STATE)
		return E_SPIFLASH_BUSY;

	g_psSpiFlashHandler.u32SpiHandler =	psSpiFlashHandler->u32SpiHandler;
	g_psSpiFlashHandler.u32FlashSize = psSpiFlashHandler->u32FlashSize;
	g_psSpiFlashHandler.eSlaveDevice = psSpiFlashHandler->eSlaveDevice;
	g_psSpiFlashHandler.u8Flag = psSpiFlashHandler->u8Flag;

	SPIFlash_WriteStart(&g_psSpiFlashHandler, u32PageAddr);
	DrvSPI_SetDataConfig(g_psSpiFlashHandler.u32SpiHandler, 2, 32);

	g_pau8Data = pau8Data;
	g_u32DataLen = 256;

	DrvSPI_ClearIntFlag(g_psSpiFlashHandler.u32SpiHandler);
	DrvSPI_EnableInt(g_psSpiFlashHandler.u32SpiHandler);
	g_eSPIFlashState = eSPIFLASH_WRITE_DATABUSY;

	DrvSPI_BurstWriteData(g_psSpiFlashHandler.u32SpiHandler,(PUINT32)g_pau8Data);
	g_pau8Data += 8;

	DrvSPI_SetGo(g_psSpiFlashHandler.u32SpiHandler);

	return E_SUCCESS;
}

void
SPIFlash_ReadStart(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	UINT32 u32ByteAddr
)
{
	UINT32 u32SpiHandler = psSpiFlashHandler->u32SpiHandler;

	// Active chip select
	DrvSPI_SelectSlave(u32SpiHandler, psSpiFlashHandler->eSlaveDevice);

	// Send fast read command
	DrvSPI_SetDataConfig(u32SpiHandler, 1, 32);
	DrvSPI_SingleWriteData0(u32SpiHandler, ((UINT32)SPIFLASH_FAST_READ<<24)|(u32ByteAddr&0xffffff));
	DrvSPI_SetGo(u32SpiHandler);
	while( DrvSPI_GetBusy(u32SpiHandler) );
	DrvSPI_SetDataConfig(u32SpiHandler, 1, 8);
	DrvSPI_SetGo(u32SpiHandler);
	while( DrvSPI_GetBusy(u32SpiHandler) );

	DrvSPI_SetDataLittleEndian(u32SpiHandler, TRUE);
}

void
SPIFlash_ReadEnd(
	S_SPIFLASH_HANDLER *psSpiFlashHandler
)
{
	DrvSPI_SetDataLittleEndian(psSpiFlashHandler->u32SpiHandler, FALSE);
	// Inactive all slave devices
	DrvSPI_SelectSlave(psSpiFlashHandler->u32SpiHandler, eDRVSPI_SLAVE_NONE);
}

void
SPIFlash_ReadData(
	UINT32 u32SpiHandler,
	PUINT8 pau8Data,
	UINT32 u32DataLen
)
{
#if 1	//	PA20 CYHuang12 speedup read function.
	UINT32 u32ReadData;
    UINT8  u8ProcBytes;

    u8ProcBytes = ((UINT32)pau8Data)%4;
    if (u8ProcBytes!=0)
    {
        u8ProcBytes = 4 - u8ProcBytes;
        if ( u8ProcBytes > u32DataLen )
        	u8ProcBytes = u32DataLen;

        DrvSPI_SetDataConfig(u32SpiHandler, 1, u8ProcBytes<<3);
        DrvSPI_SetGo(u32SpiHandler);
        u32DataLen-=u8ProcBytes;

        while( DrvSPI_GetBusy(u32SpiHandler) );
        u32ReadData = DrvSPI_SingleReadData0(u32SpiHandler);

        *pau8Data++ = (UINT8)u32ReadData;
        if ( u8ProcBytes >= 2 )
        	*pau8Data++ = (UINT8)(u32ReadData>>8);
        if ( u8ProcBytes >= 3 )
        	*pau8Data++ = (UINT8)(u32ReadData>>16);
    }

    if (u32DataLen>=4)
    {
        DrvSPI_SetDataConfig(u32SpiHandler, 1, 32);
        DrvSPI_SetGo(u32SpiHandler);
    } 
    while (u32DataLen>=4)
    {
        while( DrvSPI_GetBusy(u32SpiHandler) );
        u32ReadData = DrvSPI_SingleReadData0(u32SpiHandler);
        *((UINT32*)pau8Data) = u32ReadData;
        if (u32DataLen>=8)
        {
        	DrvSPI_SetGo(u32SpiHandler);
        }
        pau8Data+=4;
        u32DataLen-=4;
    }

    if (u32DataLen>0)
    {
        DrvSPI_SetDataConfig(u32SpiHandler, 1, u32DataLen<<3);
        DrvSPI_SetGo(u32SpiHandler);
        while( DrvSPI_GetBusy(u32SpiHandler) );
        u32ReadData = DrvSPI_SingleReadData0(u32SpiHandler);

        *pau8Data++ = (UINT8)u32ReadData;
        if ( u32DataLen >= 2 )
        	*pau8Data++ = (UINT8)(u32ReadData>>8);
        if ( u32DataLen >= 3 )
        	*pau8Data++ = (UINT8)(u32ReadData>>16);
    }

#else
	UINT32 u32ReadData;
	UINT8  u8ProcBytes, u8SpiBytes;
	
	u8ProcBytes = 0xff;
	u8SpiBytes = 0;
	do
	{
		if ( (u8ProcBytes = ((UINT32)pau8Data)%4)!=0 )
		{
			u8ProcBytes = 4 - u8ProcBytes;
			if ( u8ProcBytes > u32DataLen )
				u8ProcBytes = u32DataLen;
		}
		else if ( u32DataLen >= 4 )
			u8ProcBytes = 4;
		else
			u8ProcBytes = u32DataLen;
		if ( u8ProcBytes != u8SpiBytes )
		{
			u8SpiBytes = u8ProcBytes;
			DrvSPI_SetDataConfig(u32SpiHandler, 1, u8ProcBytes<<3);
		}

		DrvSPI_SetGo(u32SpiHandler);
		while( DrvSPI_GetBusy(u32SpiHandler) );
		u32DataLen -= u8ProcBytes;
		if ( u8ProcBytes!=4 )
		{
			u32ReadData = DrvSPI_SingleReadData0(u32SpiHandler);
			*pau8Data++ = (UINT8)u32ReadData;
			if ( u8ProcBytes >= 2 )
				*pau8Data++ = (UINT8)(u32ReadData>>8);
			if ( u8ProcBytes >= 3 )
				*pau8Data++ = (UINT8)(u32ReadData>>16);

		}
		else
		{
			*((UINT32*)pau8Data) = DrvSPI_SingleReadData0(u32SpiHandler);
			pau8Data += 4;
		}
	}while(u32DataLen>0);
#endif
}
	
void
SPIFlash_Read(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	UINT32 u32ByteAddr,
	PUINT8 pau8Data,
	UINT32 u32DataLen
)
{
	SPIFlash_ReadStart( psSpiFlashHandler, u32ByteAddr );
	// Read data
	SPIFlash_ReadData(psSpiFlashHandler->u32SpiHandler, pau8Data, u32DataLen);
	SPIFlash_ReadEnd(psSpiFlashHandler);
}

void
SPIFlash_ReadDataAlign(
	UINT32 u32SpiHandler,
	PUINT8 pau8Data,
	UINT32 u32DataLen
)
{
	// Read data
	DrvSPI_SetDataConfig(u32SpiHandler, 2, 32);
	do
	{
		DrvSPI_SetGo(u32SpiHandler);
		while( DrvSPI_GetBusy(u32SpiHandler) );
		DrvSPI_BurstReadData(u32SpiHandler, (UINT32*)pau8Data);
		pau8Data+=8;
		u32DataLen -= 8;
	}while(u32DataLen>0);
}

void
SPIFlash_BurstRead(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	UINT32 u32ByteAddr,
	PUINT8 pau8Data,
	UINT32 u32DataLen
)
{
	SPIFlash_ReadStart( psSpiFlashHandler, u32ByteAddr );
	SPIFlash_ReadDataAlign(psSpiFlashHandler->u32SpiHandler, pau8Data, u32DataLen);
	SPIFlash_ReadEnd(psSpiFlashHandler);
}

void
SPIFlash_SlowRead(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	UINT32 u32ByteAddr,
	PUINT8 pau8Data,
	UINT32 u32DataLen
)
{
	UINT8 u8ProcBytes;
	UINT32 u32ReadData, u32SpiHandler;
	
	SPIFlash_ReadStart( psSpiFlashHandler, u32ByteAddr );
	u32SpiHandler = psSpiFlashHandler->u32SpiHandler;
	while(u32DataLen)
	{
		if ( u32DataLen>4 )
			u8ProcBytes = 4;
		else
			u8ProcBytes = (UINT8)u32DataLen;

		DrvSPI_SetDataConfig(u32SpiHandler, 1, u8ProcBytes<<3);
		DrvSPI_SetGo(u32SpiHandler);
		while( DrvSPI_GetBusy(u32SpiHandler) );
		u32ReadData = DrvSPI_SingleReadData0(u32SpiHandler);
		u32DataLen -= u8ProcBytes;
		do
		{
			*pau8Data++ = (UINT8)u32ReadData;
			u32ReadData >>= 8;
		}while( (--u8ProcBytes) != 0 );
	}
	SPIFlash_ReadEnd(psSpiFlashHandler);
}

ERRCODE
SPIFlash_BurstRead_NBK(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	UINT32 u32ByteAddr,
	PUINT8 pau8Data,
	UINT32 u32DataLen
)
{
	if(g_eSPIFlashState != eSPIFLASH_IDLE_STATE)
		return E_SPIFLASH_BUSY;

	g_psSpiFlashHandler.u32SpiHandler =	psSpiFlashHandler->u32SpiHandler;
	g_psSpiFlashHandler.u32FlashSize = psSpiFlashHandler->u32FlashSize;
	g_psSpiFlashHandler.eSlaveDevice = psSpiFlashHandler->eSlaveDevice;
	g_psSpiFlashHandler.u8Flag = psSpiFlashHandler->u8Flag;

	SPIFlash_ReadStart( psSpiFlashHandler, u32ByteAddr );

	DrvSPI_SetDataConfig(g_psSpiFlashHandler.u32SpiHandler, 2, 32);

	g_pau8Data = pau8Data;
	g_u32DataLen = u32DataLen;

	DrvSPI_ClearIntFlag(g_psSpiFlashHandler.u32SpiHandler);
	DrvSPI_EnableInt(g_psSpiFlashHandler.u32SpiHandler);
	g_eSPIFlashState = eSPIFLASH_READ_BUSY;

	DrvSPI_SetGo(g_psSpiFlashHandler.u32SpiHandler);

	return E_SUCCESS;
}

static void
SPIFlash_SendDataLess4Bytes(
	UINT32 u32SpiHandler,
	PUINT8 pau8Data,
	UINT32 u32DataLen
)
{
	UINT32 u32WriteData;

	DrvSPI_SetDataConfig(u32SpiHandler, 1, u32DataLen*8);
	u32WriteData = *pau8Data ++;
	if ( u32DataLen >= 2 )
	   u32WriteData |= (*pau8Data ++)<<8;
	if ( u32DataLen == 3 )
	   u32WriteData |= (*pau8Data ++)<<16;
	DrvSPI_SingleWriteData0(u32SpiHandler, u32WriteData);
	DrvSPI_SetGo(u32SpiHandler);
	while( DrvSPI_GetBusy(u32SpiHandler) );
}

UINT32
SPIFlash_WriteData(
	UINT32 u32SpiHandler,
	UINT32 u32SPIAddr,
	PUINT8 pau8Data,
	UINT32 u32DataLen
)
{
	UINT32 u32WriteCount, u32TotalWriteCount, u32ProcessByte;
	
	u32WriteCount = 256;
	if ( u32SPIAddr%256 )
		 u32WriteCount -= u32SPIAddr%256;
	if ( u32WriteCount > u32DataLen )
		u32WriteCount = u32DataLen;
	u32TotalWriteCount = u32WriteCount;

	if ( ((UINT32)pau8Data)%4 )
	{
		u32ProcessByte = 4 - ((UINT32)pau8Data)%4;
		if ( u32ProcessByte > u32WriteCount )
			u32ProcessByte = u32WriteCount;
		SPIFlash_SendDataLess4Bytes(u32SpiHandler, pau8Data, u32ProcessByte);
		u32WriteCount -=  u32ProcessByte;
		pau8Data += u32ProcessByte;
	}

	DrvSPI_SetDataConfig(u32SpiHandler, 1, 32);
	while(u32WriteCount >= 4)
	{
		DrvSPI_SingleWriteData0(u32SpiHandler,*((PUINT32)pau8Data));
		DrvSPI_SetGo(u32SpiHandler);
		pau8Data += 4;
		u32WriteCount -= 4;
		while( DrvSPI_GetBusy(u32SpiHandler) );
	}
	if (u32WriteCount)
	{
		SPIFlash_SendDataLess4Bytes(u32SpiHandler, pau8Data, u32WriteCount);
		pau8Data += u32WriteCount;
	}
	return u32TotalWriteCount;
}

void
SPIFlash_Write(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	UINT32 u32Addr,
	PUINT8 pau8Data,
	UINT32 u32DataLen
)
{
	UINT32 u32WriteCount;
	UINT32 u32SpiHandler = psSpiFlashHandler->u32SpiHandler;

	while(u32DataLen!=0)
	{
		SPIFlash_WriteStart(psSpiFlashHandler, u32Addr);
		u32WriteCount = SPIFlash_WriteData(u32SpiHandler,u32Addr, pau8Data, u32DataLen);
		u32Addr += u32WriteCount;
		pau8Data += u32WriteCount;
		u32DataLen -= u32WriteCount;
		SPIFlash_WriteEnd(psSpiFlashHandler);
		// Wait erase complete
		SPIFlash_WaitReady(psSpiFlashHandler);
	}
}

void
SPIFlash_EraseStart(
	S_SPIFLASH_HANDLER *psSpiFlashHandler,
	UINT32 u32CmdAddr
)
{
	SPIFlash_ChipWriteEnable(psSpiFlashHandler, TRUE);
	SPIFlash_SendRecOneData(psSpiFlashHandler,u32CmdAddr,32);
}

void
SPIFlash_EraseChipStart(
	S_SPIFLASH_HANDLER *psSpiFlashHandler
)
{
	SPIFlash_ChipWriteEnable(psSpiFlashHandler, TRUE);
	SPIFlash_SendRecOneData(psSpiFlashHandler,SPIFLASH_CHIP_ERASE,8);
}

void
SPIFlash_WriteDataAlign(
	UINT32 u32SpiHandler,
	PUINT8 pau8Data
)
{
	UINT32 u32DataLen;
	DrvSPI_SetDataConfig(u32SpiHandler, 2, 32);
	u32DataLen = 256;
	do
	{
		DrvSPI_BurstWriteData(u32SpiHandler,(PUINT32)pau8Data);
		DrvSPI_SetGo(u32SpiHandler);
		pau8Data += 8;
		while( DrvSPI_GetBusy(u32SpiHandler) );
	}while(	(u32DataLen -= 8) != 0 );
}

static void NBK_ReadStep(void)
{
	DrvSPI_BurstReadData(g_psSpiFlashHandler.u32SpiHandler, (UINT32*)g_pau8Data);
	g_pau8Data+=8;
	g_u32DataLen -= 8;

	if(g_u32DataLen > 0)
		DrvSPI_SetGo(g_psSpiFlashHandler.u32SpiHandler);
	else
	{
		SPIFlash_ReadEnd(&g_psSpiFlashHandler);
		g_eSPIFlashState = eSPIFLASH_READ_SUCCESS;
	}
}

static void NBK_WriteStep(void)
{
	if((g_u32DataLen -= 8) != 0)
	{
		DrvSPI_BurstWriteData(g_psSpiFlashHandler.u32SpiHandler,(PUINT32)g_pau8Data);
		g_pau8Data += 8;
		DrvSPI_SetGo(g_psSpiFlashHandler.u32SpiHandler);
	}
	else
	{
		SPIFlash_WriteEnd(&g_psSpiFlashHandler);
		
		if( SPIFlash_ReadStatusReg(&g_psSpiFlashHandler)& SPIFLASH_BUSY )
			g_eSPIFlashState = eSPIFLASH_WRITE_ENDBUSY;
		else
			g_eSPIFlashState = eSPIFLASH_WRITE_SUCCESS;	
	}
}

void SPIFlash_ISR_NBK(void)
{
	DrvSPI_DisableInt(g_psSpiFlashHandler.u32SpiHandler);
	
	if(g_eSPIFlashState == eSPIFLASH_READ_BUSY)
		NBK_ReadStep();
	
	if(g_eSPIFlashState == eSPIFLASH_WRITE_DATABUSY)
		NBK_WriteStep();	

	if( (g_eSPIFlashState==eSPIFLASH_WRITE_DATABUSY) || (g_eSPIFlashState==eSPIFLASH_READ_BUSY) )
		DrvSPI_EnableInt(g_psSpiFlashHandler.u32SpiHandler);	
}

E_SPIFLASH_STATE
SPIFlash_GetState_NBK(void)
{
	E_SPIFLASH_STATE eSPIFlashState = g_eSPIFlashState;
	 
	if( (g_eSPIFlashState == eSPIFLASH_WRITE_ENDBUSY) )
	{
		if( SPIFlash_ReadStatusReg(&g_psSpiFlashHandler)& SPIFLASH_BUSY )
			eSPIFlashState = eSPIFLASH_WRITE_ENDBUSY;
		else
		{
			eSPIFlashState = eSPIFLASH_WRITE_SUCCESS;
			g_eSPIFlashState = eSPIFLASH_IDLE_STATE;
		}
	}
	else if( (g_eSPIFlashState == eSPIFLASH_READ_SUCCESS) )
		g_eSPIFlashState = eSPIFLASH_IDLE_STATE;
	else if( (g_eSPIFlashState == eSPIFLASH_WRITE_SUCCESS) )
		g_eSPIFlashState = eSPIFLASH_IDLE_STATE;

	return eSPIFlashState;
}
