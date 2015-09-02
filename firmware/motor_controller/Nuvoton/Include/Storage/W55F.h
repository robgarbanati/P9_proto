/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.  *
 *                                                              *
 ****************************************************************/

#ifndef __W55F_H__
#define __W55F_H__

// Include header file
#include "Platform.h"
#include "Driver/DrvGPIO.h"
#include "System/SysInfra.h"

#ifdef  __cplusplus
extern "C"
{
#endif

// Version number
#define W55F_MAJOR_NUM	2
#define W55F_MINOR_NUM	00
#define W55F_BUILD_NUM	001
#define W55F_VERSION_NUM	_SYSINFRA_VERSION(W55F_MAJOR_NUM, W55F_MINOR_NUM, W55F_BUILD_NUM)

#define W55F_PAGE_SIZE	256

// APIs declaration
UINT32
W55F_GetVersion(void);

void
W55F_Init(
	GPIO_T* pGPIO
);

void
W55F_Write(
	UINT32 u32Addr,
	PUINT32 pau32Data,
	UINT32 u32DataLen
);

void
W55F_Erase4KBits(
	UINT16 u16IndexOf4K,
	UINT16 u16EraseCount
);

void
W55F_Erase16KBits(
	UINT16 u16IndexOf16K,
	UINT16 u16EraseCount
);

void
W55F_EraseChip(void);

void
W55F_Read(
	UINT32 u32ByteAddr,
	PUINT8 pau8Data,
	UINT32 u32DataLen
);

void
W55F_PageCoding(void);

#ifdef  __cplusplus
}
#endif

#endif	// __W55F_H__

