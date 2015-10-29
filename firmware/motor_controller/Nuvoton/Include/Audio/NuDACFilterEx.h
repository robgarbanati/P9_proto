/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.  *
 *                                                              *
 ****************************************************************/

#ifndef __NUDACFILTEREX_H__
#define __NUDACFILTEREX_H__
   
#include "Platform.h"
#include "System/SysInfra.h"

#ifdef  __cplusplus
extern "C"
{
#endif

// Version Definition
#define NUDACFILTEREX_MAJOR_NUM   3
#define NUDACFILTEREX_MINOR_NUM   30
#define NUDACFILTEREX_BUILD_NUM   1
#define NUDACFILTEREX_VERSION_NUM   _SYSINFRA_VERSION(NUDACFILTEREX_MAJOR_NUM, NUDACFILTEREX_MINOR_NUM, NUDACFILTEREX_BUILD_NUM)

// Constant Definition
#define NUDACFILTEREX_UP2_WORK_BUF_SIZE	(17*2)
#define NUDACFILTEREX_UP4_WORK_BUF_SIZE	( 9*2)

void NuDACFilterEx_Up2Initial(
	UINT8 *pu8Up2WorkBuf				// Buffer address must be 2 bytes alignment.
										// Buffer size is NUDACFILTER_UP2_WORK_BUF_SIZE bytes,
										// this buffer is used to keep Up2 filter internal information.
										// Can not be used with others.
);

void NuDACFilterEx_Up2Process(			
	UINT8 *pu8Up2WorkBuf,				// Buffer address must be 2 bytes alignment.
										// Buffer size is NUDACFILTER_UP2_WORK_BUF_SIZE bytes,
										// this buffer is used to keep Up2 filter internal information.
										// Can not be used with others.
	INT16 i16PCM,						// Input  1 PCM
	INT16 *pi16Up2PCMs					// Return 2 PCMs
);

void NuDACFilterEx_Up4Initial(
	UINT8 *pu8Up4WorkBuf				// Buffer address must be 2 bytes alignment.
										// Buffer size is NUDACFILTER_UP4_WORK_BUF_SIZE bytes,
										// this buffer is used to keep Up4 filter internal information.
										// Can not be used with others.
										// Buffer address must be 2 bytes alignment.
);

void NuDACFilterEx_Up4Process(
	UINT8 *pu8Up4WorkBuf,				// Buffer address must be 2 bytes alignment.
										// Buffer size is NUDACFILTER_UP4_WORK_BUF_SIZE bytes,
										// this buffer is used to keep Up4 filter internal information.
										// Can not be used with others.
	INT16 i16PCM,						// Input  1 PCM
	INT16 *pi16Up4PCMs					// Return 4 PCMs
);

UINT32 NuDACFilterEx_GetVersion(void);

#ifdef  __cplusplus
}
#endif

#endif	//#ifndef __NUDACFILTEREX_H__

