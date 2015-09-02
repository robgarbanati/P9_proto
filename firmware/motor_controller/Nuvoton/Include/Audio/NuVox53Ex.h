/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.  *
 *                                                              *
 ****************************************************************/

#ifndef __NUVOX53EX_H__
#define __NUVOX53EX_H__
   
#include "Platform.h"
#include "System/SysInfra.h"
#include "Audio/AudioCommon.h"

#ifdef  __cplusplus
extern "C"
{
#endif

// Version Definition
#define NUVOX53EX_MAJOR_NUM   3
#define NUVOX53EX_MINOR_NUM   30
#define NUVOX53EX_BUILD_NUM   1
#define NUVOX53EX_VERSION_NUM   _SYSINFRA_VERSION(NUVOX53EX_MAJOR_NUM, NUVOX53EX_MINOR_NUM, NUVOX53EX_BUILD_NUM)

// Constant Definition 
#define NUVOX53EX_DECODE_SAMPLE_PER_FRAME	240   	// counts per frame after decoding
#define NUVOX53EX_DECODE_WORK_BUF_SIZE		(170*2)	// bytes
#define NUVOX53EX_DECODE_TEMP_BUF_SIZE		(584*2)	// bytes


UINT32 NuVox53Ex_DecodeInitiate(	// Return sampling rate. 0 represent failed in initiating.
	UINT8 *pu8DecodeWorkBuf,		// Buffer size is NUVOX53EX_DECODE_WORK_BUF_SIZE bytes,
									// this buffer is used to keep NuVox53 decode internal information.
									// Can not be used with others
	UINT8 *pu8DecodeTempBuf,		// Buffer size is NUVOX53EX_DECODE_TEMP_BUF_SIZE bytes,
									// this buffer is used to keep NuVox53 decode temporary information.
									// Can be used with others
	UINT32 u32StartAddr,			// The adddress of the first NuVox53 data.
									// For SPI flash, it is the SPI address.
									// For file with file system, it is the offset from a file
	PFN_AUDIO_DATAREQUEST pfnReadDataCallback	// Function to read NuVox53 data
);

INT32 NuVox53Ex_DecodeProcess(		// Return decoded PCM count. 0 represent no PCM decoded
	UINT8 *pu8DecodeWorkBuf,		// Buffer size is NUVOX53EX_DECODE_WORK_BUF_SIZE bytes,
									// this buffer is used to keep NuVox53 decode internal information.
									// Can not be used with others
	UINT8 *pu8DecodeTempBuf,		// Buffer size is NUVOX53EX_DECODE_TEMP_BUF_SIZE bytes,
									// this buffer is used to keep NuVox53 decode temporary information.
									// Can be used with others
	PINT16 pi16DecodedPcmBuf,		// buffer size is NUVOX53EX_DECODE_SAMPLE_PER_FRAME*2 bytes,
									// this buffer is used to keep decoded PCM data.
	PFN_AUDIO_DATAREQUEST pfnReadDataCallback,	// Function to read NuVox53 data
	PFN_AUDIO_USREVENT pfnUserEventCallback		// Function to handle user event
);

BOOL NuVox53Ex_DecodeIsEnd(			// TRUE: decode to end. FALSE: can continue decodeing
	UINT8 *pu8DecodeWorkBuf			// Buffer size is NUVOX53EX_DECODE_WORK_BUF_SIZE bytes,
									// this buffer is used to keep NuVox53 decode internal information.
									// Can not be used with others
);

UINT32 NuVox53Ex_GetVersion(void);

#ifdef  __cplusplus
}
#endif

#endif	//#ifndef __NUVOX53EX_H__
