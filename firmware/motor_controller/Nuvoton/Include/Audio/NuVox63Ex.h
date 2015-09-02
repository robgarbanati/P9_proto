/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.  *
 *                                                              *
 ****************************************************************/

#ifndef __NUVOX63EX_H__
#define __NUVOX63EX_H__
   
#include "Platform.h"
#include "System/SysInfra.h"
#include "Audio/AudioCommon.h"

#ifdef  __cplusplus
extern "C"
{
#endif

// Version Definition
#define NUVOX63EX_MAJOR_NUM   3
#define NUVOX63EX_MINOR_NUM   30
#define NUVOX63EX_BUILD_NUM   1
#define NUVOX63EX_VERSION_NUM   _SYSINFRA_VERSION(NUVOX63EX_MAJOR_NUM, NUVOX63EX_MINOR_NUM, NUVOX63EX_BUILD_NUM)

// Constant Definition 
#define NUVOX63EX_DECODE_SAMPLE_PER_FRAME	240   	// counts per frame after decoding
#define NUVOX63EX_DECODE_WORK_BUF_SIZE		(170*2)	// bytes
#define NUVOX63EX_DECODE_TEMP_BUF_SIZE		(584*2)	// bytes


UINT32 NuVox63Ex_DecodeInitiate(	// Return sampling rate. 0 represent failed in initiating.
	UINT8 *pu8DecodeWorkBuf,		// Buffer size is NUVOX63EX_DECODE_WORK_BUF_SIZE bytes,
									// this buffer is used to keep NuVox63 decode internal information.
									// Can not be used with others
	UINT8 *pu8DecodeTempBuf,		// Buffer size is NUVOX63EX_DECODE_TEMP_BUF_SIZE bytes,
									// this buffer is used to keep NUVOX63 decode temporary information.
									// Can be used with others
	UINT32 u32StartAddr,			// The adddress of the first NuVox63 data.
									// For SPI flash, it is the SPI address.
									// For file with file system, it is the offset from a file
	PFN_AUDIO_DATAREQUEST pfnReadDataCallback	// Function to read NUVOX63 data
);

INT32 NuVox63Ex_DecodeProcess(		// Return decoded PCM count. 0 represent no PCM decoded
	UINT8 *pu8DecodeWorkBuf,		// Buffer size is NUVOX63EX_DECODE_WORK_BUF_SIZE bytes,
									// this buffer is used to keep NuVox63 decode internal information.
									// Can not be used with others
	UINT8 *pu8DecodeTempBuf,		// Buffer size is NUVOX63EX_DECODE_TEMP_BUF_SIZE bytes,
									// this buffer is used to keep NuVox63 decode temporary information.
									// Can be used with others
	PINT16 pi16DecodedPcmBuf,		// buffer size is NUVOX63EX_DECODE_SAMPLE_PER_FRAME*2 bytes,
									// this buffer is used to keep decoded PCM data.
	PFN_AUDIO_DATAREQUEST pfnReadDataCallback,	// Function to read NuVox63 data
	PFN_AUDIO_USREVENT pfnUserEventCallback		// Function to handle user event
);

BOOL NuVox63Ex_DecodeIsEnd(			// TRUE: decode to end. FALSE: can continue decodeing
	UINT8 *pu8DecodeWorkBuf			// Buffer size is NUVOX63EX_DECODE_WORK_BUF_SIZE bytes,
									// this buffer is used to keep NuVox63 decode internal information.
									// Can not be used with others
);

UINT32 NuVox63Ex_GetVersion(void);

#ifdef  __cplusplus
}
#endif

#endif	//#ifndef __NUVOX63EX_H__
