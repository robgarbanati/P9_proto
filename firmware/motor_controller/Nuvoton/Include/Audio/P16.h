/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.  *
 *                                                              *
 ****************************************************************/

#ifndef __P16_H__
#define __P16_H__

#include "Platform.h"
#include "System/SysInfra.h"
#include "Audio/AudioCommon.h"

#ifdef  __cplusplus
extern "C"
{
#endif

// Version Definition
#define	P16_MAJOR_NUM 3
#define	P16_MINOR_NUM 30
#define	P16_BUILD_NUM 1
#define P16_VERSION_NUM	_SYSINFRA_VERSION(P16_MAJOR_NUM, P16_MINOR_NUM, P16_BUILD_NUM)

// Constant Definition 
#define P16_DECODE_RESIN_BUF_SIZE		16

#define P16_DECODE_SAMPLE_PER_FRAME		8								// counts per frame after decoding
#define P16_DECODE_WORK_BUF_SIZE		(16+P16_DECODE_RESIN_BUF_SIZE)	// bytes
#define P16_DECODE_TEMP_BUF_SIZE		0								// bytes

UINT32 P16_DecodeInitiate(			// Return sampling rate. 0 represent failed in initiating.
	UINT8 *pu8DecodeWorkBuf,		// Buffer size is P16_DECODE_WORK_BUF_SIZE bytes,
									// this buffer is used to keep P16 decode internal information.
									// Can not be used with others
	UINT8 *pu8DecodeTempBuf,		// this buffer shoule be NULL, because no need
	UINT32 u32StartAddr,			// The adddress of the first NuSound data.
									// For SPI flash, it is the SPI address.
									// For file with file system, it is the offset from a file
	PFN_AUDIO_DATAREQUEST pfnReadDataCallback	// Function to read P16 data
);

INT32 P16_DecodeProcess(			// Return decoded PCM count. 0 represent no PCM decoded
	UINT8 *pu8DecodeWorkBuf,		// Buffer size is P16_DECODE_WORK_BUF_SIZE bytes,
									// this buffer is used to keep P16 decode internal information.
									// Can not be used with others
	UINT8 *pu8DecodeTempBuf,		// this buffer shoule be NULL, because no need									
	PINT16 pi16DecodedPcmBuf,		// buffer size is P16_DECODE_SAMPLE_PER_FRAME*2 bytes,
									// this buffer is used to keep decoded PCM data.
	PFN_AUDIO_DATAREQUEST pfnReadDataCallback,		// Function to read P16 data
	PFN_AUDIO_USREVENT pfnUserEventCallback			// Function to handle user event
);

BOOL P16_DecodeIsEnd(			// TRUE: decode to end. FALSE: can continue decodeing
	UINT8 *pu8DecodeWorkBuf		// Buffer size is P16_DECODE_WORK_BUF_SIZE bytes,
								// this buffer is used to keep P16 decode internal information.
								// Can not be used with others
);

void P16_DecodeSampleCounts(	// Set decoding sample counts of frame, default is P16_DECODE_SAMPLE_PER_FRAME
	UINT8 *pu8DecodeWorkBuf,  	// Buffer size is P16_DECODE_WORK_BUF_SIZE bytes,
								// this buffer is used to keep P16 decode internal information.
								// Can not be used with others
	UINT8 u8DecodeSampleCounts	// Decoding sample counts of one freame
);

UINT32 P16_GetVersion(void);

#ifdef  __cplusplus
}
#endif

#endif //#ifndef __P16_H__


