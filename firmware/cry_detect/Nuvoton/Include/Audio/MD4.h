/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.  *
 *                                                              *
 ****************************************************************/

#ifndef __MD4_H__
#define __MD4_H__

#include "Platform.h"
#include "System/SysInfra.h"
#include "Audio/AudioCommon.h"

#ifdef  __cplusplus
extern "C"
{
#endif

// Version Definition
#define	MD4_MAJOR_NUM 3
#define	MD4_MINOR_NUM 30
#define	MD4_BUILD_NUM 1
#define MD4_VERSION_NUM	_SYSINFRA_VERSION(MD4_MAJOR_NUM, MD4_MINOR_NUM, MD4_BUILD_NUM)

// Constant Definition
#define MD4_DECODE_RESIN_BUFF_SIZE		32	//bytes
#define MD4_DECODE_SAMPLE_PER_FRAME		8		// counts per frame after decoding
#define MD4_DECODE_WORK_BUF_SIZE			48+MD4_DECODE_RESIN_BUFF_SIZE		// bytes
#define MD4_DECODE_TEMP_BUF_SIZE			0		// bytes

UINT32 MD4_DecodeInitiate(		// Return sampling rate. 0 represent failed in initiating.
	UINT8 *pu8DecodeWorkBuf,		// Buffer size is MD4_DECODE_WORK_BUF_SIZE bytes,
															// this buffer is used to keep MD4 decode internal information.
															// Can not be used with others
	UINT8 *pu8DecodeTempBuf,		// this buffer is reserved				
									
									
	UINT32 u32StartAddr,				// The adddress of the first NuSound data.
															// For SPI flash, it is the SPI address.
															// For file with file system, it is the offset from a file
	PFN_AUDIO_DATAREQUEST pfnReadDataCallback	// Function to read MD4 data
);

INT32 MD4_DecodeProcess(			// Return decoded PCM count. 0 represent no PCM decoded
	UINT8 *pu8DecodeWorkBuf,		// Buffer size is MD4_DECODE_WORK_BUF_SIZE bytes,
															// this buffer is used to keep MD4 decode internal information.
															// Can not be used with others
	UINT8 *pu8DecodeTempBuf,		// this buffer is reserved
									
	PINT16 pi16DecodedPcmBuf,		// buffer size is MD4_DECODE_SAMPLE_PER_FRAME*2 bytes,
															// this buffer is used to keep decoded PCM data.
	PFN_AUDIO_DATAREQUEST pfnReadDataCallback,		// Function to read MD4 data
	PFN_AUDIO_USREVENT PFN_AUDIO_USREVENT	// Function to handle user event
);

BOOL MD4_DecodeIsEnd(					// TRUE: decode to end. FALSE: can continue decodeing
	UINT8 *pu8DecodeWorkBuf			// Buffer size is MD4_DECODE_WORK_BUF_SIZE bytes,
															// this buffer is used to keep MD4 decode internal information.
															// Can not be used with others
);

void MD4_DecodeSampleCounts(	// Set decoding sample counts of frame, default is MD4_DECODE_SAMPLE_PER_FRAME
	UINT8 *pu8DecodeWorkBuf,  	// Buffer size is MD4_DECODE_WORK_BUF_SIZE bytes,
															// this buffer is used to keep MD4 decode internal information.
															// Can not be used with others
	UINT8 u8SampleCounts				// Decoding sample counts of one freame
);

UINT32 MD4_GetVersion(void);

#ifdef  __cplusplus
}
#endif

#endif //#ifndef __MD4_H__


