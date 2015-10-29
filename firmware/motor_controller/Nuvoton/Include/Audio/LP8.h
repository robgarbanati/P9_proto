/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.  *
 *                                                              *
 ****************************************************************/

#ifndef __LP8_H__
#define __LP8_H__

#include "Platform.h"
#include "System/SysInfra.h"
#include "Audio/AudioCommon.h"

#ifdef  __cplusplus
extern "C"
{
#endif

// Version Definition
#define	LP8_MAJOR_NUM 3
#define	LP8_MINOR_NUM 30
#define	LP8_BUILD_NUM 1
#define LP8_VERSION_NUM	_SYSINFRA_VERSION(LP8_MAJOR_NUM, LP8_MINOR_NUM, LP8_BUILD_NUM)

// Constant Definition
#define LP8_DECODE_RESIN_BUFF_SIZE		32		//bytes	
#define LP8_DECODE_SAMPLE_PER_FRAME		8		  // counts per frame after decoding
#define LP8_DECODE_WORK_BUF_SIZE			40+LP8_DECODE_RESIN_BUFF_SIZE		// bytes
#define LP8_DECODE_TEMP_BUF_SIZE			0		  // bytes

UINT32 LP8_DecodeInitiate(		// Return sampling rate. 0 represent failed in initiating.
	UINT8 *pu8DecodeWorkBuf,		// Buffer size is LP8_DECODE_WORK_BUF_SIZE bytes,
															// this buffer is used to keep LP8 decode internal information.
															// Can not be used with others
	UINT8 *pu8DecodeTempBuf,		// this buffer is reserved
									
	UINT32 u32StartAddr,			  // The adddress of the first NuSound data.
															// For SPI flash, it is the SPI address.
															// For file with file system, it is the offset from a file
	PFN_AUDIO_DATAREQUEST pfnReadDataCallback	// Function to read LP8 data
);

INT32 LP8_DecodeProcess(			// Return decoded PCM count. 0 represent no PCM decoded
	UINT8 *pu8DecodeWorkBuf,		// Buffer size is LP8_DECODE_WORK_BUF_SIZE bytes,
															// this buffer is used to keep LP8 decode internal information.
															// Can not be used with others
	UINT8 *pu8DecodeTempBuf,		// this buffer is reserved
									
	PINT16 pi16DecodedPcmBuf,		// buffer size is LP8_DECODE_SAMPLE_PER_FRAME*2 bytes,
															// this buffer is used to keep decoded PCM data.
	PFN_AUDIO_DATAREQUEST pfnReadDataCallback,		// Function to read LP8 data
	PFN_AUDIO_USREVENT PFN_AUDIO_USREVENT					// Function to handle user event
);

BOOL LP8_DecodeIsEnd(					// TRUE: decode to end. FALSE: can continue decodeing
	UINT8 *pu8DecodeWorkBuf			// Buffer size is LP8_DECODE_WORK_BUF_SIZE bytes,
															// this buffer is used to keep LP8 decode internal information.
															// Can not be used with others
);

void LP8_DecodeSampleCounts(	// Set decoding sample counts of frame, default is LP8_DECODE_SAMPLE_PER_FRAME
	UINT8 *pu8DecodeWorkBuf,  	// Buffer size is LP8_DECODE_WORK_BUF_SIZE bytes,
															// this buffer is used to keep LP8 decode internal information.
															// Can not be used with others
	UINT8 u8SampleCounts				// Decoding sample counts of one freame
);

UINT32 LP8_GetVersion(void);

#ifdef  __cplusplus
}
#endif

#endif //#ifndef __LP8_H__


