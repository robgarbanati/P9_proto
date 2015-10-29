/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.  *
 *                                                              *
 ****************************************************************/

#ifndef __NUONEEX_H__
#define __NUONEEX_H__
   
#include "Platform.h"
#include "System/SysInfra.h"
#include "Audio/AudioCommon.h"

#ifdef  __cplusplus
extern "C"
{
#endif

// Version Definition
#define NUONEEX_MAJOR_NUM   3
#define NUONEEX_MINOR_NUM   30
#define NUONEEX_BUILD_NUM   1
#define NUONEEX_VERSION_NUM   _SYSINFRA_VERSION(NUONEEX_MAJOR_NUM, NUONEEX_MINOR_NUM, NUONEEX_BUILD_NUM)

// Constant Definition
#define NUONEEX_DECODE_SAMPLE_PER_FRAME	     320 	// counts per frame after decoding
#define NUONEEX_DECODE_WORK_BUF_SIZE		(198*2) // bytes
#define NUONEEX_DECODE_TEMP_BUF_SIZE		(640*2)	// bytes

#define NUONEEX_ENCODE_SAMPLE_PER_FRAME      320	// counts of PCMs per frame before encodeing
#define NUONEEX_ENCODE_WORK_BUF_SIZE		(330*2)	// bytes
#define NUONEEX_ENCODE_TEMP_BUF_SIZE		(640*2)	// bytes
#define NUONEEX_ENCODE_DATA_BUF_SIZE		120		// bytes


#define NUONEEX_ENCODE_BIT_PER_FRAME(bps)	((NUONEEX_ENCODE_SAMPLE_PER_FRAME*bps)/10)	// total bits of one encoded frame
#define NUONEEX_ENCODE_BYTE_PER_FRAME(bps)	 (NUONEEX_ENCODE_BIT_PER_FRAME(bps)>>3)	// total bytes of one encoded frame

enum eNuOneExEncodeBPS		// Bit Per Sample after encoding
{
	E_NUONEEX_ENCODE_BPS_05 = NUONEEX_ENCODE_BIT_PER_FRAME( 5),				// 0.5 bit per sample after encoding
	E_NUONEEX_ENCODE_BPS_06 = NUONEEX_ENCODE_BIT_PER_FRAME( 6),				// 0.6 bit per sample after encoding
	E_NUONEEX_ENCODE_BPS_07 = NUONEEX_ENCODE_BIT_PER_FRAME( 7),				// 0.7 bit per sample after encoding
	E_NUONEEX_ENCODE_BPS_08 = NUONEEX_ENCODE_BIT_PER_FRAME( 8),				// 0.8 bit per sample after encoding
	E_NUONEEX_ENCODE_BPS_09 = NUONEEX_ENCODE_BIT_PER_FRAME( 9),				// 0.9 bit per sample after encoding
	E_NUONEEX_ENCODE_BPS_10 = NUONEEX_ENCODE_BIT_PER_FRAME(10),				// 1.0 bit per sample after encoding
	E_NUONEEX_ENCODE_BPS_11 = NUONEEX_ENCODE_BIT_PER_FRAME(11),				// 1.1 bit per sample after encoding
	E_NUONEEX_ENCODE_BPS_12 = NUONEEX_ENCODE_BIT_PER_FRAME(12),				// 1.2 bit per sample after encoding
	E_NUONEEX_ENCODE_BPS_13 = NUONEEX_ENCODE_BIT_PER_FRAME(13),				// 1.3 bit per sample after encoding
	E_NUONEEX_ENCODE_BPS_14 = NUONEEX_ENCODE_BIT_PER_FRAME(14),				// 1.4 bit per sample after encoding
	E_NUONEEX_ENCODE_BPS_15 = NUONEEX_ENCODE_BIT_PER_FRAME(15),				// 1.5 bit per sample after encoding
	E_NUONEEX_ENCODE_BPS_16 = NUONEEX_ENCODE_BIT_PER_FRAME(16),				// 1.6 bit per sample after encoding
	E_NUONEEX_ENCODE_BPS_17 = NUONEEX_ENCODE_BIT_PER_FRAME(17),				// 1.7 bit per sample after encoding
	E_NUONEEX_ENCODE_BPS_18 = NUONEEX_ENCODE_BIT_PER_FRAME(18),				// 1.8 bit per sample after encoding
	E_NUONEEX_ENCODE_BPS_19 = NUONEEX_ENCODE_BIT_PER_FRAME(19),				// 1.9 bit per sample after encoding
	E_NUONEEX_ENCODE_BPS_20 = NUONEEX_ENCODE_BIT_PER_FRAME(20),				// 2.0 bit per sample after encoding
	E_NUONEEX_ENCODE_BPS_21 = NUONEEX_ENCODE_BIT_PER_FRAME(21),				// 2.1 bit per sample after encoding
	E_NUONEEX_ENCODE_BPS_22 = NUONEEX_ENCODE_BIT_PER_FRAME(22),				// 2.2 bit per sample after encoding
	E_NUONEEX_ENCODE_BPS_23 = NUONEEX_ENCODE_BIT_PER_FRAME(23),				// 2.3 bit per sample after encoding
	E_NUONEEX_ENCODE_BPS_24 = NUONEEX_ENCODE_BIT_PER_FRAME(24),				// 2.4 bit per sample after encoding
	E_NUONEEX_ENCODE_BPS_25 = NUONEEX_ENCODE_BIT_PER_FRAME(25),				// 2.5 bit per sample after encoding
	E_NUONEEX_ENCODE_BPS_26 = NUONEEX_ENCODE_BIT_PER_FRAME(26),				// 2.6 bit per sample after encoding
	E_NUONEEX_ENCODE_BPS_27 = NUONEEX_ENCODE_BIT_PER_FRAME(27),				// 2.7 bit per sample after encoding
	E_NUONEEX_ENCODE_BPS_28 = NUONEEX_ENCODE_BIT_PER_FRAME(28),				// 2.8 bit per sample after encoding
	E_NUONEEX_ENCODE_BPS_29 = NUONEEX_ENCODE_BIT_PER_FRAME(29),				// 2.9 bit per sample after encoding
	E_NUONEEX_ENCODE_BPS_30 = NUONEEX_ENCODE_BIT_PER_FRAME(30)				// 3.0 bit per sample after encoding
};

enum eNuOneExEncodeDataSize	// The bytes of one encoded frame
{
	E_NUONEEX_ENCODE_BPS_05_DATASIZE = NUONEEX_ENCODE_BYTE_PER_FRAME( 5),	// The bytes of one encoded frame at 0.5 bit per sample
	E_NUONEEX_ENCODE_BPS_06_DATASIZE = NUONEEX_ENCODE_BYTE_PER_FRAME( 6),	// The bytes of one encoded frame at 0.6 bit per sample
	E_NUONEEX_ENCODE_BPS_07_DATASIZE = NUONEEX_ENCODE_BYTE_PER_FRAME( 7),	// The bytes of one encoded frame at 0.7 bit per sample
	E_NUONEEX_ENCODE_BPS_08_DATASIZE = NUONEEX_ENCODE_BYTE_PER_FRAME( 8),	// The bytes of one encoded frame at 0.8 bit per sample
	E_NUONEEX_ENCODE_BPS_09_DATASIZE = NUONEEX_ENCODE_BYTE_PER_FRAME( 9),	// The bytes of one encoded frame at 0.9 bit per sample
	E_NUONEEX_ENCODE_BPS_10_DATASIZE = NUONEEX_ENCODE_BYTE_PER_FRAME(10),	// The bytes of one encoded frame at 1.0 bit per sample
	E_NUONEEX_ENCODE_BPS_11_DATASIZE = NUONEEX_ENCODE_BYTE_PER_FRAME(11),	// The bytes of one encoded frame at 1.1 bit per sample
	E_NUONEEX_ENCODE_BPS_12_DATASIZE = NUONEEX_ENCODE_BYTE_PER_FRAME(12),	// The bytes of one encoded frame at 1.2 bit per sample
	E_NUONEEX_ENCODE_BPS_13_DATASIZE = NUONEEX_ENCODE_BYTE_PER_FRAME(13),	// The bytes of one encoded frame at 1.3 bit per sample
	E_NUONEEX_ENCODE_BPS_14_DATASIZE = NUONEEX_ENCODE_BYTE_PER_FRAME(14),	// The bytes of one encoded frame at 1.4 bit per sample
	E_NUONEEX_ENCODE_BPS_15_DATASIZE = NUONEEX_ENCODE_BYTE_PER_FRAME(15),	// The bytes of one encoded frame at 1.5 bit per sample
	E_NUONEEX_ENCODE_BPS_16_DATASIZE = NUONEEX_ENCODE_BYTE_PER_FRAME(16),	// The bytes of one encoded frame at 1.6 bit per sample
	E_NUONEEX_ENCODE_BPS_17_DATASIZE = NUONEEX_ENCODE_BYTE_PER_FRAME(17),	// The bytes of one encoded frame at 1.7 bit per sample
	E_NUONEEX_ENCODE_BPS_18_DATASIZE = NUONEEX_ENCODE_BYTE_PER_FRAME(18),	// The bytes of one encoded frame at 1.8 bit per sample
	E_NUONEEX_ENCODE_BPS_19_DATASIZE = NUONEEX_ENCODE_BYTE_PER_FRAME(19),	// The bytes of one encoded frame at 1.9 bit per sample
	E_NUONEEX_ENCODE_BPS_20_DATASIZE = NUONEEX_ENCODE_BYTE_PER_FRAME(20),	// The bytes of one encoded frame at 2.0 bit per sample
	E_NUONEEX_ENCODE_BPS_21_DATASIZE = NUONEEX_ENCODE_BYTE_PER_FRAME(21),	// The bytes of one encoded frame at 2.1 bit per sample
	E_NUONEEX_ENCODE_BPS_22_DATASIZE = NUONEEX_ENCODE_BYTE_PER_FRAME(22),	// The bytes of one encoded frame at 2.2 bit per sample
	E_NUONEEX_ENCODE_BPS_23_DATASIZE = NUONEEX_ENCODE_BYTE_PER_FRAME(23),	// The bytes of one encoded frame at 2.3 bit per sample
	E_NUONEEX_ENCODE_BPS_24_DATASIZE = NUONEEX_ENCODE_BYTE_PER_FRAME(24),	// The bytes of one encoded frame at 2.4 bit per sample
	E_NUONEEX_ENCODE_BPS_25_DATASIZE = NUONEEX_ENCODE_BYTE_PER_FRAME(25),	// The bytes of one encoded frame at 2.5 bit per sample
	E_NUONEEX_ENCODE_BPS_26_DATASIZE = NUONEEX_ENCODE_BYTE_PER_FRAME(26),	// The bytes of one encoded frame at 2.6 bit per sample
	E_NUONEEX_ENCODE_BPS_27_DATASIZE = NUONEEX_ENCODE_BYTE_PER_FRAME(27),	// The bytes of one encoded frame at 2.7 bit per sample
	E_NUONEEX_ENCODE_BPS_28_DATASIZE = NUONEEX_ENCODE_BYTE_PER_FRAME(28),	// The bytes of one encoded frame at 2.8 bit per sample
	E_NUONEEX_ENCODE_BPS_29_DATASIZE = NUONEEX_ENCODE_BYTE_PER_FRAME(29),	// The bytes of one encoded frame at 2.9 bit per sample
	E_NUONEEX_ENCODE_BPS_30_DATASIZE = NUONEEX_ENCODE_BYTE_PER_FRAME(30)	// The bytes of one encoded frame at 3.0 bit per sample
};


UINT32 NuOneEx_DecodeInitiate(	// Return sampling rate. 0 represent failed in initiating.
	UINT8 *pu8DecodeWorkBuf,	// Buffer size is NUONEEX_DECODE_WORK_BUF_SIZE bytes,
								// this buffer is used to keep NuOne decode internal information.
								// Can not be used with others
	UINT8 *pu8DecodeTempBuf,	// Buffer size is NUONEEX_DECODE_TEMP_BUF_SIZE bytes,
								// this buffer is used to keep NuOne decode temporary information.
								// Can be used with others
	UINT32 u32StartAddr,		// The adddress of the first NuOne data.
								// For SPI flash, it is the SPI address.
								// For file with file system, it is the offset from a file
	PFN_AUDIO_DATAREQUEST pfnReadDataCallback	// Function to read NuOne data
);

INT32 NuOneEx_DecodeProcess(	// Return decoded PCM count. 0 represent no PCM decoded
	UINT8 *pu8DecodeWorkBuf,	// Buffer size is NUONEEX_DECODE_WORK_BUF_SIZE bytes,
								// this buffer is used to keep NuOne decode internal information.
								// Can not be used with others
	UINT8 *pu8DecodeTempBuf,	// Buffer size is NUONEEX_DECODE_TEMP_BUF_SIZE bytes,
								// this buffer is used to keep NuOne decode temporary information.
								// Can be used with others
	PINT16 pi16DecodedPcmBuf,	// buffer size is NUONEEX_DECODE_SAMPLE_PER_FRAME*2 bytes,
								// this buffer is used to keep decoded PCM data.
	PFN_AUDIO_DATAREQUEST pfnReadDataCallback,	// Function to read NuOne data
	PFN_AUDIO_USREVENT pfnUserEventCallback		// Function to handle user event
);

BOOL NuOneEx_DecodeIsEnd(		// TRUE: decode to end. FALSE: can continue decodeing
	UINT8 *pu8DecodeWorkBuf		// Buffer size is NUONEEX_DECODE_WORK_BUF_SIZE bytes,
								// this buffer is used to keep NuOne decode internal information.
								// Can not be used with others
);

void NuOneEx_EncodeInitiate(	// Return sampling rate. 0 represent failed in initiating.
	UINT8 *pu8EncodeWorkBuf,	// Buffer size is NUONEEX_ENCODE_WORK_BUF_SIZE bytes,
								// this buffer is used to keep NuOne encode internal information.
								// Can not be used with others
	UINT8 *pu8EncodeTempBuf,	// Buffer size is NUONEEX_ENCODE_TEMP_BUF_SIZE bytes,
								// this buffer is used to keep NuOne encode temporary information.
								// Can be used with others
	S_AUDIOCHUNK_HEADER *pAudioChunkHeader,		// Audio Chunck header will be initiated.
												// The member "u32TotalSize" will be filled with 0xffffffff.
	enum eNuOneExEncodeBPS eBpsID,	// ID of Bit Per Sample after encoding 
	UINT16 u16SampleRate			// Audio sampling rate
);

UINT32 NuOneEx_EncodeProcess(	// Return the lenghth of encoded data
	UINT8 *pu8EncodeWorkBuf,	// Buffer size is NUONEEX_ENCODE_WORK_BUF_SIZE bytes,
								// this buffer is used to keep NuOne encode internal information.
								// Can not be used with others
	UINT8 *pu8EncodeTempBuf,	// Buffer size is NUONEEX_ENCODE_TEMP_BUF_SIZE bytes,
								// this buffer is used to keep NuOne encode temporary information.
								// Can be used with others
	PINT16 pi16PcmBuf,			// buffer size is NUONEEX_ENCODE_SAMPLE_PER_FRAME*2 bytes,
								// this buffer is used to keep PCM data for encoding
	PINT8  pi8EncodeDataBuf		// buffer size is E_NUONEEX_ENCODE_BPS_xx_DATASIZE bytes,
								// xx is the bit rate to encode PCM data.
);

void NuOneEx_EncodeEnd(
	UINT8 *pu8EncodeWorkBuf,					// Buffer size is NUONEEX_ENCODE_WORK_BUF_SIZE bytes,
												// this buffer is used to keep NuOne encode internal information.
												// Can not be used with others
	UINT8 *pu8EncodeTempBuf,					// Buffer size is NUONEEX_ENCODE_TEMP_BUF_SIZE bytes,
												// this buffer is used to keep NuOne encode temporary information.
												// Can be used with others
	S_AUDIOCHUNK_HEADER *pAudioChunkHeader,		// Audio Chunck header will be updated with correct encode data size.
												// The member "u32TotalSize" will be filled with the value of "u32EncodeDataSize."
	UINT32 u32EncodeDataSize
);

UINT32 NuOneEx_GetVersion(void);

#ifdef  __cplusplus
}
#endif

#endif	//#ifndef __NUONEEX_H__
