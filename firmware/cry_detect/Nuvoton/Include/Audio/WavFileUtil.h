/********************************************************************************
 *                                                              								*
 * Copyright (c) Nuvoton Technology Corp. All rights reserved. 								*
 *                                                              								*
 ********************************************************************************/
#ifndef	__WAVFILEUTIL_H__
#define	__WAVFILEUTIL_H__

#include "Platform.h"
#include "System/SysInfra.h"

#ifdef	__cplusplus
extern "C"
{
#endif

//-----------------------------------------------------------------------------
// Version information
//	   -----------------------------------------------------------------
//     |      31:24       |     23:16    |     15:8     |     7:0      |
//     -----------------------------------------------------------------
//     | Should all be 0s | Major number | Minor number | Build number |
//     -----------------------------------------------------------------
//     Major number: Module+"_"+MAJOR+"_"+NUM
//     Minor number: Module+"_"+MIMOR+"_"+NUM
//     Build number: Module+"_"+BUILD+"_"+NUM
//-----------------------------------------------------------------------------
#define WAVFILEUTIL_MAJOR_NUM		2
#define WAVFILEUTIL_MINOR_NUM		00
#define WAVFILEUTIL_BUILD_NUM		1

#define WAVFILEUTIL_VERSION_NUM _SYSINFRA_VERSION(WAVFILEUTIL_MAJOR_NUM, WAVFILEUTIL_MINOR_NUM, WAVFILEUTIL_BUILD_NUM)

// E_WAVFILEUTIL_INVALIDADDR			Agrument has invalid address
// E_WAVFILEUTIL_INVALIDHEADER			RIFF or WAVE format is invalid
// E_WAVFILEUTIL_GETFILEINFOFAIL		Read wav file information error
// E_WAVFILEUTIL_FORMATNOTSUPPORT		Format not support
// E_WAVFILEUTIL_WRITEDATAFAIL			Write chunk information fail
// E_WAVFILEUTIL_SPACENOTENOUGH			Can't write data into target.
#define E_WAVFILEUTIL_INVALIDADDR		_SYSINFRA_ERRCODE(TRUE, MODULE_ID_WAVFILEUTIL, 1)
#define E_WAVFILEUTIL_INVALIDHEADER		_SYSINFRA_ERRCODE(TRUE, MODULE_ID_WAVFILEUTIL, 2)
#define E_WAVFILEUTIL_GETFILEINFOFAIL	_SYSINFRA_ERRCODE(TRUE, MODULE_ID_WAVFILEUTIL, 3)

#define E_WAVFILEUTIL_FORMATNOTSUPPORT	_SYSINFRA_ERRCODE(TRUE, MODULE_ID_WAVFILEUTIL, 11)
#define E_WAVFILEUTIL_WRITEDATAFAIL		_SYSINFRA_ERRCODE(TRUE, MODULE_ID_WAVFILEUTIL, 12)
#define E_WAVFILEUTIL_SPACENOTENOUGH	_SYSINFRA_ERRCODE(TRUE, MODULE_ID_WAVFILEUTIL, 13)

typedef UINT32 (*PFN_WAVFILEUTIL_GETDATA)(void *pDesAddr, UINT32 u32Position, UINT32 u32ByteNum, void *pVoid);
typedef UINT32 (*PFN_WAVFILEUTIL_SETDATA)(UINT32 u32Position, void *pSrcAddr, UINT32 u32ByteNum, void *pVoid);

#define WAV_FORMAT_LEN		14
#define	WAV_FORMAT_EX_LEN	18	//sizeof(S_WavFormatEx)
#define IMA_WAV_FORMAT_LEN	20	//sizeof(S_IMAWavFormat)

typedef enum tagWavFormatTag
{
	eWAVE_FORMAT_UNKNOWN	=	0x0000,
	eWAVE_FORMAT_PCM		=	0x0001,
	eWAVE_FORMAT_IMA_ADPCM	=	0x0011,
	eWAVE_FORMAT_DVI_ADPCM	=	0x0011
} E_WavFormatTag;

typedef struct tagWavFormat
{
    UINT16	eFormatTag;	//Data format
    UINT16  u16Channels;		//Channel number
    UINT32	u32SamplesPerSec;	//Sample rate
    UINT32	u32AvgBytesPerSec;	//Byte rate
    UINT16	u16BlockAlign;	 	//Alias BlockAlign = (Channel * BitPerSample) / 8
} S_WavFormat;

typedef struct	tagIMAWavFormat
{
    UINT16	eFormatTag;	//Data format
    UINT16  u16Channels;	   	//Channel number
    UINT32	u32SamplesPerSec;	//Sample rate
    UINT32	u32AvgBytesPerSec;	//Byte rate
    UINT16	u16BlockAlign;		//Alias BlockAlign = (Channel * BitPerSample) / 8
    UINT16	u16BitsPerSample;	//Bits per sample; encoded sample
    UINT16  u16CbSize;			//??
	UINT16	u16SamplesPerBlock;	//If data format is IMA ADPCM, this field must be used
} S_IMAWavFormat;

typedef struct tagWavFormatEx{
    E_WavFormatTag	eFormatTag; 		//Data format
    UINT16  u16Channels;		//Channel number
    UINT32	u32SamplesPerSec; 	//Sample rate
    UINT32	u32AvgBytesPerSec; 	//Byte rate
    UINT16	u16BlockAlign;		//Alias BlockAlign = (Channel * BitPerSample) / 8
    UINT16	u16BitsPerSample;   //Bits per sample; encoded sample
    UINT16  u16CbSize;			//??
} S_WavFormatEx;

typedef struct tagWavFileWriteInfo
{	
	PVOID	pvFileHandle;
	PFN_WAVFILEUTIL_SETDATA pfnSetDataCallback;	// Get data callback function, application must fill it
	UINT32	u32FileAddr;						// File position, application must fill it
	UINT32	u32FileOffset;						// File relative position
		
	UINT32	u32RiffDataLenFilePos;
	UINT32	u32DataChunkLenFilePos;
} S_WavFileWriteInfo;

typedef struct tagWavFileReadInfo
{	
	PVOID	pvFileHandle;
	PFN_WAVFILEUTIL_GETDATA pfnGetDataCallback;	// Get data callback function, application must fill it
	UINT32	u32FileAddr;						// File position, application must fill it
	UINT32	u32FileOffset;						// File relative position
				
	UINT32	u32RiffDataLen;			//Data length in RIFF chunk: WAVEID (4 bytes) + WAV chunk size
	UINT32	u32FormatFilePos;		//Format chunk position
	UINT32	u32FormatDataLen;		//Format chunk size: 16 or 18 or 40
	UINT32	u32DataFilePos;			//Data chunk position
	UINT32	u32DataLen;				//Data chunk size
	UINT32	u32DataPos;				//Current access position in data chunk
	UINT16	u16SamplesPerBlock;
	UINT16	u16BytesPerBlock;
	S_WavFormatEx	sWavFormatEx;
} S_WavFileReadInfo;

//////////////////////////////////////////////////////////////////////////////////////////////
// Description:
//	Get version inforamtion.
// Parameter:
//	
// Return:
//	
//////////////////////////////////////////////////////////////////////////////////////////////
UINT32
WavFileUtil_GetVersion(void);

//////////////////////////////////////////////////////////////////////////////////////////////
// Description:
//	
// Parameter:
//	
// Return:
//	
//////////////////////////////////////////////////////////////////////////////////////////////
UINT32
WavFileUtil_Read_Initialize(
	S_WavFileReadInfo*	psInfo
	);

//////////////////////////////////////////////////////////////////////////////////////////////
// Description:
//	
// Parameter:
//	
// Return:
//	
//////////////////////////////////////////////////////////////////////////////////////////////
UINT32
WavFileUtil_Read_GetFormat(
	S_WavFileReadInfo*	psInfo,
	E_WavFormatTag*	peWavFormatTag,
	PUINT16	pu16Channels,
	PUINT32	pu32SamplingRate,
	PUINT16	pu16BitsPerSample
	);

//////////////////////////////////////////////////////////////////////////////////////////////
// Description:
//	
// Parameter:
//	
// Return:
//	
//////////////////////////////////////////////////////////////////////////////////////////////
UINT32
WavFileUtil_Read_GetFormatBlockInfo(
	S_WavFileReadInfo*	psInfo,
	PUINT16	pu16SamplesPerBlock,
	PUINT16	pu16BytesPerBlock
	);

//////////////////////////////////////////////////////////////////////////////////////////////
// Description:
//	
// Parameter:
//	
// Return:
//	
//////////////////////////////////////////////////////////////////////////////////////////////
UINT32
WavFileUtil_Read_GetTimeLength(
	S_WavFileReadInfo*	psInfo
	);

//////////////////////////////////////////////////////////////////////////////////////////////
// Description:
//	
// Parameter:
//	
// Return:
//	
//////////////////////////////////////////////////////////////////////////////////////////////
UINT32
WavFileUtil_Read_GetTimePosition(
	S_WavFileReadInfo*	psInfo
	);

//////////////////////////////////////////////////////////////////////////////////////////////
// Description:
//	
// Parameter:
//	
// Return:
//	
//////////////////////////////////////////////////////////////////////////////////////////////
UINT32
WavFileUtil_Read_ReadData(
	S_WavFileReadInfo*	psInfo,
	BYTE*	pbyData,
	UINT32	u32DataSize
	);

//////////////////////////////////////////////////////////////////////////////////////////////
// Description:
//	
// Parameter:
//	
// Return:
//	
//////////////////////////////////////////////////////////////////////////////////////////////
void WavFileUtil_Read_Finish(
	S_WavFileReadInfo*	psInfo
	);


//////////////////////////////////////////////////////////////////////////////////////////////
// Description:
//	
// Parameter:
//	
// Return:
//	
//////////////////////////////////////////////////////////////////////////////////////////////
UINT32 
WavFileUtil_Write_Initialize(
	S_WavFileWriteInfo*	psInfo
	);

//////////////////////////////////////////////////////////////////////////////////////////////
// Description:
//	
// Parameter:
//	
// Return:
//	
//////////////////////////////////////////////////////////////////////////////////////////////
UINT32 
WavFileUtil_Write_SetFormat(
	S_WavFileWriteInfo*	psInfo,
	E_WavFormatTag		eWavFormatTag,
	UINT16				u16Channels,
	UINT32				u32SamplingRate,
	UINT16				u16BitsPerSample
	);

//////////////////////////////////////////////////////////////////////////////////////////////
// Description:
//	
// Parameter:
//	
// Return:
//	
//////////////////////////////////////////////////////////////////////////////////////////////
UINT32 
WavFileUtil_Write_WriteData(
	S_WavFileWriteInfo*	psInfo,
	const BYTE*			pbyData,
	UINT32				u32DataSize
	);

//////////////////////////////////////////////////////////////////////////////////////////////
// Description:
//	
// Parameter:
//	
// Return:
//	
//////////////////////////////////////////////////////////////////////////////////////////////
UINT32 
WavFileUtil_Write_Finish(
	S_WavFileWriteInfo*	psInfo
);

#ifdef	__cplusplus
}
#endif

#endif // __WAVFILEUTIL_H__























