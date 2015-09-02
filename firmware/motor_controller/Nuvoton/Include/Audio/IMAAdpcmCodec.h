/********************************************************************************
 *                                                              								*
 * Copyright (c) Nuvoton Technology Corp. All rights reserved. 								  *
 *                                                              								*
 ********************************************************************************/
#ifndef __IMAADPCMCODEC_H__
#define __IMAADPCMCODEC_H__

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
#define IMAADPCM_MAJOR_NUM		2
#define IMAADPCM_MINOR_NUM		10
#define IMAADPCM_BUILD_NUM		1

#define IMAADPCM_VERSION_NUM _SYSINFRA_VERSION(IMAADPCM_MAJOR_NUM, IMAADPCM_MINOR_NUM, IMAADPCM_BUILD_NUM)

//E_IMAADPCM_FORMATNOTSUPPORT	Format doesn't support
#define E_IMAADPCM_FORMATNOTSUPPORT	_SYSINFRA_ERRCODE(TRUE, MODULE_ID_IMAADPCM, 1)


typedef UINT32 (*PFN_IMAADPCM_GETDATA)(BYTE*	pbyData, UINT32 u32ByteNum);
typedef UINT32 (*PFN_IMAADPCM_SETDATA)(BYTE*	pbyData, UINT32 u32ByteNum);

typedef struct
{
	BYTE* pbyInBufAddr;
	UINT16 u16InBufSize;
	UINT16 u16InBufIndex;
	PINT16 pi16BufExAddr;
	UINT16 u16BufExSize;
	UINT16 u16BufExSmplCnt;
	UINT16 u16BufExSmplIndex;
	PFN_IMAADPCM_GETDATA pfnGetData;
	PFN_IMAADPCM_SETDATA pfnSetData;
}S_IMAADPCM_BUFMGRINFO;

typedef struct tagIMAAdpcmState 
{
    INT16	i16PrevValue;		/* Previous output value. */
    INT8	i8StepIndex;		/* Index into stepsize table */    

	UINT8   u8Channels;		    /* Channel number. */
	UINT16	u16SamplesPerBlock;	/* Total sample number in per block. */
	UINT16	u16BytesPerBlock;	/* Total byte number in per block. */
	UINT16	u16SampleIndex;		/* Sample index in current block; it is for reduced memory request. */
	UINT8	u8PrevByte;
} S_IMAAdpcmState;

//////////////////////////////////////////////////////////////////////////////////////////////
// Description:
//	Get version inforamtion.	
// Parameter:
//	
// Return:
//	
//////////////////////////////////////////////////////////////////////////////////////////////
UINT32 
IMAAdpcm_GetVersion(void);

//////////////////////////////////////////////////////////////////////////////////////////////
// Description:
//	
// Parameter:
//	
// Return:
//	
//////////////////////////////////////////////////////////////////////////////////////////////
UINT32	
IMAAdpcm_Initialize(
	S_IMAAdpcmState*	psState,
	UINT16	u16Channels,
	UINT32	u32SamplingRate,
	UINT16	u16BitsPerSample
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
IMAAdpcm_DecodeData(
	S_IMAADPCM_BUFMGRINFO *psBufMgrInfo,
	PINT16		pi16OutData,
	UINT32		u32OutSmplNum,
	S_IMAAdpcmState*	psState
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
IMAAdpcm_DecodeBlock(
	BYTE*				pbyInData,
	PINT16				pi16OutData, 
	UINT16				u16SampleCount,
	S_IMAAdpcmState*	psState
);

//////////////////////////////////////////////////////////////////////////////////////////////
// Description:
//	Encode PCM data directly.
// Parameter:
//	pi16Indata[in]:
//		PCM buffer address; function gets PCM data from this buffer.
//	pbyOutdata[out]:
//		IMAADPCM buffer address; function stores encoded data in this buffer.
//	u16Len[in]:
//		buffer length:
//			1. Unit is sample.
//			2. length must be even to avoid nibble issue.
//	psState:
//		IMAAdpcm structure, function will refet it to encode data.
// Return:
//	None.
//////////////////////////////////////////////////////////////////////////////////////////////
void IMAAdpcm_Encode(
	PINT16				pi16Indata, 
	BYTE*				pbyOutdata, 
	UINT16				u16Len, 
	S_IMAAdpcmState*	psState
	);

//////////////////////////////////////////////////////////////////////////////////////////////
// Description:
//	To decode IMAAdpcm data directly.
// Parameter:
//	pbyIndata[in]:
//		IMAADPCM buffer address; function get encoded data in this buffer.
//	pi16Outdata[out]:
//		PCM buffer address; function stores PCM data to this buffer.
//	u16Len[in]:
//		buffer length:
//			1. Unit is sample.
//			2. length must be even to avoid nibble issue.
//	psState[in]:
//		IMAAdpcm structure, function will refet it to decode data.
//	u8ChannelIndex[in]:
//		Reserve; please assign 0.
//	u8ChannelNextDataWordOffset[in]:
//		Reserve; please assign 0.
// Return:
//	None.
//////////////////////////////////////////////////////////////////////////////////////////////
void IMAAdpcm_Decode(
	BYTE*				pbyIndata, 
	PINT16				pi16Outdata, 
	UINT16				u16Len, 
	S_IMAAdpcmState*	psState,
	UINT8				u8ChannelIndex,
	UINT8				u8ChannelNextDataWordOffset
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
IMAAdpcm_EncodeData(
	S_IMAADPCM_BUFMGRINFO *psBufMgrInfo,
	BYTE*				pbyOutData,
	UINT32				u32InSmplNum,
	S_IMAAdpcmState*	psState
	);

#ifdef	__cplusplus
}
#endif

#endif //__IMAADPCMCODEC_H__






