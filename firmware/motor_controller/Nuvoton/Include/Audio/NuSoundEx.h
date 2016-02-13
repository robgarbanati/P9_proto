/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.  *
 *                                                              *
 ****************************************************************/

#ifndef __NUSOUNDEX_H__
#define __NUSOUNDEX_H__

#include "Platform.h"
#include "System/SysInfra.h"
#include "Audio/AudioCommon.h"

#ifdef  __cplusplus
extern "C"
{
#endif

#define	NUSOUNDEX_MAJOR_NUM 3
#define	NUSOUNDEX_MINOR_NUM 30
#define	NUSOUNDEX_BUILD_NUM 1
#define NUSOUNDDEEX_VERSION_NUM	_SYSINFRA_VERSION(NUSOUNDEX_MAJOR_NUM, NUSOUNDEX_MINOR_NUM, NUSOUNDEX_BUILD_NUM)

#define NUSOUNDEX_DECODE_SAMPLE_PER_FRAME	128		// counts per frame after decoding
#define NUSOUNDEX_DECODE_WORK_BUF_SIZE		1100	// bytes
#define NUSOUNDEX_DECODE_TEMP_BUF_SIZE		284		// bytes

/******************************************************************************************
 * Description:
 * 	Version control API.
 * Arguments:
 *  None.
 ******************************************************************************************/ 
UINT32 NuSoundEx_GetVersion
	(void
	);

/***************************************************************************
 * Description:
 * 	
 * Arguments:
 *  
 * Return:
 *		
 ***************************************************************************/ 
UINT32 NuSoundEx_DecodeInitiate(
	UINT8 *pu8WorkBuf,
	UINT8 *pu8TempBuf,
	UINT32 u32StartAddr,
	PFN_AUDIO_DATAREQUEST pfnReadDataCallback
	);

/***************************************************************************
 * Description:
 * 	
 * Arguments:
 *  
 * Return:
 *		
 ***************************************************************************/ 
INT32 NuSoundEx_DecodeProcess(
	UINT8 *pu8WorkBuf,
	UINT8 *pu8TempBuf,
	PINT16 pi16DecodedPcmBuf,
	PFN_AUDIO_DATAREQUEST pfnReadDataCallback,
	PFN_AUDIO_USREVENT pfnUserEventCallback
	);

/***************************************************************************
 * Description:
 * 	
 * Arguments:
 *  
 * Return:
 *		
 ***************************************************************************/ 
BOOL NuSoundEx_DecodeIsEnd(
	UINT8 *pu8WorkBuf
	);

#ifdef  __cplusplus
}
#endif

#endif //#ifndef __NUSOUNDEX_H__


