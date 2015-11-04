/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.  *
 *                                                              *
 ****************************************************************/

#ifndef __NUSOUND_H__
#define __NUSOUND_H__

#include "Platform.h"
#include "System/SysInfra.h"

#ifdef  __cplusplus
extern "C"
{
#endif

#define	NUSOUND_MAJOR_NUM 1
#define	NUSOUND_MINOR_NUM 30
#define	NUSOUND_BUILD_NUM 001
#define NUSOUND_VERSION_NUM	_SYSINFRA_VERSION(NUSOUND_MAJOR_NUM, NUSOUND_MINOR_NUM, NUSOUND_BUILD_NUM)

//#define FAAD_MIN_STREAMSIZE 768
#define FAAD_MIN_STREAMSIZE 660
//#define NEW_FAAD_MIN_STREAMSIZE (long)(440)
//#define MY_FAAD_MIN_STREAMSIZE (long)(192) // max 108 for these data
#define MY_FAAD_MIN_STREAMSIZE (long)(1) // max 108 for these data

#define DAC_BUF_LEN 128
#define DAC_BUF_LEN2 (DAC_BUF_LEN*2)
#define DAC_DATA_BUF_NUM 2

#define NUSOUND_CHANNEL_BUFFER_SIZE (long)(471)

typedef UINT32 (*PFN_NUSOUNDCALLBACK_USRDATAREQUEST)(INT32 i32Start,UINT8 *pu8buf,INT32 i32count);
typedef UINT32 (*PFN_NUSOUND_USREVENT)(UINT16 u16Index, UINT16 u16SubIndex);

/******************************************************************************************
 * Description:
 * 	Version control API.
 * Arguments:
 *  None.
 ******************************************************************************************/ 
UINT32 NuSound_GetVersion
	(void
	);

/***************************************************************************
 * Description:
 * 	
 * Arguments:
 *  None.
 * Return:
 *	None.	
 ***************************************************************************/ 
void NuSound_Initialize(
	void
	);

/***************************************************************************
 * Description:
 * 	
 * Arguments:
 *  
 * Return:
 *	None.	
 ***************************************************************************/ 
void NuSound_RegisterCallback(
	PFN_NUSOUNDCALLBACK_USRDATAREQUEST pfnUserDataReguestCallBack,
	PFN_NUSOUND_USREVENT pfnUsrEvent_Callback
	);

/***************************************************************************
 * Description:
 * 	
 * Arguments:
 *  
 * Return:
 *	None.	
 ***************************************************************************/ 
void NuSound_Memory_Init(
	INT32 i32ChannelMax,
	PINT8 pi8Ptr
	);

/***************************************************************************
 * Description:
 * 	
 * Arguments:
 *  
 * Return:
 *	None.	
 ***************************************************************************/ 
INT32 NuSOund_Get_ROM_Address(
	INT16 i16AudioID
	);

/***************************************************************************
 * Description:
 * 	
 * Arguments:
 *  
 * Return:
 *		
 ***************************************************************************/ 
INT32 NuSound_Channel_Init(
	INT32 i32Channel,
	UINT8 *pu8Buf,
	UINT8 (*pu8PCMData)[FAAD_MIN_STREAMSIZE],
	INT32 *pi32Start
	);

/***************************************************************************
 * Description:
 * 	
 * Arguments:
 *  
 * Return:
 *		
 ***************************************************************************/ 
INT32 NuSound_Decode_Frame_Main(
	INT32 i32Channel,
	PINT16 pi16Dest
	);

/***************************************************************************
 * Description:
 * 	
 * Arguments:
 *  
 * Return:
 *		
 ***************************************************************************/ 
INT32 NuSound_End(
	INT32 i32channel
	);

#ifdef  __cplusplus
}
#endif

#endif //#ifndef __NUSOUND_H__


