
/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.  *
 *                                                              *
 ****************************************************************/

#ifndef __REVERB_H__
#define __REVERB_H__

#include "Platform.h"

#ifdef  __cplusplus
extern "C"
{
#endif

#define	REVERB_MAJOR_NUM 2
#define	REVERB_MINOR_NUM 00
#define	REVERB_BUILD_NUM 001
#define REVERB_VERSION_NUM	_SYSINFRA_VERSION(REVERB_MAJOR_NUM, REVERB_MINOR_NUM, REVERB_BUILD_NUM)

#define FRAME_LEN		64		// bigger frame will save data movement loading. since will move #IMPULSE_ORDER data every FRAME_LEN.
#define IMPULSE_ORDER	1219	

/******************************************************************************************
 * Description:
 * 	Initialize ReverB.
 * Arguments:
 *  None.
 ******************************************************************************************/ 
void ReverB_Init(void);

/******************************************************************************************
 * Description:
 * Arguments:
 ******************************************************************************************/ 
INT16 ReverB_GetData8(PINT16 pi16OpDataPtr);

/******************************************************************************************
 * Description:
 * Arguments:
 ******************************************************************************************/ 
INT16 ReverB_FeedData64(PINT16 pi16IpDataPtr);

/******************************************************************************************
 * Description:
 * 	Version control API.
 * Arguments:
 *  None.
 ******************************************************************************************/ 
UINT32 ReverB_GetVersion
	(void
	);

#ifdef  __cplusplus
}
#endif

#endif	//#ifndef __REVERB_H__

