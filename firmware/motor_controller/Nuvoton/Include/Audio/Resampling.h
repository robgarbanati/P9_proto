/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) Nuvoton Technology Corp. All rights reserved.                                              */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#ifndef __RESAMPLING_H__
#define __RESAMPLING_H__

#include "Platform.h"
#include "System/SysInfra.h"

#ifdef  __cplusplus
extern "C"
{
#endif

// Version number
#define RESAMPLING_MAJOR_NUM	3
#define RESAMPLING_MINOR_NUM	01
#define RESAMPLING_BUILD_NUM	001			 
#define RESAMPLING_VERSION_NUM	_SYSINFRA_VERSION(RESAMPLING_MAJOR_NUM, RESAMPLING_MINOR_NUM, RESAMPLING_BUILD_NUM)

//---------------------------------------------------------------------------------------------------------
// Function: 	Resampling_Reset
// Parameter:
// Description: 	
//				Reset internal FIFO buffer and variables.			
//---------------------------------------------------------------------------------------------------------
void Resampling_Reset(void);

//---------------------------------------------------------------------------------------------------------
// Function: 	Resampling_UpX2
// Parameter:
//				i16InputPCM		    [in]		Input 2's complement 16-bits PCM value. 
//				pi16OutPCMBuff		[out]		Save two samples after resampling.
// Description: 	
//				This API up-samples one sample PCM as two samples. User must initialize and control buffer
//				to save two samples.  		
//---------------------------------------------------------------------------------------------------------
void Resampling_UpX2(
	 INT16	i16InputPCM,
	 PINT16	pi16OutPCMBuff
);

//---------------------------------------------------------------------------------------------------------
// Function: 	ReSampling_DownX2
// Parameter:
//				i16InputPCM1		[in]		Input first 2's complement 16-bits PCM value. 
//				i16InputPCM2		[in]		Input second 2's complement 16-bits PCM value.
// Description: 	
//				This API down-samples two sample PCM as one samples. 
// Return:      one sample PCM.  			
//---------------------------------------------------------------------------------------------------------
INT16 Resampling_DownX2(
	 INT16	i16InputPCM1,
	 INT16	i16InputPCM2
);

//---------------------------------------------------------------------------------------------------------
// Get Resampleing library version
//---------------------------------------------------------------------------------------------------------
UINT32
Resampleing_GetVersion(void);

#ifdef  __cplusplus
}
#endif

#endif	//__RESAMPLING_H__
