/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) Nuvoton Technology Corp. All rights reserved.                                         	   */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#ifndef __SOFTAGC_H__
#define __SOFTAGC_H__

#include "Platform.h"
#include "System/SysInfra.h"

#ifdef  __cplusplus
extern "C"
{
#endif

#define SOFTAGC_MAJOR_NUM		3
#define SOFTAGC_MINOR_NUM		1
#define SOFTAGC_BUILD_NUM		0

#define SOFTAGC_VERSION_NUM _SYSINFRA_VERSION(SOFTAGC_MAJOR_NUM, SOFTAGC_MINOR_NUM, SOFTAGC_BUILD_NUM)

#define	 	AVG_SHIFT		   				(7)
#define		SOFTAGC_FRAME					(1000)			   /*1000 times interrupt, each int = 8 samples*/
#define		SOFTAGC_HOLD_TIME				(SOFTAGC_FRAME>>1)
#define		SOFTAGC_RAMP_UP_TIME			(SOFTAGC_FRAME/20)
#define		SOFTAGC_RAMP_DOWN_TIME			(SOFTAGC_RAMP_UP_TIME/10)

#define		SOFTAGC_SILENCE_RANGE			((2048/25)*(2048/25)>>AVG_SHIFT)
#define		SOFTAGC_PCM_UPPER_BOUNDARY		((1300)*(1300)>>AVG_SHIFT)
#define		SOFTAGC_PCM_LOWER_BOUNDARY		((512-300)*(512-300)>>AVG_SHIFT)


#if (defined(__N572F064__)||defined(__N572F065__))
#define 	SOFTAGC_GAIN_UP_BOUNDARY		DRVADC_PAG2_39DB
#else
#define		SOFTAGC_GAIN_UP_BOUNDARY		DRVADC_PAG1_30DB
#endif

void SoftAGC_Init(
	UINT8 u8Gain1,
	UINT8 u8Gain2
);

void SoftAGC_Process(
	PINT16	i16PCMBuf
);

#ifdef  __cplusplus
}
#endif

#endif //__SOFTAGC_H__
