/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.  *
 *                                                              *
 ****************************************************************/

#ifndef __DIRDETECT_H__
#define __DIRDETECT_H__

#include "Platform.h"
#include "System/SysInfra.h"

#ifdef  __cplusplus
extern "C"
{
#endif

#define	DIRDETECT_MAJOR_NUM 3
#define	DIRDETECT_MINOR_NUM 10
#define	DIRDETECT_BUILD_NUM 001
#define DIRDETECT_VERSION_NUM	_SYSINFRA_VERSION(DIRDETECT_MAJOR_NUM, DIRDETECT_MINOR_NUM, DIRDETECT_BUILD_NUM)

#define SAMPLE_FREQUENCY 16000
#define DOWN_SAMPLE_SIZE 3
#define CYCLE_REPEAT_COUNT 4//4
#define ADC_FREQENCY (SAMPLE_FREQUENCY*DOWN_SAMPLE_SIZE*CYCLE_REPEAT_COUNT)

#define CORRELATION_SIZE 128//256
#define CORR_SHIFT_MAX 5
#define ADC_BUFFER_SIZE (CORRELATION_SIZE+CORR_SHIFT_MAX*2) // 266

#define ENERGY_BURST_MIN_CHECK 20//40
//#define ENERGY_FRAME_MAX_CHECK 20//40

#define ENG_BURST_WIDTH 16//32

#define DETECT_DELAY 18//6

#define MATRIX_COUNT_MAX 149

/******************************************************************************************
 * Description:
 * 	Version control API.
 * Arguments:
 *  None.
 ******************************************************************************************/ 
UINT32 DirDetect_GetVersion
	(void
	);

/******************************************************************************************
 * Description:
 * 	Version control API.
 * Arguments:
 *  None.
 ******************************************************************************************/ 
INT32 DirDetect_Judge(
	PINT16 pi16PcmA,
	PINT16 pi16PcmB,
	PINT16 pi16PcmC
	);

#ifdef  __cplusplus
}
#endif

#endif
