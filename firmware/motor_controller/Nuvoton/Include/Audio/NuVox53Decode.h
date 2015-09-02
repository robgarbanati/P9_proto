/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.  *
 *                                                              *
 ****************************************************************/

#ifndef __NUVOX53DECODE_H__
#define __NUVOX53DECODE_H__
   
#include "Platform.h"
#include "System/SysInfra.h"

#ifdef  __cplusplus
extern "C"
{
#endif

// Version Definition
#define NUVOX53DECODE_MAJOR_NUM   3
#define NUVOX53DECODE_MINOR_NUM   10
#define NUVOX53DECODE_BUILD_NUM   002
#define NUVOX53DECODE_VERSION_NUM   _SYSINFRA_VERSION(NUVOX53DECODE_MAJOR_NUM, NUVOX53DECODE_MINOR_NUM, NUVOX53DECODE_BUILD_NUM)

// Constant Definition
#define NUVOX53DECODE_FRAME_COMPRS_SIZE    24   // UINT8 
#define NUVOX53DECODE_FRAME_SAMPLE_SIZE	  240   // INT16
#define NUVOX53DECODE_SRAM_STATIC_SIZES	  206   // INT16
#define NUVOX53DECODE_SRAM_SHARED_SIZES	  705   // INT16

/***************************************************************************/
/* Extern memory                                               */
/***************************************************************************/

extern INT16 g_i16NuVox53DecodeSRAM[];
#define NUVOX53DECODE_USEPF            (((UINT8 *) g_i16NuVox53DecodeSRAM) +   0) 

extern INT16 g_i16NuGlobalSRAM[];
#define NUVOX53DECODE_COMPRES_BUFFER   (((UINT8  *) g_i16NuGlobalSRAM) +   0)   //   0 ~  11   // UINT8 NUVOX53DECODE_COMPRES_BUFFER[NUVOX53DECODE_FRAME_COMPRS_SIZE];
#define NUVOX53DECODE_SAMPLES_BUFFER   (((INT16  *) g_i16NuGlobalSRAM) +  10)   //  10 ~ 249   // INT16 NUVOX53DECODE_SAMPLES_BUFFER[NUVOX53DECODE_FRAME_SAMPLE_SIZE];
#define NUVOX53DECODE_WORK_RATE        (((INT16  *) g_i16NuGlobalSRAM) + 635)   // 635 ~ 635   // INT16 NUVOX53DECODE_WORK_RATE[1];

/***************************************************************************/
/* Extern function                                               */
/***************************************************************************/

  void NuVox53Decode_DecodeInitial(void);
  void NuVox53Decode_DecodeProcess(void);
UINT32 NuVox53Decode_GetVersion(void);

#ifdef  __cplusplus
}
#endif

#endif	//#ifndef __NUVOX53DECODE_H__

