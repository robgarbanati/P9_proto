/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.  *
 *                                                              *
 ****************************************************************/

#ifndef __NUONEDECODE_H__
#define __NUONEDECODE_H__
   
#include "Platform.h"
#include "System/SysInfra.h"

#ifdef  __cplusplus
extern "C"
{
#endif

// Version Definition
#define NUONEDECODE_MAJOR_NUM   3
#define NUONEDECODE_MINOR_NUM   10
#define NUONEDECODE_BUILD_NUM   001
#define NUONEDECODE_VERSION_NUM   _SYSINFRA_VERSION(NUONEDECODE_MAJOR_NUM, NUONEDECODE_MINOR_NUM, NUONEDECODE_BUILD_NUM)

// Constant Definition
#define NUONEDECODE_FRAME_SAMPLE_SIZE	320 
#define NUONEDECODE_CODE_SIZE		    120 
#define NUONEDECODE_SRAM_STATIC_SIZES	175 
#define NUONEDECODE_SRAM_SHARED_SIZES	640 

/***************************************************************************/
/* External memory                                                */
/***************************************************************************/

extern INT16 g_i16NuOneDecodeSRAM[];

#define NUONEDECODE_MODE             (INT16  *) (g_i16NuOneDecodeSRAM + 160)                  // extern INT16 NUONEDECODE_MODE;
#define NUONEDECODE_BITS_PER_FRAME   (INT16  *) (g_i16NuOneDecodeSRAM + 161)                  // extern INT16 NUONEDECODE_BITS_PER_FRAME;	
#define NUONEDECODE_COMPRESS_BUFFER  (INT16 **) (g_i16NuOneDecodeSRAM + 162)   //   0 ~  59   // extern INT16 *dp_out_words; // dp_out_words[C_BITS_PER_FRAME/16];

extern INT16 g_i16NuGlobalSRAM[];

#define NUONEDECODE_SAMPLES_BUFFER   (INT16  *) (g_i16NuGlobalSRAM +   0)   //   0 ~ 319   // extern INT16 *NUONEDECODE_SAMPLES_BUFFER;     // NUONEDECODE_SAMPLES_BUFFER[C_FRAMESIZE];

/***************************************************************************/
/* External function                                                */
/***************************************************************************/

void NuOneDecode_DecodeInitial(void);
void NuOneDecode_DecodeProcess(void);
UINT32 NuOneDecode_GetVersion(void);

#ifdef  __cplusplus
}
#endif

#endif	//#ifndef __NUONEDECODE_H__

