/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.  *
 *                                                              *
 ****************************************************************/

#ifndef __NULITEDECODE_H__
#define __NULITEDECODE_H__
   
#include "Platform.h"
#include "System/SysInfra.h"

#ifdef  __cplusplus
extern "C"
{
#endif

// Version Definition
#define NULITEDECODE_MAJOR_NUM   3
#define NULITEDECODE_MINOR_NUM   10
#define NULITEDECODE_BUILD_NUM   001
#define NULITEDECODE_VERSION_NUM   _SYSINFRA_VERSION(NULITEDECODE_MAJOR_NUM, NULITEDECODE_MINOR_NUM, NULITEDECODE_BUILD_NUM)

// Constant Definition
#define NULITEDECODE_FRAME_SAMPLE_SIZE	160 
#define NULITEDECODE_CODE_SIZE		     60 
#define NULITEDECODE_SRAM_STATIC_SIZES	 95 
#define NULITEDECODE_SRAM_SHARED_SIZES	329 

/***************************************************************************/
/* Extern memory                                               */
/***************************************************************************/

extern INT16 g_i16NuLiteDecodeSRAM[];

#define NULITEDECODE_MODE             (INT16  *) (g_i16NuLiteDecodeSRAM +  80)                  // extern INT16 NULITEDECODE_MODE;
#define NULITEDECODE_BITS_PER_FRAME   (INT16  *) (g_i16NuLiteDecodeSRAM +  81)                  // extern INT16 NULITEDECODE_BITS_PER_FRAME;
#define NULITEDECODE_COMPRESS_BUFFER  (INT16 **) (g_i16NuLiteDecodeSRAM +  82)   //   0 ~  29   // extern INT16 *dp_out_words; // dp_out_words[C_BITS_PER_FRAME/16];

extern INT16 g_i16NuGlobalSRAM[];

#define NULITEDECODE_SAMPLES_BUFFER   (INT16  *) (g_i16NuGlobalSRAM +   0)   //   0 ~ 159   // extern INT16 *NULITEDECODE_SAMPLES_BUFFER;     // NULITEDECODE_SAMPLES_BUFFER[C_FRAMESIZE];

/***************************************************************************/
/* Extern function                                               */
/***************************************************************************/

void NuLiteDecode_DecodeInitial(void);
void NuLiteDecode_DecodeProcess(void);
UINT32 NuLiteDecode_GetVersion(void);

#ifdef  __cplusplus
}
#endif

#endif	//#ifndef __NULITEDECODE_H__

