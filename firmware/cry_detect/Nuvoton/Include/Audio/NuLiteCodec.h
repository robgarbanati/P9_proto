/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.  *
 *                                                              *
 ****************************************************************/

#ifndef __NULITECODEC_H__
#define __NULITECODEC_H__
   
#include "Platform.h"
#include "System/SysInfra.h"

#ifdef  __cplusplus
extern "C"
{
#endif

// Version Definition
#define NULITECODEC_MAJOR_NUM   3
#define NULITECODEC_MINOR_NUM   10
#define NULITECODEC_BUILD_NUM   001
#define NULITECODEC_VERSION_NUM   _SYSINFRA_VERSION(NULITECODEC_MAJOR_NUM, NULITECODEC_MINOR_NUM, NULITECODEC_BUILD_NUM)

// Constant Definition
#define NULITECODEC_FRAME_SAMPLE_SIZE	160 
#define NULITECODEC_CODE_SIZE		     60 
#define NULITECODEC_SRAM_STATIC_SIZES	165 
#define NULITECODEC_SRAM_SHARED_SIZES	337 

/***************************************************************************/
/* External memory                                               */
/***************************************************************************/

extern INT16 g_i16NuLiteCodecSRAM[];

#define NULITECODEC_MODE             (INT16  *) (g_i16NuLiteCodecSRAM + 160)                  // extern INT16 NULITECODEC_MODE;
#define NULITECODEC_BITS_PER_FRAME   (INT16  *) (g_i16NuLiteCodecSRAM + 161)                  // extern INT16 NULITECODEC_BITS_PER_FRAME;
#define NULITECODEC_COMPRESS_BUFFER  (INT16 **) (g_i16NuLiteCodecSRAM + 162)   // 130 ~ 159   // extern INT16 *dp_out_words; // dp_out_words[C_BITS_PER_FRAME/16];

extern INT16 g_i16NuGlobalSRAM[];

#define NULITECODEC_SAMPLES_BUFFER   (INT16  *) (g_i16NuGlobalSRAM +   0)   //   0 ~ 159   // extern INT16 *NULITECODEC_SAMPLES_BUFFER;     // NULITECODEC_SAMPLES_BUFFER[C_FRAMESIZE];

/***************************************************************************/
/* External function                                               */
/***************************************************************************/

void NuLiteCodec_EncodeInitial(void);
void NuLiteCodec_EncodeProcess(void);

void NuLiteCodec_DecodeInitial(void);
void NuLiteCodec_DecodeProcess(void);

UINT32 NuLiteCodec_GetVersion(void);

#ifdef  __cplusplus
}
#endif

#endif	//#ifndef __NULITECODEC_H__

