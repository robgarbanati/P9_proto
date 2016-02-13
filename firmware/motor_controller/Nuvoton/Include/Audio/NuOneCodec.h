/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.  *
 *                                                              *
 ****************************************************************/

#ifndef __NUONECODEC_H__
#define __NUONECODEC_H__
   
#include "Platform.h"
#include "System/SysInfra.h"

#ifdef  __cplusplus
extern "C"
{
#endif

// Version Definition
#define NUONECODEC_MAJOR_NUM   3
#define NUONECODEC_MINOR_NUM   10
#define NUONECODEC_BUILD_NUM   001
#define NUONECODEC_VERSION_NUM   _SYSINFRA_VERSION(NUONECODEC_MAJOR_NUM, NUONECODEC_MINOR_NUM, NUONECODEC_BUILD_NUM)

// Constant Definition
#define NUONECODEC_FRAME_SAMPLE_SIZE	320 
#define NUONECODEC_CODE_SIZE		    120 
#define NUONECODEC_SRAM_STATIC_SIZES	325 
#define NUONECODEC_SRAM_SHARED_SIZES	640 

/***************************************************************************/
/* External memory                                               */
/***************************************************************************/

extern INT16 g_i16NuOneCodecSRAM[];

#define NUONECODEC_MODE             (INT16  *) (g_i16NuOneCodecSRAM + 320)                  // extern INT16 NUONECODEC_MODE;
#define NUONECODEC_BITS_PER_FRAME   (INT16  *) (g_i16NuOneCodecSRAM + 321)                  // extern INT16 NUONECODEC_BITS_PER_FRAME;	
#define NUONECODEC_COMPRESS_BUFFER  (INT16 **) (g_i16NuOneCodecSRAM + 322)   // 260 ~ 319   // extern INT16 *dp_out_words; // dp_out_words[C_BITS_PER_FRAME/16];

extern INT16 g_i16NuGlobalSRAM[];

#define NUONECODEC_SAMPLES_BUFFER   (INT16  *) (g_i16NuGlobalSRAM +   0)   //   0 ~ 319   // extern INT16 *NUONECODEC_SAMPLES_BUFFER;     // NUONECODEC_SAMPLES_BUFFER[C_FRAMESIZE];

/***************************************************************************/
/* External function                                               */
/***************************************************************************/

void NuOneCodec_EncodeInitial(void);
void NuOneCodec_EncodeProcess(void);

void NuOneCodec_DecodeInitial(void);
void NuOneCodec_DecodeProcess(void);

UINT32 NuOneCodec_GetVersion(void);


// only for compatibility
#define NUONE_FRAME_SAMPLE_SIZE	320
#define NUONE_CODE_SIZE		40

typedef UINT32 (*PFN_NUONECALLBACK_USREVENT)(UINT16 u16Index, UINT16 u16SubIndex);
typedef UINT32 (*PFN_NUONECALLBACK_USRDATAREQUEST)(PINT8 pi8DataPtr, INT8 i8Channel, INT16 i16DataCount);

extern PINT32 g_pi32DataLeft;

extern void NuOneCodec_DecodeInitial(void);
extern void NuOneCodec_EncodeInitial(void);

#define NuOneCodec_Decode_Init(pi8Static,pi8Share,i8Channel)\
	NuOneCodec_DecodeInitial();

#define NuOneCodec_Encode_Init(pi8Static,pi8Share)\
	NuOneCodec_EncodeInitial();

void NuOneCodec_Encode_Set_BPS(INT16 i16BpsID);
	
void NuOneCodec_Decode_Set_BPS(INT16 i16BpsID,INT8 i8Channel);

void NuOneCodec_Initialize(void	);

void NuOneCodec_RegisterCallback(PFN_NUONECALLBACK_USREVENT pfnUserEventCallback,PFN_NUONECALLBACK_USRDATAREQUEST pfnUserDataReguestCallBack);

UINT32 NuOneCodec_GetVersion(void);

void NuOneCodec_Encode_Main(PINT16 pi16Samples,PINT8 pi8FrameBuffer,INT8 i8Channel,INT32 i32Count);
	
void NuOneCodec_Decode_Main(PINT16 pi16Words,PINT8 pi8OutBuffer,INT8 i8Channel,INT32 i32Count);

INT16 NuOneCodec_Event_Decode_Main(PINT8 pi8OutBuffer,INT8 i8Channel,INT32 i32Count,PINT16 pi16SampleRate);

INT16 NuOneCodec_Channel_End(INT8 i8Channel);
// only for compatibility

#ifdef  __cplusplus
}
#endif

#endif	//#ifndef __NUONECODEC_H__

