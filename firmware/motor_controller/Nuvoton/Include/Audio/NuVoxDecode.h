/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.  *
 *                                                              *
 ****************************************************************/

#ifndef __NUVOXDECODE_H__
#define __NUVOXDECODE_H__
   
#include "Platform.h"
#include "System/SysInfra.h"

#ifdef  __cplusplus
extern "C"
{
#endif

// Version Definition
#define NUVOXDECODE_MAJOR_NUM   3
#define NUVOXDECODE_MINOR_NUM   10
#define NUVOXDECODE_BUILD_NUM   001
#define NUVOXDECODE_VERSION_NUM   _SYSINFRA_VERSION(NUVOXDECODE_MAJOR_NUM, NUVOXDECODE_MINOR_NUM, NUVOXDECODE_BUILD_NUM)

// Constant Definition
#define NUVOXDECODE_FRAME_SAMPLE_SIZE	180 
#define NUVOXDECODE_CODE_SIZE		      7 
#define NUVOXDECODE_SRAM_STATIC_SIZES	414 
#define NUVOXDECODE_SRAM_SHARED_SIZES  1008 

/**************************************************************************/
/* Constant definitions                                                   */
/**************************************************************************/

#define NUVOXDECODE_FRAME          180
#define NUVOXDECODE_DISP_ORD        64   /* pulse dispersion filter order */

/***************************************************************************/
/* External memory                                               */
/***************************************************************************/

extern INT16 g_i16NuVoxDecodeSRAM[];

#define NUVOXDECODE_MOD8_FRAME ( INT16   *) (g_i16NuVoxDecodeSRAM +   0)   //   0   // INT16 NUVOXDECODE_MOD8_FRAME;             

extern INT16 g_i16NuGlobalSRAM[];

#define NUVOXDECODE_SP_OUT     ( INT16  *) (g_i16NuGlobalSRAM +   9)   //   9 ~ 252  // extern  INT16 NUVOXDECODE_SP_OUT[];
#define NUVOXDECODE_DATA_IN    ( INT16  *) (g_i16NuGlobalSRAM +   9)   //   9 ~  12  // static  INT16 NUVOXDECODE_DATA_IN[4]; 

/***************************************************************************/
/* External function                                               */
/***************************************************************************/

void NuVoxDecode_DecodeInitial(void);
void NuVoxDecode_DecodeProcess(void);
UINT32 NuVoxDecode_GetVersion(void);

#ifdef  __cplusplus
}
#endif

#endif	//#ifndef __NUVOXDECODE_H__

