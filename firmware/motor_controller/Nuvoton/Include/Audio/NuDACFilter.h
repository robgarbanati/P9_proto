/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.  *
 *                                                              *
 ****************************************************************/

#ifndef __NUDACFILTER_H__
#define __NUDACFILTER_H__
   
#include "Platform.h"
#include "System/SysInfra.h"

#ifdef  __cplusplus
extern "C"
{
#endif

// Version Definition
#define NUDACFILTER_MAJOR_NUM   3
#define NUDACFILTER_MINOR_NUM   10
#define NUDACFILTER_BUILD_NUM   002
#define NUDACFILTER_VERSION_NUM   _SYSINFRA_VERSION(NUDACFILTER_MAJOR_NUM, NUDACFILTER_MINOR_NUM, NUDACFILTER_BUILD_NUM)

// Constant Definition
#define NUDAC2XFILTER_SRAM_STATIC_SIZES	 17 
#define NUDAC4XFILTER_SRAM_STATIC_SIZES	  9
	
#define NUDACFILTER_SRAM_STATIC_SIZES	 NUDAC2XFILTER_SRAM_STATIC_SIZES 

/***************************************************************************/
/* External memory                                               */
/***************************************************************************/

extern INT16 g_i16NuDACFilterSRAM[];

#define NUDAC2XFILTER_SAMPLES_BUFFER  (INT16  *) (g_i16NuDACFilterSRAM +  0)   // INT16 dp_dac2x_sample_buffer[16];
#define NUDAC2XFILTER_BUFFER_INDEX    (INT16  *) (g_i16NuDACFilterSRAM + 16)   // INT16 dp_dac2x_buffer_index;	
#define NUDAC4XFILTER_SAMPLES_BUFFER  (INT16  *) (g_i16NuDACFilterSRAM +  0)   // INT16 dp_dac4x_sample_buffer[ 8];
#define NUDAC4XFILTER_BUFFER_INDEX    (INT16  *) (g_i16NuDACFilterSRAM +  8)   // INT16 dp_dac4x_buffer_index;	

#define NUDACFILTER_SAMPLES_BUFFER  NUDAC2XFILTER_SAMPLES_BUFFER
#define NUDACFILTER_BUFFER_INDEX    NUDAC2XFILTER_BUFFER_INDEX	

/***************************************************************************/
/* External function                                               */
/***************************************************************************/

  void NuDACFilter_Up2LpfInitial(void);
 INT32 NuDACFilter_Up2LpfProcess(void);
  void NuDACFilter_Up4LpfInitial(void);
  void NuDACFilter_Up4LpfProcess(INT16 *);
UINT32 NuDACFilter_GetVersion(void);

#ifdef  __cplusplus
}
#endif

#endif	//#ifndef __NUDACFILTER_H__

