/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.  *
 *                                                              *
 ****************************************************************/

#ifndef __NUADCFILTER_H__
#define __NUADCFILTER_H__
   
#include "Platform.h"
#include "System/SysInfra.h"

#ifdef  __cplusplus
extern "C"
{
#endif

// Version Definition
#define NUADCFILTER_MAJOR_NUM   3
#define NUADCFILTER_MINOR_NUM   10
#define NUADCFILTER_BUILD_NUM   002
#define NUADCFILTER_VERSION_NUM   _SYSINFRA_VERSION(NUADCFILTER_MAJOR_NUM, NUADCFILTER_MINOR_NUM, NUADCFILTER_BUILD_NUM)

// Constant Definition
#define NUADCFILTER_SRAM_STATIC_SIZES	 33 

/***************************************************************************/
/*External memory                                               */
/***************************************************************************/

extern INT16 g_i16NuADCFilterSRAM[];

#define NUADCFILTER_SAMPLES_BUFFER  (INT16  *) (g_i16NuADCFilterSRAM +  0)   // INT16 dp_adc_sample_buffer[32];
#define NUADCFILTER_BUFFER_INDEX    (INT16  *) (g_i16NuADCFilterSRAM + 32)   // INT16 dp_adc_buffer_index;	

/***************************************************************************/
/*External function                                               */
/***************************************************************************/

  void NuADCFilter_LpfDown2Initial(void);
 INT16 NuADCFilter_LpfDown2Process(void);
  void NuADCFilter_LpfDown4Initial(void);
 INT16 NuADCFilter_LpfDown4Process(void);
UINT32 NuADCFilter_GetVersion(void);

#ifdef  __cplusplus
}
#endif

#endif	//#ifndef __NUADCFILTER_H__

