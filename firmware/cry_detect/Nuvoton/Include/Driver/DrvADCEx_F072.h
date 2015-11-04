/********************************************************************
 *																	*
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.	*
 *																	*
 ********************************************************************/

#ifndef __DRVADCEX_H__
#define __DRVADCEX_H__

// --------------------------------------------------------------
// Include Header Files
// --------------------------------------------------------------
#include "Platform.h"

//---------------------------------------------------------------
// ADC pre-amp gain2 definition
//---------------------------------------------------------------
#define	DRVADC_PAG2_0DB 		0
#define DRVADC_PAG2_10DB 		1
#define DRVADC_PAG2_20DB 		2
#define DRVADC_PAG2_30DB 		3
//---------------------------------------------------------------
// ADC pre-amp gain1 definition
//---------------------------------------------------------------
#define DRVADC_PAG1_20DB         0
#define DRVADC_PAG1_21DB         1
#define DRVADC_PAG1_22DB         2
#define DRVADC_PAG1_23DB         3
#define DRVADC_PAG1_24DB         4
#define DRVADC_PAG1_25DB         5
#define DRVADC_PAG1_26DB         6
#define DRVADC_PAG1_27DB         7
#define DRVADC_PAG1_28DB         8
#define DRVADC_PAG1_29DB         9
#define DRVADC_PAG1_30DB         10
#define DRVADC_PAG1_31DB         11
#define DRVADC_PAG1_32DB         12
#define DRVADC_PAG1_33DB         13
#define DRVADC_PAG1_34DB         14
#define DRVADC_PAG1_35DB         15
#define DRVADC_PAG1_36DB         16
#define DRVADC_PAG1_37DB         17
#define DRVADC_PAG1_38DB         18
#define DRVADC_PAG1_39DB         19
#define DRVADC_PAG1_40DB         20
//---------------------------------------------------------------
// AVDD/2 accelerating
//---------------------------------------------------------------
typedef enum {
	eDRVADC_CTRS_R10K		= 0,
	eDRVADC_CTRS_R40K		= 1,
	eDRVADC_CTRS_R200K		= 2,
	eDRVADC_CTRS_R600K		= 3
} E_DRVADC_ACCELERATING_CTRS;
//---------------------------------------------------------------
// Fast wake up for RC
//---------------------------------------------------------------
typedef enum {
	eDRVADC_FWU_R8K			= 0,
	eDRVADC_FWU_R40K		= 1,
	eDRVADC_FWU_R200K		= 2,
	eDRVADC_FWU_R400K		= 3
} E_DRVADC_WAKEUP_FWU;


//---------------------------------------------------------------------------------------------------------
// Function: DrvADC_EnableRegulator                                                                                                                                                         
//                                                                                                         
// Description:                                                                                            
//    	This API will enable regulator for gain noiseless performance.
//		Regulator is enabled: PGC power source is from regulator-> 0.85* AVDD.
//                            microphone bias is from regulator-> 0.85*AVDD.                                                 
//---------------------------------------------------------------------------------------------------------
static __inline
void 
DrvADC_EnableRegulator(void)
{
	ADC.PGCR.u32Reg = (ADC.PGCR.u32Reg|ADC_PGCR_MICE|ADC_PGCR_APPS);
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvADC_DisableRegulator                                                                                                                                                         
//                                                                                                         
// Description:                                                                                            
//    	Regulator is disabled: PGC power source is from AVDD.
//      					   microphone bias is from AVDD.                                                 
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvADC_DisableRegulator(void)
{
	ADC.PGCR.u32Reg = (ADC.PGCR.u32Reg|(~(ADC_PGCR_MICE|ADC_PGCR_APPS)));
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvADC_SetRegulatorRC                                                                                                                                                         
//                                                                                                         
// Parameters:
//		eACC_R		[in]: Resistance value for AVDD accelerating.
//		eWAKE_R		[in]: RC value for regulator fast wake up.
//
// Description:                                                                                            
//    	This API set RC vlaue that will accelerate regulator wake up. User must call it before mic-bias
//      calculating and ADC setting if regulator is enabled.
//      					   
// Note:
//		"eACC_R = eDRVADC_CTRS_R10K" and "eWAKE_R = eDRVADC_FWU_R8K" is fast path for regulator. After 
//		fast path, user must reset "eACC_R = eDRVADC_CTRS_R600K" and "eWAKE_R = eDRVADC_FWU_R400K" for 
//		regulator normal path, then ADC can get valid value.
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvADC_SetRegulatorRC(
	E_DRVADC_ACCELERATING_CTRS	eACC_R,
	E_DRVADC_WAKEUP_FWU			eWAKE_R
)
{
    ADC.MIBSCR.CTRS = eACC_R;
	ADC.MIBSCR.FWU	= eWAKE_R;
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvADC_GetRegulatorRC                                                                                                                                                         
//                                                                                                         
// Parameters:
// 		pu8ACC_R 	[out] :	save resistance value of AVDD accelerating.
//		pu8WAKE_R	[out] :	save RC value of regualtor for fast wake up.
// 
// Description:                                                                                            
//    	Get current RC value of AVDD accelerating and regulator.                                                 
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvADC_GetRegulatorRC(
	PUINT8	pu8ACC_R,
	PUINT8	pu8WAKE_R
)
{
   *pu8ACC_R = ADC.MIBSCR.CTRS;
   *pu8WAKE_R =	ADC.MIBSCR.FWU; 
}

#endif

