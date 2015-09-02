/********************************************************************
 *																	*
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.	*
 *																	*
 ********************************************************************/

#include "Driver/DrvCLK.h"
#include "Driver/DrvAPU.h"

//#define DRVAPU_RAMP_VALUE 64
#define DRVAPU_RAMP_VALUE 2

/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
#if defined (__N572F064__)|| defined(__N572F065__)
	INT16 g_i16DCBiasSW = 0x018f; // remove DC in DAC , Note: 0x18f should be read from configuration block
#else
	INT16 g_i16DCBiasSW = 0;
#endif

// for ramp up/down channel0
volatile E_RAMP_TYPE	g_eRampCh0 = eDRVAPU_NOT_RAMP;
INT16	g_i16ApuDataCh0 = 0;
INT16	g_i16ApuRampDataCh0 = 0;

// for eamp up/down channel1
volatile E_RAMP_TYPE	g_eRampCh1 = eDRVAPU_NOT_RAMP;
INT16	g_i16ApuDataCh1 = 0;
INT16	g_i16ApuRampDataCh1 = 0;


/*---------------------------------------------------------------------------------------------------------*/
/* Implement Functons                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
// Get Timer driver version
/*---------------------------------------------------------------------------------------------------------*/
UINT32
DrvAPU_GetVersion(void)
{
	return DRVAPU_VERSION_NUM;
}


/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvAPU_Open
// 
// Return:                                                                                                
//              ERRCODE                                                                       
// Description:                                                                                            
//				1.Check whether timer0 is idle or not
//				2.Open APU APB clock and reset IP
/*---------------------------------------------------------------------------------------------------------*/
ERRCODE
DrvAPU_Open(void)
{
#if defined (__N572F064__)|| defined(__N572F065__)
	UINT32 u32DacOfs;
	INT32 i32DacOfs;
#endif

	if (DrvCLK_IsEnabledApbDevice(DRVCLK_CLK_TMR0))
		return E_DRVAPU_TIMER0_ISOCCUPIED;
	

#if defined (__N572F064__)|| defined(__N572F065__)
 	u32DacOfs = (SYS.IMGMAP1 & 0x7FFC0) << 13;
	i32DacOfs = u32DacOfs;
	g_i16DCBiasSW = (i32DacOfs >> 19);
	DrvCLK_EnableApbDeviceClock(DRVCLK_CLK_APU);
	DrvSYS_ResetIP(DRVSYS_RST_APU);
	APU.DACCH1.u32Reg = (UINT32)(g_i16DCBiasSW );  // compensate DAC DC bias

#else
	DrvCLK_EnableAhbDeviceClock(DRVCLK_CLK_APU);
	DrvSYS_ResetIP(DRVSYS_RST_APU);
	g_i16DCBiasSW = 0;
	APU.DACCH1.u32Reg = 0;  						// compensate DAC DC bias
#endif	
	
	return E_SUCCESS;
}


/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvAPU_Close
// Parameter:  
//				void
// Return:                                                                                                
//              void                                                                       
// Description:                                                                                            
//				Close APU APB clock 
/*---------------------------------------------------------------------------------------------------------*/
void
DrvAPU_Close(void)
{
	DrvSYS_ResetIP(DRVSYS_RST_APU);
	DrvCLK_DisableApbDeviceClock(DRVCLK_CLK_APU);
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvAPU_EnablePA
// Parameter:  
//				void
// Return:                                                                                                
//              void                                                                       
// Description:                                                                                            
//				Enable power amplifier (SOP) 
/*---------------------------------------------------------------------------------------------------------*/
void
DrvAPU_EnablePA(void)
{
	// enable POP sound control
	// disable PA
//	APU.APUCR.u32Reg  = (APU.APUCR.u32Reg&(~ (APU_APUCR_PAMPE))|(APU_APUCR_POPCE)|(APU_APUCR_IVCE));
	
	//DrvTimer_WaitMicrosecondTmr0(2000);  // least 1mS

	// disable POP sound control
	// enable PA
	APU.APUCR.u32Reg  = (APU.APUCR.u32Reg/*&(~(APU_APUCR_POPCE))*/|(APU_APUCR_PAMPE));
	//DrvTimer_WaitMicrosecondTmr0(2000);
	
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvAPU_DisablePA
// Parameter:  
//				void
// Return:                                                                                                
//              void                                                                       
// Description:                                                                                            
//				Disable power amplifier (SOP) 
/*---------------------------------------------------------------------------------------------------------*/
void
DrvAPU_DisablePA(void)
{
	// disable PA
	// enable POP sound control
//	APU.APUCR.u32Reg  = (APU.APUCR.u32Reg&(~(APU_APUCR_PAMPE))|(APU_APUCR_POPCE));

//	DrvTimer_WaitMicrosecondTmr0(2000);  // least 1mS

//	APU.APUCR.u32Reg&=(~APU_APUCR_IVCE);
	APU.APUCR.u32Reg  = (APU.APUCR.u32Reg&(~(APU_APUCR_PAMPE)));
	//DrvTimer_WaitMicrosecondTmr0(2000);
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvAPU_WaitRampDownCh0                                                                              
//                                                                                                         
// Description:                                                                                            
//    	Wait channel0 to be ramped down to 0
//---------------------------------------------------------------------------------------------------------
void 
DrvAPU_WaitRampDownCh0(void)
{
	g_i16ApuRampDataCh0 = (APU.APUDR[7].u32Reg<<3);
	g_i16ApuRampDataCh0 >>= 3;
	g_i16ApuDataCh0 = g_i16ApuRampDataCh0;
	g_eRampCh0 = eDRVAPU_RAMP_DOWN;

	while(g_eRampCh0 == eDRVAPU_RAMP_DOWN) 
	{};
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvAPU_WaitRampUpCh0                                                                              
//                                                                                                         
// Description:                                                                                            
//    	Wait channel0 to be ramped up to i16RampUpValue
//---------------------------------------------------------------------------------------------------------
void 
DrvAPU_WaitRampUpAPUCh0(
	INT16 i16RampUpValue   // 16 bits
)  
{
	g_i16ApuRampDataCh0 = i16RampUpValue >> 3;
#if defined (__N572F064__)|| defined(__N572F065__)
	g_i16ApuDataCh0 = 0;
#else
	g_i16ApuDataCh0 = 0xf000;
#endif
	g_eRampCh0 = eDRVAPU_RAMP_UP;

	while(g_eRampCh0 == eDRVAPU_RAMP_UP) {};
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvAPU_GetRampDownValue                                                                              
//                                                                                                         
// Description:                                                                                            
//    	Get ramp down value
//---------------------------------------------------------------------------------------------------------
INT16
DrvAPU_GetRampDownValue(
	INT16 i16ApuData,
	INT16 i16ApuRampData
	)
{
#if defined (__N572F064__)|| defined(__N572F065__)
	if (i16ApuData != 0)
	{
		if (i16ApuRampData > 0)
		{
			i16ApuData -= DRVAPU_RAMP_VALUE;
			if (i16ApuData < 0)
			{
				i16ApuData = 0;
			}
		}
		else if (i16ApuRampData < 0)
		{
			i16ApuData += DRVAPU_RAMP_VALUE;
			if (i16ApuData > 0)
			{
				i16ApuData = 0;
			}
		}
	}
#else
	if (i16ApuData != 0xfffff000)
	{
		i16ApuData -= DRVAPU_RAMP_VALUE;
		if ((i16ApuData&0x1000)==0)
		   i16ApuData = 0xf000;
	}
#endif
	return i16ApuData;
}


//---------------------------------------------------------------------------------------------------------
// Function: DrvAPU_GetRampDownValueCh0                                                                              
//                                                                                                         
// Description:                                                                                            
//    	Get ramp down value of channel 0
//---------------------------------------------------------------------------------------------------------
static __inline
INT16
DrvAPU_GetRampDownValueCh0(void)
{
	return (g_i16ApuDataCh0 = DrvAPU_GetRampDownValue(g_i16ApuDataCh0,g_i16ApuRampDataCh0));
}


//---------------------------------------------------------------------------------------------------------
// Function: DrvAPU_GetRampUpValue                                                                              
//                                                                                                         
// Description:                                                                                            
//    	Get ramp up value
//---------------------------------------------------------------------------------------------------------
INT16
DrvAPU_GetRampUpValue(
	INT16 i16ApuData,
	INT16 i16ApuRampData
	)
{
	if (i16ApuData != i16ApuRampData)
	{
		if (i16ApuRampData < 0)
		{
			i16ApuData -= DRVAPU_RAMP_VALUE;
			if (i16ApuData < i16ApuRampData)
			{
				i16ApuData = i16ApuRampData;
			}
		}
		else if (i16ApuRampData >= 0)
		{
			i16ApuData += DRVAPU_RAMP_VALUE;
			if (i16ApuData > i16ApuRampData)
			{
				i16ApuData = i16ApuRampData;
			}
		}
	}
	return i16ApuData;
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvAPU_GetRampUpValueCh0                                                                              
//                                                                                                         
// Description:                                                                                            
//    	Get ramp up value of channel 0
//---------------------------------------------------------------------------------------------------------
static __inline
INT16
DrvAPU_GetRampUpValueCh0(void)
{
	return (g_i16ApuDataCh0 = DrvAPU_GetRampUpValue(g_i16ApuDataCh0,g_i16ApuRampDataCh0));
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvAPU_DoRampCh0                                                                              
//                                                                                                         
// Description:                                                                                            
//    	Do ramp up/down channel 0. Called in interrupt handler.
//---------------------------------------------------------------------------------------------------------
void
DrvAPU_DoRampCh0(void)
{
	INT16 	i16Temp;
	UINT8 i;
	if (g_eRampCh0 == eDRVAPU_RAMP_DOWN)
	{
		i16Temp = DrvAPU_GetRampDownValueCh0();
		for (i = 0 ; i < 8 ; i++)
			DrvAPU_SetApuData13bitCh0(i,i16Temp);
#if defined (__N572F064__)|| defined(__N572F065__)
		if (DrvAPU_GetApuDataCh0(0) == 0)
#else
		if (DrvAPU_GetApuDataCh0(0) == 0xfffff000)
#endif
			g_eRampCh0 = eDRVAPU_NOT_RAMP;
	}
	else if (g_eRampCh0 == eDRVAPU_RAMP_UP)
	{
		i16Temp = DrvAPU_GetRampUpValueCh0();
		for (i = 0 ; i < 8 ; i++)
			DrvAPU_SetApuData13bitCh0(i,i16Temp);
		if (g_i16ApuDataCh0 == g_i16ApuRampDataCh0)
			g_eRampCh0 = eDRVAPU_NOT_RAMP;
	}
}



//---------------------------------------------------------------------------------------------------------
// Function: DrvAPU_WaitRampDownCh1                                                                              
//                                                                                                         
// Description:                                                                                            
//    	Wait channel1 to be ramped down to DAC DC bias value.
//---------------------------------------------------------------------------------------------------------
void 
DrvAPU_WaitRampDownCh1(void)
{
	g_i16ApuRampDataCh1 = (APU.DACCH1.u32Reg<<3);
	g_i16ApuRampDataCh1 >>= 3;
	g_i16ApuRampDataCh1 -= g_i16DCBiasSW;
	g_i16ApuDataCh1 = g_i16ApuRampDataCh1;
	g_eRampCh1 = eDRVAPU_RAMP_DOWN;

	while(g_eRampCh1 == eDRVAPU_RAMP_DOWN) {};
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvAPU_WaitRampUpCh1                                                                              
//                                                                                                         
// Description:                                                                                            
//    	Wait channel1 to be ramped up to i16RampUpValue
//---------------------------------------------------------------------------------------------------------
void 
DrvAPU_WaitRampUpCh1(
	INT16 i16RampUpValue   // 16 bits
)  
{
	g_i16ApuRampDataCh1 = i16RampUpValue >> 3;
	g_i16ApuRampDataCh1 -= g_i16DCBiasSW;
	g_i16ApuDataCh1 = 0;
	g_eRampCh1 = eDRVAPU_RAMP_UP;

	while(g_eRampCh1 == eDRVAPU_RAMP_UP) 
	{};
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvAPU_GetRampDownValueCh1                                                                              
//                                                                                                         
// Description:                                                                                            
//    	Get ramp down value of channel 01
//---------------------------------------------------------------------------------------------------------
static __inline
INT16
DrvAPU_GetRampDownValueCh1(void)
{
	return (g_i16ApuDataCh1 = DrvAPU_GetRampDownValue(g_i16ApuDataCh1,g_i16ApuRampDataCh1));
}


//---------------------------------------------------------------------------------------------------------
// Function: DrvAPU_GetRampUpValueCh1                                                                              
//                                                                                                         
// Description:                                                                                            
//    	Get ramp up value of channel 1
//---------------------------------------------------------------------------------------------------------
static __inline
INT16
DrvAPU_GetRampUpValueCh1(void)
{
	return (g_i16ApuDataCh1 = DrvAPU_GetRampUpValue(g_i16ApuDataCh1,g_i16ApuRampDataCh1));
}


//---------------------------------------------------------------------------------------------------------
// Function: DrvAPU_DoRampCh1                                                                              
//                                                                                                         
// Description:                                                                                            
//    	Do ramp up/down channel 1. Called in interrupt handler.
//---------------------------------------------------------------------------------------------------------
void
DrvAPU_DoRampCh1(void)
{
	if (g_eRampCh1 == eDRVAPU_RAMP_DOWN)
	{
		DrvAPU_SetApuData13bitCh1(DrvAPU_GetRampDownValueCh1());
		if (g_i16ApuDataCh1 == 0)
			g_eRampCh1 = eDRVAPU_NOT_RAMP;
	}
	else if (g_eRampCh1 == eDRVAPU_RAMP_UP)
	{
		DrvAPU_SetApuData13bitCh1(DrvAPU_GetRampUpValueCh1());
		if (g_i16ApuDataCh1 == g_i16ApuRampDataCh1)
			g_eRampCh1 = eDRVAPU_NOT_RAMP;
	}
}



