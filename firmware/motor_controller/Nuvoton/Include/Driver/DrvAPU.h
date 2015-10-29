/********************************************************************
 *																	*
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.	*
 *																	*
 ********************************************************************/

#ifndef __DRVAPU_H__
#define __DRVAPU_H__

// --------------------------------------------------------------
// Include Header Files
// --------------------------------------------------------------
#include "Platform.h"						 
#include "System/SysInfra.h"
#include "Driver/DrvTimer.h"


#ifdef  __cplusplus
extern "C"
{
#endif


// --------------------------------------------------------------
// Version Definition and Error Code
// --------------------------------------------------------------
#define	DRVAPU_MAJOR_NUM 1
#define	DRVAPU_MINOR_NUM 20
#define	DRVAPU_BUILD_NUM 001
#define DRVAPU_VERSION_NUM				_SYSINFRA_VERSION(DRVAPU_MAJOR_NUM, DRVAPU_MINOR_NUM, DRVAPU_BUILD_NUM)

// Error message
// E_DRVAPU_TIMER0_ISOCCUPIED				Timer0 is not idle
#define	E_DRVAPU_TIMER0_ISOCCUPIED		_SYSINFRA_ERRCODE(TRUE, MODULE_ID_DRVAPU, 1)

#define DrvAPU_WaitRampUpCh0(i16RampUpValue)			DrvAPU_WaitRampUpAPUCh0(i16RampUpValue)
//---------------------------------------------------------------
// APU output bypass power amplifier 
//---------------------------------------------------------------
typedef enum{
	eDRVAPU_NONBYPASS_PA = 0,
	eDRVAPU_BYPASS_PA
} E_DRVAPU_PA;

//---------------------------------------------------------------
// APU Volume
//---------------------------------------------------------------
typedef enum{
  	eDRVAPU_VOLUME_0DB = 0,	    //MAXIMUM
	eDRVAPU_VOLUME_NEG_3DB, 
	eDRVAPU_VOLUME_NEG_6DB, 
	eDRVAPU_VOLUME_NEG_9DB, 
	eDRVAPU_VOLUME_NEG_12DB,	
	eDRVAPU_VOLUME_NEG_15DB,	
	eDRVAPU_VOLUME_NEG_18DB		//MINMUM
} E_DRVAPU_VOLUME;

//---------------------------------------------------------------
// APU interrupt threshold
//---------------------------------------------------------------
 typedef enum{
  	eDRVAPU_BUF0_TH = 0,
	eDRVAPU_BUF1_TH,
	eDRVAPU_BUF2_TH,
	eDRVAPU_BUF3_TH,
	eDRVAPU_BUF4_TH,
	eDRVAPU_BUF5_TH,
	eDRVAPU_BUF6_TH,
	eDRVAPU_BUF7_TH,
 }E_DRVAPU_INT_THRESHOLD;

//---------------------------------------------------------------
// APU ramp status
//---------------------------------------------------------------
typedef enum {
	eDRVAPU_NOT_RAMP = 0,
	eDRVAPU_RAMP_DOWN = 1,
	eDRVAPU_RAMP_UP	= 2
} E_RAMP_TYPE;


//---------------------------------------------------------------
extern INT16 g_i16DCBiasSW;
extern INT16 g_i16AveCalibrationValue;
extern volatile E_RAMP_TYPE	g_eRampCh0;
extern volatile E_RAMP_TYPE	g_eRampCh1;

//---------------------------------------------------------------
// Function Prototypes
//---------------------------------------------------------------

/*---------------------------------------------------------------------------------------------------------*/
// Get Timer driver version
/*---------------------------------------------------------------------------------------------------------*/
UINT32
DrvAPU_GetVersion(void);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvAPU_Open
//
// Return :  
//				ERRCODE  		
//
// Description:                                                                                            
//				1.Check whether timer0 is idle or not
//				2.Open APU APB clock and reset IP
/*---------------------------------------------------------------------------------------------------------*/
ERRCODE
DrvAPU_Open(void);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvAPU_Close
//
// Description:                                                                                            
//				Close APU APB clock. 
/*---------------------------------------------------------------------------------------------------------*/
void
DrvAPU_Close(void);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvAPU_EnablePA
//
// Description:                                                                                            
//				Enable power amplifier. 
/*---------------------------------------------------------------------------------------------------------*/
void
DrvAPU_EnablePA(void);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvAPU_EnablePA
//
// Description:                                                                                            
//				Enable power amplifier. 
/*---------------------------------------------------------------------------------------------------------*/
void
DrvAPU_DisablePA(void);

// inline function declaration

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvAPU_EnableDAC
//
// 
// Description:                                                                                            
//				Enable DAC.
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvAPU_EnableDAC(void)
{
	APU.APUCR.DACE = 1;
	APU.APUCR.DACGN = 1;
	//DrvAPU_RampUp();
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvAPU_DisableDAC
//
// 
// Description:                                                                                            
//				Disable DAC.
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void 
DrvAPU_DisableDAC(void)
{
	APU.APUCR.DACE = 0;
	APU.APUCR.DACGN = 0;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvAPU_BypassPA
//
// Parameter:
//				eAPUOutput			[in] bypass PA or not:
//										 Please refer to E_DRVAPU_PA			
// Description:                                                                                            
//				APU output pin whether bypass power amplifier or not.
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvAPU_BypassPA(
	E_DRVAPU_PA	eAPUOutput
)
{
	APU.APUCR.BPPAM = eAPUOutput;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvAPU_SetVolume
//
// Parameter:  
//				u8APUVolume			[in] APU volume (0~7)
//
// Description:                                                                                            
//				Set APU volume.
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvAPU_SetVolume(
	E_DRVAPU_VOLUME eAPUVolume
)
{
	APU.APUVM.VOLUM = eAPUVolume;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvAPU_GetVolume
//
// Return:                                                                                                
//              APU volume
//
// Description:                                                                                            
//				Get APU volume.
/*---------------------------------------------------------------------------------------------------------*/
static __inline
E_DRVAPU_VOLUME
DrvAPU_GetVolume(void)
{
	return (E_DRVAPU_VOLUME)APU.APUVM.VOLUM;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvAPU_StartCh0
//
// Parameter:  
// 				u32TimerOption		[in] 	2 timer0 options -- enable interrupt or not, prescale value.
//											ex: (DRVTIMER_ENABLE_INTERRUPT|150)
//
//											DRVTIMER_ENABLE_INTERRUPT  
//												: Enable timer interrupt if it's set. Otherwise disable.
//											0~255 (prescale value)
//
//				u16CompareValue		[in]	Timer0 compare value (0~65536, but 0x0 and 0x1 are not allowed)
// Note:
// 		for saving the code, we won't support sample rate calculation in driver.
// 		"prescale" and "compare value" of timer0 should be passed by upper layer.
// 		User can look into the table we provide in reference guide or calculate
// 		them by self.
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvAPU_StartCh0(
	UINT8  u8PreScale,
	UINT16 u16CompareValue
)
{
	DrvTimer_OpenTmr0(DRVTIMER_START_COUNTING|DRVTIMER_PERIODIC_MODE|u8PreScale,u16CompareValue);
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvAPU_StopCh0
//
// Description:                                                                                            
//				Stop channel 0. (close timer0)
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvAPU_StopCh0(void)
{
	DrvTimer_CloseTmr0();
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvAPU_PauseCh0
//
// Description:                                                                                            
//				Pause channel 0. (disable timer0)
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvAPU_PauseCh0(void)
{
	DrvTimer_DisableTmr0();
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvAPU_ResumeCh0
//
// Description:                                                                                            
//				Resume channel 0. (enable timer0)
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvAPU_ResumeCh0(void)
{
	DrvTimer_EnableTmr0();
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvAPU_EnableThresholdInt
//
// Description:                                                                                            
//				Enable channel0 threshold interrupt.
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvAPU_EnableThresholdInt(void)
{
	APU.APUCR.APUIE = 1;
	NVIC_EnableIRQ(APU_IRQn); 
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvAPU_DisableThresholdInt
//
// Description:                                                                                            
//				Disable channel0 threshold interrupt.
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvAPU_DisableThresholdInt(void)
{
	APU.APUCR.APUIE = 0;
	NVIC_DisableIRQ(APU_IRQn); 
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvAPU_ClearThresholdIntFlag
//
// Description:                                                                                            
//				Clear threshold interrupt flag
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvAPU_ClearThresholdIntFlag(void)
{
	APU.APUCR.APUIS = 0;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvAPU_GetThresholdIntFlag
//
// Description:                                                                                            
//				Get threshold interrupt flag
/*---------------------------------------------------------------------------------------------------------*/
static __inline
UINT8 
DrvAPU_GetThresholdIntFlag(void)
{
	return (UINT8)APU.APUCR.APUIS;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvAPU_GetThresholdValue
//
// Description:                                                                                            
//				Get threshold Setting Value
/*---------------------------------------------------------------------------------------------------------*/
static __inline
UINT8 
DrvAPU_GetThresholdValue(void)
{
	return (UINT8)APU.APUCR.TSHD;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvAPU_SetApuData13bitCh0
//
// Parameter:  
//				u8Idx				[in] index number of PCM buffer (0~7)
//				i16Pcm				[in] PCM data (2' complement)
//
// Description:                                                                                            
//				Set 13 bits audio data to APU channel0 data
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvAPU_SetApuData13bitCh0(
	UINT8 u8Idx,
	INT16 i16Pcm
)
{
	APU.APUDR[u8Idx].u32Reg = (UINT32)(i16Pcm);
}
/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvAPU_SetApuDataCh0
//
// Parameter:  
//				u8Idx				[in] index number of PCM buffer (0~7)
//				i16Pcm				[in] PCM data (2' complement)
//
// Description:                                                                                            
//				Set APU channel0 data
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvAPU_SetApuDataCh0(
	UINT8 u8Idx,
	INT16 i16Pcm
)
{
	DrvAPU_SetApuData13bitCh0(u8Idx, i16Pcm >> 3);
}


/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvAPU_GetApuDataCh0
//
// Parameter:  
//				u8Idx				[in] index number of PCM buffer (0~7)
// return:
//									[in] PCM data (2' complement) of buffer[u8Idx]
//
// Description:                                                                                            
//				Get APU channel0 data
/*---------------------------------------------------------------------------------------------------------*/
static __inline
INT16
DrvAPU_GetApuDataCh0(
	UINT8 u8Idx
)
{
#if defined (__N572F064__)|| defined (__N572F065__)	
	return (((INT16)(APU.APUDR[u8Idx].u32Reg<<3))>>3);
#else
	return (INT16)(APU.APUDR[u8Idx].u32Reg);
#endif
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvAPU_SetApuData13bitCh1
//
// Parameter:  
//				i16Pcm				[in] PCM data (2' complement)
//
// Description:                                                                                            
//				Set  13 bits audio data to APU channel1, will calibrate DC for saving power
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvAPU_SetApuData13bitCh1(
	INT16 i16Pcm
)
{
	i16Pcm += g_i16DCBiasSW;	// compensate DC bias
	if (i16Pcm > 4095)
		i16Pcm = 4095;
	if (i16Pcm < -4096)
		i16Pcm = -4096;
	APU.DACCH1.u32Reg = (UINT32)(i16Pcm );
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvAPU_SetApuDataCh1
//
// Parameter:  
//				i16Pcm				[in] PCM data (2' complement)
//
// Description:                                                                                            
//				Set 16 bits audio data to APU channel1 
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvAPU_SetApuDataCh1(
	INT16 i16Pcm
)
{
	DrvAPU_SetApuData13bitCh1(i16Pcm >> 3);
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvAPU_GetApuDataCh1
//
// return:
//									[in] PCM data (2' complement) of buffer
//
// Description:                                                                                            
//				Get APU channel1 data
/*---------------------------------------------------------------------------------------------------------*/
static __inline
INT16
DrvAPU_GetApuDataCh1()
{
#if defined (__N572F064__)|| defined (__N572F065__)
	return ((INT16)(APU.DACCH1.u32Reg<<3)>>3);
#else 
	return (INT16)(APU.DACCH1.u32Reg);
#endif
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvAPU_ResetApuDataCh0                                                                              
//                                                                                                         
// Description:                                                                                            
//    	Clear CH0 data to avoid pop-sound.
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvAPU_ResetApuDataCh0(void)
{
	APU.APUDR[0].u32Reg = 0; 
	APU.APUDR[1].u32Reg = 0;
	APU.APUDR[2].u32Reg = 0;
	APU.APUDR[3].u32Reg = 0;
	APU.APUDR[4].u32Reg = 0;
	APU.APUDR[5].u32Reg = 0;
	APU.APUDR[6].u32Reg = 0;
	APU.APUDR[7].u32Reg = 0;
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvAPU_ResetApuDataCh1                                                                              
//                                                                                                         
// Description:                                                                                            
//    	Reset CH1 data to DC bias value to compensate the DC bias.
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvAPU_ResetApuDataCh1(void)
{
	// compensate DAC DC bias. 
	APU.DACCH1.u32Reg = (UINT32)(g_i16DCBiasSW );  // compensate DAC DC bias
}
	
/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvAPU_SetIntThreshold
//
// Parameter:  
//				eBufInt				[in] Buffer threshold 
//
// Description:                                                                                            
//				Set APU Interrupt of buffer threshold
/*---------------------------------------------------------------------------------------------------------*/
static __inline					  
void
DrvAPU_SetIntThreshold(
	E_DRVAPU_INT_THRESHOLD	eBufInt	
)
{
	APU.APUCR.TSHD = eBufInt;
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvADC_CalibrateDacDcWithAdcDc                                                                              
//                                                                                                         
// Description:                                                                                            
//    	Calibrate DAC DC offset with ADC DC value. The fuction should be called as if apPlication would like
//		to output ADC value to DAC.
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvAPU_CalibrateDacDcWithAdcDc(void)
{
	// compensate ADC & DAC DC bias. 
	g_i16DCBiasSW -=  (g_i16AveCalibrationValue << 1);	// ADC 12 bits -> DAC 13 bits
	APU.DACCH1.u32Reg = (UINT32)(g_i16DCBiasSW );  // compensate DAC DC bias
}

//---------------------------------------------------------------------------------------------------------
// Function: IsRampCh0                                                                              
//                                                                                                         
// Description:                                                                                            
//    	Check ramp status of channel0
//---------------------------------------------------------------------------------------------------------
static __inline
BOOL
DrvAPU_IsRampCh0()
{
	return (g_eRampCh0 != eDRVAPU_NOT_RAMP);
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvAPU_DoRampCh0                                                                              
//                                                                                                         
// Description:                                                                                            
//    	Do ramp up/down channel 0. Called in interrupt handler.
//---------------------------------------------------------------------------------------------------------
void
DrvAPU_DoRampCh0(void);


//---------------------------------------------------------------------------------------------------------
// Function: DrvAPU_WaitRampDownCh0                                                                              
//                                                                                                         
// Description:                                                                                            
//    	Wait channel0 to be ramped down to 0
//---------------------------------------------------------------------------------------------------------
void 
DrvAPU_WaitRampDownCh0(void);

//---------------------------------------------------------------------------------------------------------
// Function: DrvAPU_WaitRampUpAPUCh0                                                                              
//                                                                                                         
// Description:                                                                                            
//    	Wait channel0 to be ramped up to i16RampUpValue
//---------------------------------------------------------------------------------------------------------
void 
DrvAPU_WaitRampUpAPUCh0(
	INT16 i16RampUpValue   // 16 bits
);

//---------------------------------------------------------------------------------------------------------
// Function: IsRampCh1                                                                              
//                                                                                                         
// Description:                                                                                            
//    	Check ramp status of channel1
//---------------------------------------------------------------------------------------------------------
static __inline
BOOL
DrvAPU_IsRampCh1()
{
	return (g_eRampCh1 != eDRVAPU_NOT_RAMP);
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvAPU_DoRampCh1                                                                              
//                                                                                                         
// Description:                                                                                            
//    	Do ramp up/down channel 1. Called in interrupt handler.
//---------------------------------------------------------------------------------------------------------
void
DrvAPU_DoRampCh1(void);

//---------------------------------------------------------------------------------------------------------
// Function: DrvAPU_WaitRampDownCh1                                                                              
//                                                                                                         
// Description:                                                                                            
//    	Wait channel1 to be ramped down to DAC DC bias value.
//---------------------------------------------------------------------------------------------------------
void 
DrvAPU_WaitRampDownCh1(void);

//---------------------------------------------------------------------------------------------------------
// Function: DrvAPU_WaitRampUpCh1                                                                              
//                                                                                                         
// Description:                                                                                            
//    	Wait channel1 to be ramped up to i16RampUpValue
//---------------------------------------------------------------------------------------------------------
void 
DrvAPU_WaitRampUpCh1(
	INT16 i16RampUpValue   // 16 bits
);  



#ifdef  __cplusplus
}
#endif

#endif	// __DRVAPU_H__


