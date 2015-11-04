//---------------------------------------------------------------------------------------------------------
//                                                                                                         
// Copyright(c) Nuvoton Technology Corp. All rights reserved.                                         
//                                                                                                         
//---------------------------------------------------------------------------------------------------------


//---------------------------------------------------------------------------------------------------------
// Include related headers                                                                                 
//---------------------------------------------------------------------------------------------------------
#include "Driver/DrvADC.h"
#include "Driver/DrvCLK.h"
#include "Driver/DrvSYS.h"

//---------------------------------------------------------------------------------------------------------
// Global file scope (static) variables                                                                    
//---------------------------------------------------------------------------------------------------------
INT16 g_i16AveCalibrationValue;

//--------------------------------------------------------------------------------------------------------------------
// Function: DrvADC_Open                                                                                              
//                                                                                                                    
// Parameters:        	                                                                                              
//   	eOpMode				[in]	Specify the operation mode.                                                           
//   	u8DataFormat		[in]	data format of converion result
//										DRVADC_2COMPLIMENT,    
//										DRVADC_UNSIGNED                                                                                                              
// Returns:                                                                                                           
//   	void                                                                                                            
//                                                                                                                    
// Description:                                                                                                       
//   	Enable the ADC function, enable IP clock, set operation mode & data format, 
//		but  not start conversion in this API.
//			Need set clock source w/ DrvClk API.
//			Need set the required pins w/ DrvSys API.
//  	Operations to start run ADC:
// 			DrvADC_Open()					// ADC is enabled
//			DrvADC_PreAmpGainControl()		// option
//			DrvADC_EnableSelfCalibration()		// option
//			DrvADC_IsCalibrationDone()			// option
//			DrvADC_SetConversionSequence()	// option
//			DrvADC_EnableCompare0()			// option
//			DrvADC_EnableCompare1()			// option
//			DrvADC_SetConversionSequence()
//			DrvADC_EnableAdcInt()			
//			DrvADC_EnableCompare0Int();		// option
//			DrvADC_EnableCompare1Int();		// option
//			DrvADC_StartConvert()   or   DrvADC_EnableExtTrigger()
//		Operations to stop run ADC:
//			DrvADC_StopConvert() 
//		DrvADC_DisableAdc()
//		DrvADC_Close() 
//--------------------------------------------------------------------------------------------------------------------
void 
DrvADC_Open(void
	//E_DRVADC_OPERATION_MODE eOpMode,
	//UINT8 u8DataFormat
)
{
	DrvCLK_EnableApbDeviceClock(DRVCLK_CLK_ADC);
	DrvSYS_ResetIP(DRVSYS_RST_ADC);
	
	//DrvADC_SetAdcOperationMode(eOpMode);
	//ADC.ADCR.ADFM = u8DataFormat;
	
	//DrvADC_EnableAdc();
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvADC_Close                                                                                  
//                                                                                                         
// Parameters:        	                                                                                   
// 		void                                                                                                
//                                                                                                         
// Returns:                                                                                                
//    	void                                                                                                
//                                                                                                         
// Description:                                                                                            
//    	Stop run the ADC functions, ADC engine clock and ADC interrupt.                                        
//---------------------------------------------------------------------------------------------------------
void 
DrvADC_Close(void)
{     
	ADC.ADCR.u32Reg = 0;
	DrvADC_ClearAdcIntFlag();
	DrvCLK_DisableApbDeviceClock(DRVCLK_CLK_ADC);
}
//-----------------------------------------------------------------------------------------------------------
// Function: DrvADC_GetVersion                                                                               
//                                                                                                           
// Parameters:        	                                                                                     
//    void                                                                                                  
//                                                                                                           
// Returns:                                                                                                  
//    Version number.                                                                                        
//                                                                                                           
// Description:                                                                                              
//    Get the version number of N572 ADC driver.                                                           
//-----------------------------------------------------------------------------------------------------------
UINT32 DrvADC_GetVersion(void)
{
	return DRVADC_VERSION_NUM;
}


//-----------------------------------------------------------------------------------------------------------
// Function: DrvADC_AnalysisAdcCalibration                                                                                  
//                                                                                                         
// Description:                                                                                            
//    	Calibrate ADC DC offset. The fuction will try to scan by setting pre-amp OP offset bias 
//		compensation voltage from 0 to 1F, and get the best offset setting.
//-----------------------------------------------------------------------------------------------------------
//INT16	ai16CalibrationValue[32];

void
DrvADC_AnalysisAdcCalibration(void)
{
	UINT16 u16Offset;
	UINT32 u32Count1;
//	INT16 i16Dummy;

	INT32 	i32TotalAvePCMValue;
	UINT16 	u16CalibrationSampleCount;
	INT32 	i32CurrAcc8PCMValue;
//	INT16 	ai16AdcPcm[8];
	INT16	i16PrevCalibrationValue=2047;


	ADC.PGCR.OPMUTE = 1;

	u16Offset = 0;
	while(1)
	{
		ADC.PGCR.OS = u16Offset;

		for(u32Count1 = 0 ; u32Count1 < 8; ) // wait for a while for ADC value to be stable
		{ 
			if (DrvADC_GetAdcIntFlag())
			{
				DrvADC_ClearAdcIntFlag();
/*
				ai16AdcPcm[0] = DrvADC_GetConversionDataSigned(0);
				ai16AdcPcm[1] = DrvADC_GetConversionDataSigned(1);
				ai16AdcPcm[2] = DrvADC_GetConversionDataSigned(2);
				ai16AdcPcm[3] = DrvADC_GetConversionDataSigned(3);
				ai16AdcPcm[4] = DrvADC_GetConversionDataSigned(4);
				ai16AdcPcm[5] = DrvADC_GetConversionDataSigned(5);
				ai16AdcPcm[6] = DrvADC_GetConversionDataSigned(6);
				ai16AdcPcm[7] = DrvADC_GetConversionDataSigned(7);
*/
				u32Count1++;
			}
		}

		i32TotalAvePCMValue = 0;
		u16CalibrationSampleCount = 0;

		while(1)
		{
			if (DrvADC_GetAdcIntFlag())
			{
				DrvADC_ClearAdcIntFlag();
/*
				ai16AdcPcm[0] = DrvADC_GetConversionDataSigned(0);
				ai16AdcPcm[1] = DrvADC_GetConversionDataSigned(1);
				ai16AdcPcm[2] = DrvADC_GetConversionDataSigned(2);
				ai16AdcPcm[3] = DrvADC_GetConversionDataSigned(3);
				ai16AdcPcm[4] = DrvADC_GetConversionDataSigned(4);
				ai16AdcPcm[5] = DrvADC_GetConversionDataSigned(5);
				ai16AdcPcm[6] = DrvADC_GetConversionDataSigned(6);
				ai16AdcPcm[7] = DrvADC_GetConversionDataSigned(7);

				i32CurrAcc8PCMValue = 	ai16AdcPcm[0]+
										ai16AdcPcm[1]+
										ai16AdcPcm[2]+
										ai16AdcPcm[3]+
										ai16AdcPcm[4]+
										ai16AdcPcm[5]+
										ai16AdcPcm[6]+
										ai16AdcPcm[7];
*/
				i32CurrAcc8PCMValue =  	DrvADC_GetConversionDataSigned(0)+
										DrvADC_GetConversionDataSigned(1)+
										DrvADC_GetConversionDataSigned(2)+
										DrvADC_GetConversionDataSigned(3)+
										DrvADC_GetConversionDataSigned(4)+
										DrvADC_GetConversionDataSigned(5)+
										DrvADC_GetConversionDataSigned(6)+
										DrvADC_GetConversionDataSigned(7);

				i32CurrAcc8PCMValue >>= 3	;
				i32TotalAvePCMValue += i32CurrAcc8PCMValue;
				u16CalibrationSampleCount++;
				if (u16CalibrationSampleCount >= 8)
				{
					break;
				}
			}
		}
		g_i16AveCalibrationValue = (INT16)(i32TotalAvePCMValue >> 3) ;
//		ai16CalibrationValue[u8Offset] = g_i16AveCalibrationValue;
		if (g_i16AveCalibrationValue <= 0)
		{
			if (i16PrevCalibrationValue < -g_i16AveCalibrationValue)
			{
				ADC.PGCR.OS = u16Offset-1;
				g_i16AveCalibrationValue = i16PrevCalibrationValue;
			}
			break;
		}
		else
		{
			i16PrevCalibrationValue = g_i16AveCalibrationValue;
		}
			
		u16Offset++;
#if (defined(__N572F064__)||defined(__N572F065__))
		if (u16Offset == 0x20)
#else
	    if (u16Offset == 0x200)
#endif
			break;
	}

	ADC.PGCR.OPMUTE = 0;
}
//-----------------------------------------------------------------------------------------------------------
// Function: DrvADC_GetAveCalibrationValue                                                                                  
//                                                                                                         
// Description:                                                                                            
//    	get average value of PCM after calibration.
//-----------------------------------------------------------------------------------------------------------
INT16
DrvADC_GetAveCalibrationValue(void)
{
   return (g_i16AveCalibrationValue);
}
