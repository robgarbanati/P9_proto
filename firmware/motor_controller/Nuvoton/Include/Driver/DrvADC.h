/********************************************************************
 *																	*
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.	*
 *																	*
 ********************************************************************/

#ifndef __DRVADC_H__
#define __DRVADC_H__

// --------------------------------------------------------------
// Include Header Files
// --------------------------------------------------------------
#include "Platform.h"
#if (defined(__N572F064__)||defined(__N572F065__))
#include "Driver/DrvADCEx_F065.h"
#elif (defined(__N572F072__)||defined(__N572P072__))
#include "Driver/DrvADCEx_F072.h"
#endif

// --------------------------------------------------------------
// Version Definition and Error Code
// --------------------------------------------------------------
#define	DRVADC_MAJOR_NUM 2
#define	DRVADC_MINOR_NUM 00
#define	DRVADC_BUILD_NUM 001

#define DRVADC_VERSION_NUM    _SYSINFRA_VERSION(DRVADC_MAJOR_NUM, DRVADC_MINOR_NUM, DRVADC_BUILD_NUM)

//E_DRVADC_ARGUMENT_OVERFLOW				Argument value is larger than its limitation
//E_DRVADC_CHANNEL_NUMBER_OVERFLOW		    Channel number is larger than 7
#define E_DRVADC_ARGUMENT_OVERFLOW			_SYSINFRA_ERRCODE(TRUE, MODULE_ID_DRVADC, 1)
#define E_DRVADC_CHANNEL_NUMBER_OVERFLOW	_SYSINFRA_ERRCODE(TRUE, MODULE_ID_DRVADC, 2)


//---------------------------------------------------------------
// ADC Operation Mode (ADMD in ADCR)
//---------------------------------------------------------------
typedef enum {
	eDRVADC_SINGLE_CONVERSION	= 0,
	eDRVADC_SINGLE_CYCLE_SCAN	= 2,
	eDRVADC_CONTINUOUS_SCAN		= 3
} E_DRVADC_OPERATION_MODE;

//---------------------------------------------------------------
// ADC Conversion format 
//---------------------------------------------------------------
typedef enum {
	eDRVADC_UNSIGEND	= 0,
	eDRVADC_2COMPLIMENT	= 1
} E_DRVADC_CONVERSION_FORMAT;


//---------------------------------------------------------------
// Trigger Condition (TRGCOND in ADCR)
//---------------------------------------------------------------
typedef enum {
	eDRVADC_LOW_LEVEL		= 0,
	eDRVADC_HIGH_LEVEL		= 1,
	eDRVADC_FALLING_EDGE	= 2,
	eDRVADC_POSITIVE_EDGE	= 3
} E_DRVADC_TRG_COND;

//---------------------------------------------------------------
// Compare Condition (CMPCOND in ADCMPR0/1)
//---------------------------------------------------------------
typedef enum {
	eDRVADC_LESS_THAN			= 0,
	eDRVADC_GREATER_OR_EQUAL	= 1
} E_DRVADC_CMP_COND;

//---------------------------------------------------------------
// ADC Single-Channels And Pair-Channels
//---------------------------------------------------------------
typedef enum {
	 eDRVADC_CH0 = 0x0,
	 eDRVADC_CH1 = 0x1,
	 eDRVADC_CH2 = 0x2,
	 eDRVADC_CH3 = 0x3,
	 eDRVADC_CH4 = 0x4,
	 eDRVADC_CH5 = 0x5,
	 eDRVADC_CH6 = 0x6,
	 eDRVADC_CH7 = 0x7,
	 eDRVADC_CH0CH1 = 0x8,
	 eDRVADC_CH2CH3 = 0xA,
	 eDRVADC_CH4CH5 = 0xC,
	 eDRVADC_NONECH = 0xE,
	 eDRVADC_SCANEND = 0xF,
} E_DRVADC_CHANNEL_NUM;

// --------------------------------------------------------------
// Function Prototypes 
// --------------------------------------------------------------

UINT32
DrvADC_GetVersion(void);

//---------------------------------------------------------------------------------------------------------
// Function: DrvADC_Open                                                                          
//                                                                                     
// Description:                                                                                            
//    	1.Enable  ADC engine and clock.                                                                           
//---------------------------------------------------------------------------------------------------------
void 
DrvADC_Open(void);

//---------------------------------------------------------------------------------------------------------
// Function: DrvADC_Close                                                                          
//                                                                                     
// Description:                                                                                            
//    	1.disable  ADC engine and clock.                                                                           
//---------------------------------------------------------------------------------------------------------
void 
DrvADC_Close(void);

//---------------------------------------------------------------------------------------------------------
// Function: DrvADC_ClearAdcIntFlag                                                                          
//                                                                                     
// Description:                                                                                            
//    	Clear the ADC interrupt flag.                                                                           
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvADC_ClearAdcIntFlag(void)
{
	ADC.ADSR.u32Reg = (ADC.ADSR.u32Reg&(~(ADC_ADSR_CMPF0|ADC_ADSR_CMPF1)))|ADC_ADSR_ADF;
}
				
//---------------------------------------------------------------------------------------------------------
// Function: DrvADC_ClearCompare0IntFlag                                                                       
//                                                                               
// Description:                                                                                            
//    	Clear the ADC compare 0 interrupt flag.                                                                 
//---------------------------------------------------------------------------------------------------------
static __inline
void 
DrvADC_ClearCompare0IntFlag(void)
{
	ADC.ADSR.u32Reg = (ADC.ADSR.u32Reg&(~(ADC_ADSR_ADF|ADC_ADSR_CMPF1)))|ADC_ADSR_CMPF0;
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvADC_ClearCompare1IntFlag                                                                       
//                                                                                      
//                                                                                                         
// Description:                                                                                            
//    	Clear the ADC compare 1 interrupt flag.                                                                 
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvADC_ClearCompare1IntFlag(void)
{
	ADC.ADSR.u32Reg = (ADC.ADSR.u32Reg&(~(ADC_ADSR_ADF|ADC_ADSR_CMPF0)))|ADC_ADSR_CMPF1;
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvADC_EnableAdc                                                                                                                                                                            
//                                                                                                         
// Description:                                                                                            
//    	Enable the ADC function.                                         
//---------------------------------------------------------------------------------------------------------
static __inline
void 
DrvADC_EnableAdc(void)
{
	ADC.ADCR.ADEN = 1;
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvADC_DisableAdc                                                                                                                                                                        
//                                                                                                         
// Description:                                                                                            
//    	Disable the ADC function. ADC should be disabled if not used for saving power.                                        
//---------------------------------------------------------------------------------------------------------
static __inline
void 
DrvADC_DisableAdc(void)
{
	ADC.ADCR.ADEN = 0;
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvADC_EnableAdcInt                                                                                  
//                                                                                                         
// Description:                                                                                            
//    	Enable the ADC interrupt. The interrupt will be issued as it completes conversion.                                      
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvADC_EnableAdcInt(void)
{
	DrvADC_ClearAdcIntFlag();
	ADC.ADCR.ADIE = 1;
	NVIC_EnableIRQ(ADC_IRQn);
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvADC_DisableAdcInt                                                                          
//                                                                                                         
// Description:                                                                                            
//    	Disable the ADC interrupt.                                                                           
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvADC_DisableAdcInt(void)
{
	ADC.ADCR.ADIE = 0;
	DrvADC_ClearAdcIntFlag();
	NVIC_DisableIRQ(ADC_IRQn);
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvADC_GetAdcIntFlag                                                                       
//                                                                                                         
// Returns:                                                                                                
//    	TRUE:  the conversion action is finished.                                                             
//    	FALSE: the conversion action is in progress.                                                         
//                                                                                                         
// Description:                                                                                            
//    	Check whether the conversion action is finished or not.                                              
//---------------------------------------------------------------------------------------------------------
static __inline
BOOL 
DrvADC_GetAdcIntFlag(void)
{
	return ADC.ADSR.ADF;
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvADC_GetConversionDataUnsigned                                                                      
//                                                                                                         
// Parameters:        	                                                                                   
//    	u8ScanSeq 		[in]  Specify the ADC scan sequence.                                                         
//                                                                                                         
// Returns:                                                                                                
//    	unsigned 12-bit conversion result. 
//                                                                                                         
// Description:                                                                                            
//    	Get the conversion result of the specified ADC scan sequence.  
//		Must consistent w/ the data format was set in DrvADC_Open().                                             
//---------------------------------------------------------------------------------------------------------
static __inline
UINT16 
DrvADC_GetConversionDataUnsigned(
	UINT8 u8ScanSeq)
{
	return (UINT16)ADC.ADDR[u8ScanSeq].RSLT;
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvADC_GetConversionDataSigned                                                                      
//                                                                                                         
// Parameters:        	                                                                                   
//    	u8ScanSeq 		[in]  Specify the ADC scan sequence.                                                         
//                                                                                                         
// Returns:                                                                                                
//    	Signed 12-bit conversion result. 
//                                                                                                         
// Description:                                                                                            
//    	Get the conversion result of the specified ADC channel. 
//		Must consistent w/ the data format was set in DrvADC_Open().
//---------------------------------------------------------------------------------------------------------
static __inline
INT16 
DrvADC_GetConversionDataSigned(
	UINT8 u8ScanSeq)
{
	return (INT16)ADC.ADDR[u8ScanSeq].RSLT;
}																 

//---------------------------------------------------------------------------------------------------------
// Function: DrvADC_IsDataValid                                                                            
//                                                                                                         
// Parameters:        	                                                                                   
//   	u8ScanSeq        [in]  Specify the ADC scan sequence.                                                          
//                                                                                                         
// Returns:                                                                                                
//    	TRUE: valid.                                                                                         
//    	FALSE: invalid.                                                                                      
//                                                                                                         
// Description:                                                                                            
//    	Check whether the conversion data is valid or not.                                                   
//---------------------------------------------------------------------------------------------------------
static __inline
BOOL 
DrvADC_IsDataValid(
	UINT8 u8ScanSeq)
{
	return ADC.ADDR[u8ScanSeq].VALID;
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvADC_IsDataOverrun                                                                          
//                                                                                                         
// Parameters:        	                                                                                   
//   	eScanSeq       [in]  Specify the ADC scan sequence.                                                          
//                                                                                                         
// Returns:                                                                                                
//    	TRUE: overrun.                                                                                       
//    	FALSE: non-overrun.                                                                                  
//                                                                                                         
// Description:                                                                                            
//    	Check whether the conversion data is overrun or not.                                                 
//---------------------------------------------------------------------------------------------------------
static __inline
BOOL 
DrvADC_IsDataOverrun(
	UINT8 u8ScanSeq)
{
	return ADC.ADDR[u8ScanSeq].OVERRUN;
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvADC_GetCurConvertCH                                                                       
//                                                                                                         
// Returns:                                                                                                
//    	 Current conversion channel number.                                                       
//                                                                                                         
// Description:                                                                                            
//    	This filed reflects current conversion channel when BUSY=1. 
//		When BUSY=0, it shows the next channel will be converted.                                              
//---------------------------------------------------------------------------------------------------------
static __inline
UINT8
DrvADC_GetCurConvertCH(void)
{
   	return ADC.ADSR.CHANNEL;
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvADC_IsChannelBusy                                                                       
//                                                                                                         
// Returns:                                                                                                
//    	TRUE:  A/D converter is busy at conversion.                                                             
//    	FALSE: A/D converter is in idle state.                                                         
//                                                                                                         
// Description:                                                                                            
//    	Check busy state accroding current channel.                                              
//---------------------------------------------------------------------------------------------------------
static __inline
BOOL
DrvADC_IsChannelBusy(void)
{
	return ADC.ADSR.BUSY;
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvADC_GetCompare0IntFlag                                                                       
//                                                                                                         
// Returns:                                                                                                
//    	TRUE:  Conversion result in ADDR meets ADCMPR0 setting.                                                             
//    	FALSE: Conversion result in ADDR does not meet ADCMPR0 setting.                                                         
//                                                                                                         
// Description:                                                                                            
//    	Check whether the conversion result meets setting condition or not.                                              
//---------------------------------------------------------------------------------------------------------
static __inline
BOOL 
DrvADC_GetCompare0IntFlag(void)
{
	return ADC.ADSR.CMPF0;
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvADC_GetCompare1IntFlag                                                                       
//                                                                                                         
// Returns:                                                                                                
//    	TRUE:  Conversion result in ADDR meets ADCMPR1 setting.                                                            
//    	FALSE: Conversion result in ADDR does not meet ADCMPR1 setting.                                                         
//                                                                                                         
// Description:                                                                                            
//    	Check whether the conversion result meets setting condition or not.                                              
//---------------------------------------------------------------------------------------------------------
static __inline
BOOL 
DrvADC_GetCompare1IntFlag(void)
{
	return ADC.ADSR.CMPF1;
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvADC_SetAdcOperationMode                                                                    
//                                                                                                         
// Parameters:        	                                                                                   
//    	eOpMode 	[in]  Specify the operation mode.                                                            
//                                                                                                         
// Description:                                                                                            
//    	Set the ADC operation mode. Must stop ADC conversion before change mode.
//---------------------------------------------------------------------------------------------------------
static __inline
void 
DrvADC_SetAdcOperationMode(
	E_DRVADC_OPERATION_MODE eOpMode)
{
	ADC.ADCR.ADMD = eOpMode;
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvADC_EnableExtTrigger                                                                       
//                                                                                                         
// Parameters:        	                                                                                   
//    	TriggerCondition 	[in]  Specify the trigger condition.                                               
//                                                                                                         
// Description:                                                                                            
//    	Allow the external trigger pin STADC (PB8) to be the trigger source of ADC.                          
//    	The trigger condition could be low-level/high-level/falling-edge/positive-edge.                      
//    	TRGE bit and ADST bit should be clear to 0 before changing TRGS bit.                                 
//---------------------------------------------------------------------------------------------------------
static __inline
void 
DrvADC_EnableExtTrigger(
	E_DRVADC_TRG_COND eTrgCond)
{
	ADC.ADCR.TRGCOND = eTrgCond;	// Set the trigger condition 
	ADC.ADCR.TRGEN = 1;				// Enable the trigger source of STADC pin 
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvADC_DisableExtTrigger                                                                      
//                                                                                                         
// Description:                                                                                            
//    	Prohibit the external ADC trigger.                                                                   
//---------------------------------------------------------------------------------------------------------
static __inline
void 
DrvADC_DisableExtTrigger(void)
{
	ADC.ADCR.TRGEN = 0;
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvADC_StartConvert                                                                           
//                                                                                                         
// Description:                                                                                            
//    	Set the ADST bit of ADCR to start the A/D conversion action.                                         
//---------------------------------------------------------------------------------------------------------
static __inline
void 
DrvADC_StartConvert(void)
{
	ADC.ADCR.ADST = 1;
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvADC_StopConvert                                                                            
//                                                                                                         
// Description:                                                                                            
//    	Clear the ADST bit of ADCR to stop the A/D conversion action.                                        
//---------------------------------------------------------------------------------------------------------
static __inline
void 
DrvADC_StopConvert(void)
{
	ADC.ADCR.ADST = 0;
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvADC_EnableCompare0Int                                                                        
//                                                                                                         
// Description:                                                                                            
//    	Enable the ADC compare 0 interrupt.                                      
//---------------------------------------------------------------------------------------------------------
static __inline
void 
DrvADC_EnableCompare0Int(void)
{
	DrvADC_ClearCompare0IntFlag();
	ADC.ADCMPR[0].CMPIE = 1;
	NVIC_EnableIRQ(ADC_IRQn);
}
//---------------------------------------------------------------------------------------------------------
// Function: DrvADC_DisableCompare0Int                                                                       
//                                                                                                         
// Description:                                                                                            
//    	Disable the ADC compare 0 interrupt.                                                                 
//---------------------------------------------------------------------------------------------------------
static __inline
void 
DrvADC_DisableCompare0Int(void)
{
	ADC.ADCMPR[0].CMPIE = 0;
	DrvADC_ClearCompare0IntFlag();
	NVIC_DisableIRQ(ADC_IRQn);
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvADC_EnableCompare1Int                                                                        
//                                                                                                         
// Description:                                                                                            
//   	Enable the ADC compare 1 interrupt.                                      
//---------------------------------------------------------------------------------------------------------
static __inline
void 
DrvADC_EnableCompare1Int(void)
{
	DrvADC_ClearCompare1IntFlag();
	ADC.ADCMPR[1].CMPIE = 1;
	NVIC_DisableIRQ(ADC_IRQn);
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvADC_DisableCompare1Int                                                                       
//                                                                                                         
// Description:                                                                                            
//    	Disable the ADC compare 1 interrupt.                                                                 
//---------------------------------------------------------------------------------------------------------
static __inline
void 
DrvADC_DisableCompare1Int(void)
{
	ADC.ADCMPR[1].CMPIE = 0;
	DrvADC_ClearCompare1IntFlag();
	NVIC_DisableIRQ(ADC_IRQn);
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvADC_SetConversionDataFormat                                                                      
//                                                                                                         
// Parameters:        	                                                                                   
//    	eDataFormat 		       [in]  Data Format of ADC Conversion Result.                                                         
//                                 1. DRVADC_2COMPLIMENT                                                           
//								   2. DRVADC_UNSIGNED				
// Returns:                                                                                                
//    	void. 
//                                                                                                         
// Description:                                                                                            
//       Set  ADC conversion result data format.                                
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvADC_SetConversionDataFormat(
	E_DRVADC_CONVERSION_FORMAT	eDataFormat
)
{
	ADC.ADCR.ADFM = eDataFormat;	
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvADC_SetConversionSequence                                                                          
//                                                                                                         
// Parameters:        	                                                                                   
//   	eSeq0~eSeq7 	[in]	Specify the ADC channels in conversion sequence.                                                          
//                                  0000~0111 	: channel number
//									1000		: input pair of pre-amplifier channel 0/1
//									1010		: input pair of pre-amplifier channel 2/3
//									1100		: input pair of pre-amplifier channel 4/5
//									1110		: none
//									1xx1		: no channel is selected, scan sequence end.
//									user can refer to E_CHANNEL_NUM.
// Description:                                                                                            
//    	Specify the ADC channels in conversion sequence.                                                 
//---------------------------------------------------------------------------------------------------------
static __inline
void 
DrvADC_SetConversionSequence(
	E_DRVADC_CHANNEL_NUM eSeq0,
	E_DRVADC_CHANNEL_NUM eSeq1,
	E_DRVADC_CHANNEL_NUM eSeq2,
	E_DRVADC_CHANNEL_NUM eSeq3,
	E_DRVADC_CHANNEL_NUM eSeq4,
	E_DRVADC_CHANNEL_NUM eSeq5,
	E_DRVADC_CHANNEL_NUM eSeq6,
	E_DRVADC_CHANNEL_NUM eSeq7
)
{
	ADC.ADCHSR.u32Reg = eSeq0 + (eSeq1 << 4) + (eSeq2 << 8) + (eSeq3 << 12) + 
						(eSeq4 << 16) + (eSeq5 << 20) + (eSeq6 << 24) + (eSeq7 << 28);
}

//-----------------------------------------------------------------------------------------------------------
// Function: DrvADC_EnableCompare0                                                                             
//                                                                                                           
// Parameters:        	                                                                                     
//    	u8CmpChannelNum 	[in]:  	Specify the channel number that want to compare.                                
//    	eCmpCond    	¡@¡@[in]:  	Specify the compare condition.                                                  
//                           		eDrvADC_LESS_THAN for the condition of "less than the compare data";                    
//                           		eDrvADC_GREATER_OR_EQUAL for the condition of "greater or equal to the compare data."   
//    	u16CmpData      	[in]:  	Specify the compare data.                                                       
//    	u8CmpMatchCount 	[in]:  	Specify the compare match count.                                                
//                                                                                                           
// Description:                                                                                              
//    	Enable the ADC result monitor 0 and configure the necessary settings. 
//		Compare match interrupt will be issued as match count is up.
//-----------------------------------------------------------------------------------------------------------
static __inline
void 
DrvADC_EnableCompare0(
	UINT8  u8CmpChannelNum, 
	E_DRVADC_CMP_COND  eCmpCond, 
	UINT16 u16CmpData, 
	UINT8  u8CmpMatchCount)
{
	ADC.ADCMPR[0].u32Reg = (u16CmpData << 16) + (u8CmpMatchCount << 8) + (u8CmpChannelNum << 3) + (eCmpCond << 2) + 1;
}

//-----------------------------------------------------------------------------------------------------------
// Function: DrvADC_DisableCompare0                                                                           
//                                                                                                           
// Description:                                                                                              
//    	Disable the ADC result monitor 0.                                                                      
//-----------------------------------------------------------------------------------------------------------
static __inline
void 
DrvADC_DisableCompare0(void)
{
	ADC.ADCMPR[0].CMPEN = 0;
}

//-----------------------------------------------------------------------------------------------------------
// Function: DrvADC_EnableCompare1                                                                             
//                                                                                                           
// Parameters:        	                                                                                     
//    	u8CmpChannelNum  [in]:  Specify the channel number that want to compare.                                
//    	eCmpCond   ¡@¡@  [in]:  Specify the compare condition.                                                  
//           	                eDrvADC_LESS_THAN for the condition of "less than the compare data";                    
//           	                eDrvADC_GREATER_OR_EQUAL for the condition of "greater or equal to the compare data."   
//    	u16CmpData       [in]:  Specify the compare data.                                                       
//    	u8CmpMatchCount  [in]:  Specify the compare match count.                                                
//                                                                                                           
// Description:                                                                                              
//    	Enable the ADC result monitor 1 and configure the necessary settings.                                  
//-----------------------------------------------------------------------------------------------------------
static __inline
void 
DrvADC_EnableCompare1(
	UINT8  u8CmpChannelNum, 
	E_DRVADC_CMP_COND  eCmpCond, 
	UINT16 u16CmpData, 
	UINT8  u8CmpMatchCount)
{
	ADC.ADCMPR[1].u32Reg = (u16CmpData << 16) + (u8CmpMatchCount << 8) + (u8CmpChannelNum << 3) + (eCmpCond << 2) + 1;
}

//-----------------------------------------------------------------------------------------------------------
// Function: DrvADC_DisableCompare1                                                                            
//                                                                                                           
// Description:                                                                                              
//    	Disable the ADC result monitor 1.                                                                      
//-----------------------------------------------------------------------------------------------------------
static __inline
void 
DrvADC_DisableCompare1(void)
{
	ADC.ADCMPR[1].CMPEN = 0;
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvADC_SetGain                                                                              
//                                                                                                         
// Parameters:        	                                                                                   
//    	u8Gain1								[in]	0~3. Gain setting bits for the first stage of pre-amp.
//		u8Gain2                         	[in]	0~31. Gain setting bits for the second stage of pre-amp.                                                                                                                                                                   
// Description:                                                                                            
//    	Set Gain1 and Gain2 of pre-amplifier .                                        
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvADC_SetGain(
	UINT8 u8Gain1,
	UINT8 u8Gain2
)
{
	ADC.PGCR.PAG_1 = u8Gain1; 
	ADC.PGCR.PAG_II = u8Gain2; 
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvADC_PreAmpGainControl                                                                              
//                                                                                                         
// Parameters:        	                                                                                   
//    	u8PreAmpGain1								[in]	0~3. Gain setting bits for the first stage of pre-amp.
//		u8BiasCompensation                      	[in]	0~31. Configuration for Pre-amp OP offset bias compensation voltage.
//		u8PreAmpGain2								[in]	0~31. Gain setting bits for the second stage of pre-amp.
//		bOpMuteControl                             [in]    0:Open, 1:short. Mute control of first stage pre-amp for offset bias calibration.                                                            
//                                                                                                         
// Description:                                                                                            
//    	Set the parameters of pre-amplifier gain control.                                        
//---------------------------------------------------------------------------------------------------------
static __inline
void
DrvADC_PreAmpGainControl(
	UINT8  u8PreAmpGain1,
	UINT8  u8BiasCompensation,
	UINT8  u8PreAmpGain2,
	BOOL   bOpMuteControl)
{

	//ADC.PGCR.u32Reg = bOpMuteControl|(u8PreAmpGain2<<1)|(u8BiasCompensation<<6)|(u8PreAmpGain1<<11);
	ADC.PGCR.OPMUTE = bOpMuteControl;
	ADC.PGCR.PAG_II = u8PreAmpGain2;
	ADC.PGCR.OS = u8BiasCompensation;
	ADC.PGCR.PAG_1 =  u8PreAmpGain1;
}

//---------------------------------------------------------------------------------------------------------
// Function: DrvADC_AnalysisAdcCalibration                                                                              
//                                                                                                         
// Description:                                                                                            
//    	Calibrate ADC DC offset. The fuction will try to scan by setting pre-amp OP offset bias 
//		compensation voltage from 0 to 1F, and get the best offset setting.
//---------------------------------------------------------------------------------------------------------
void
DrvADC_AnalysisAdcCalibration(void);

//---------------------------------------------------------------------------------------------------------
// Function: DrvADC_GetAveCalibrationValue                                                                              
//                                                                                                         
// Description:                                                                                            
//    	get average value of PCM after calibration.
//---------------------------------------------------------------------------------------------------------
INT16
DrvADC_GetAveCalibrationValue(void);

#endif


