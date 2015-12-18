#include <stdio.h>
#include <string.h>
#include "Platform.h"
#include "Driver/DrvGPIO.h"
#include "Driver/DrvSYS.h"
#include "Driver/DrvCLK.h"
#include "Driver/DrvSPI.h"
#include "Driver/DrvPWM.h"
#include "Driver/DrvTimer.h"
#include "SysClkConfig.h"
#include "Debug.h"
#include "Main.h"
#include "Spi.h"
#include "Motor.h"
#include "PWM.h"
#include "Driver/DrvADC.h"
#include "Driver/DrvAPU.h"

int current_sensed;

void ADC_IRQHandler()
{
		if (DrvADC_GetAdcIntFlag())
		{
				current_sensed = DrvADC_GetConversionDataSigned(0);
//				printf("adc result: %d\n", DrvADC_GetConversionDataSigned(0));

			DrvADC_ClearAdcIntFlag();
		}			
}
void init_ADC_clock(void)
{
	UINT32	u32HCLK = 0,u32ADCClk = 0;
//	INT32 adc_clk_freq;
	
	DrvCLK_SetClkSrcAdc(eDRVCLK_ADCSRC_48M);
//	adc_clk_freq = ADC_CLOCK_FREQUENCY;
	
	DrvCLK_SetClkDividerAdc(43);//ADC_CLOCK_FREQUENCY);  // == sampling_frequency*25
	
	u32HCLK = DrvCLK_GetHclk();
	u32ADCClk = DrvCLK_GetClkAdc();

	
	//The ADC engine clock must meet the constraint: ADCLK <=  HCKL/2.
	if (u32ADCClk>(u32HCLK/2)) {
//		puts("Error: ADCClk is greater than half the frequency of the HCLK.\n");
//		puts("Check adcClockInit() in Main.c\n");
	}
}

// wait for a while for ADC value to be stable
static void SkipAdcUnstableInput(UINT16 u16SkipCount)
{
	UINT16 i;
	for(i = 0 ; i < u16SkipCount; ) 
	{ 
		if (DrvADC_GetAdcIntFlag())
		{
			DrvADC_ClearAdcIntFlag();
			i++;
		}
	}
}



// Start to record. 
 void start_ADC(void)
{
	 DrvADC_StartConvert();				// start convert
	 SkipAdcUnstableInput(1280);			// skip 128 * 8 samples
	 DrvADC_EnableAdcInt();				//enable ADC interrupt
}


//	Init ADC. Including:
//  Preamps.   Offset.   Operation Mode.
void init_ADC(void)
{															    
	DrvADC_Open();
	DrvADC_EnableAdc();

	DrvADC_EnableRegulator();
	DrvADC_SetRegulatorRC(eDRVADC_CTRS_R10K,eDRVADC_FWU_R8K);
	DrvTimer_WaitMillisecondTmr2(100);
	DrvADC_SetRegulatorRC(eDRVADC_CTRS_R600K,eDRVADC_FWU_R400K);

	DrvADC_StopConvert();

	
	DrvADC_PreAmpGainControl(DRVADC_PAG1_20DB, 16, DRVADC_PAG2_0DB, TRUE);
//	DrvADC_PreAmpGainControl(DRVADC_PAG1_30DB, 16, DRVADC_PAG2_30DB, TRUE);


	DrvADC_SetAdcOperationMode(eDRVADC_CONTINUOUS_SCAN);
	DrvADC_SetConversionDataFormat(eDRVADC_2COMPLIMENT);

	DrvADC_SetConversionSequence(eDRVADC_CH0,eDRVADC_SCANEND,eDRVADC_SCANEND,eDRVADC_SCANEND,
									eDRVADC_SCANEND,eDRVADC_SCANEND, eDRVADC_SCANEND,eDRVADC_SCANEND);	

	DrvADC_StartConvert();
	DrvADC_AnalysisAdcCalibration();
	DrvAPU_CalibrateDacDcWithAdcDc();
	DrvADC_StopConvert();
	
	start_ADC();
}
