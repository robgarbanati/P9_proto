#include <string.h>
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>
#include "Driver/DrvGPIO.h"
#include "Driver/DrvAPU.h"
#include "Driver/DrvADC.h"
#include "Platform.h"
#include "Spi.h"
#include "DirDetect.h"
#include "Debug.h"

#include "../Shared/Nuvoton/XPrintf.h"

#include "Commu/SoftUART.h"

//
// Global Variables
//

#define GAIN   100000000

volatile uint16_t audio_filter = 0; // received from spi.
volatile UINT16 cry_volume = 0; // Result of cry analysis. To be sent over spi.
int filt_amp_ave_B = 0;
int16_t phaseAB, phaseAC, phaseBC;

//
// Local Variables and Defines
//

static uint16_t cry_thresholdA, cry_thresholdB;

static int16_t ADC_Buf_A[ADC_BUFFER_SIZE]; // Make sure this is the left microphone
static int16_t ADC_Buf_B[ADC_BUFFER_SIZE]; // Make sure this is the top microphone
static int16_t ADC_Buf_C[ADC_BUFFER_SIZE]; // Make sure this is the right microphone

static int16_t BP_Buf_A[ADC_BUFFER_SIZE];

static UINT16	x = 0;
static unsigned short collect_samples;

int Vol_Max, vol_max_A, vol_max_B, vol_max_C;
int Vol_Min, vol_min_A, vol_min_B, vol_min_C;

//
// Local Functions
//

typedef struct
{
    int64_t array[CIRCBUF_LENGTH];
    uint8_t head;
} CircBuf;

// Grab the element at position, 0-indexed from head
int64_t circbuf_access(CircBuf* buffer, uint8_t position, int filter_length)
{
	uint8_t i;
	
	assert(filter_length > 0);
	assert(position < filter_length);
	i = buffer->head + position;
	if(i >= filter_length)
	{
		i -= filter_length;
	}
	return buffer->array[i];
}

void circbuf_push(CircBuf* buffer, int64_t element, int filter_length)
{
	if(buffer->head == 0)
	{
		buffer->head = filter_length-1;
	}
	else
	{
		buffer->head -= 1;
	}
	buffer->array[buffer->head] = element;
}

void circbuf_init(CircBuf* buffer)
{
	int i;
	buffer->head = 0;
	for(i=0;i<CIRCBUF_LENGTH;i++)
	{
		buffer->array[i] = 0;
	}
}


// Handle the direction detection ADC interrupt.
void ADC_IRQHandler()
{
	if (collect_samples) {
		if (DrvADC_GetAdcIntFlag())
		{
			ADC_Buf_A[x] = 	DrvADC_GetConversionDataSigned(0);
			ADC_Buf_B[x] = 	DrvADC_GetConversionDataSigned(1);
			ADC_Buf_C[x] = 	DrvADC_GetConversionDataSigned(2);
			
			if(ADC_Buf_A[x] > vol_max_A)
			{
				vol_max_A = ADC_Buf_A[x];
			}
			if(ADC_Buf_B[x] > vol_max_B)
			{
				vol_max_B = ADC_Buf_B[x];
			}
			if(ADC_Buf_C[x] > vol_max_C)
			{
				vol_max_C = ADC_Buf_C[x];
			}
			
			if(ADC_Buf_A[x] < vol_min_A)
			{
				vol_min_A = ADC_Buf_A[x];
			}
			if(ADC_Buf_B[x] < vol_min_B)
			{
				vol_min_B = ADC_Buf_B[x];
			}
			if(ADC_Buf_C[x] < vol_min_C)
			{
				vol_min_C = ADC_Buf_C[x];
			}
	
			x++;
			
			if (x >= (ADC_BUFFER_SIZE))
			{
				collect_samples = 0;
				x = 0;
			}
		}
	}
	DrvADC_ClearAdcIntFlag();
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

	
	DrvADC_PreAmpGainControl(DRVADC_PAG1_20DB, 0, DRVADC_PAG2_0DB, TRUE);

	DrvADC_SetAdcOperationMode(eDRVADC_CONTINUOUS_SCAN);		    // ap must set to eDRVADC_CONTINUOUS_SCAN for calibration
	DrvADC_SetConversionDataFormat(eDRVADC_2COMPLIMENT);		        // ap must set to DRVADC_2COMPLIMENT for calibration

	DrvADC_SetConversionSequence(eDRVADC_CH0,eDRVADC_CH1,eDRVADC_CH2,eDRVADC_SCANEND,
									eDRVADC_SCANEND,eDRVADC_SCANEND, eDRVADC_SCANEND,eDRVADC_SCANEND);	

	DrvADC_StartConvert();
	DrvADC_AnalysisAdcCalibration();
	DrvAPU_CalibrateDacDcWithAdcDc();
	DrvADC_StopConvert();
}



// Start to record. 
 void start_ADC(void)
{
	 DrvADC_StartConvert();				// start convert
	 SkipAdcUnstableInput(1280);			// skip 128 * 8 samples
	 DrvADC_EnableAdcInt();				//enable ADC interrupt

	 x = 0;
	 collect_samples = 1;
}




// Find the number of indices ADC_Buf_A is shifted from ADC_Buf_B. Return as best_phase.
// best_phase * SAMPLE_FREQUENCY = time delay in audio from microphoneB to microphoneA.
// (positive best_phase means sound hits microphoneB first) TODO: check that statement.
int16_t find_phase(INT16* buf1, INT16* buf2, int vol_min_1, int vol_min_2)
{
	int phase,i;
	int sub = 0, cost = 0;
	int best_phase = 0, bestcost = 0x7FFFFFFF;
	int height_difference = vol_min_1 - vol_min_2;
	int64_t sumcost = 0;
	
	for(phase = -P_E_RES; phase<P_E_RES; phase++) {
		cost = 0;
		for(i = P_E_RES+100; i<ADC_BUFFER_SIZE - P_E_RES; i++) {
			
			sub = (buf1[i + phase] - buf2[i] - height_difference);
			cost = cost + sub*sub;
		}
//		printf("%d %d; ", phase, cost);
		if (bestcost > cost) {
			bestcost = cost;
			best_phase = phase;
			sumcost += cost;
		}
	}
//	printf("%d %lld %d\n", bestcost, sumcost/P_E_RES/2, best_phase);

	return best_phase; 
}


short find_phase32(INT32* buf1, INT32* buf2, INT32 vol_min_1, INT32 vol_min_2)
{
	int phase,i;
	int sub = 0, cost = 0;
	int best_phase = 0, bestcost = 0x7FFFFFFF;
	int height_difference = vol_min_1 - vol_min_2;
	
	for(phase = -P_E_RES; phase<P_E_RES; phase++) {
		cost = 0;
		for(i = P_E_RES; i<ADC_BUFFER_SIZE - P_E_RES; i++) {
			sub = (buf1[i + phase] - buf2[i] - height_difference)>>20;
			cost = cost + sub*sub;
		}
		if (bestcost > cost) {
			bestcost = cost;
			best_phase = phase;
		}
	}

	return best_phase; 
}

int elliptic_filter(int16_t* x, int16_t* y)
{	
//	int audio_filter = SHD_FILTER;
	//  RoR 900-1200
	int64_t RoRa[] = { 1e+08, -4.01365e+08, 8.26119e+08, -1.01624e+09, 8.02648e+08, -3.78857e+08, 9.17189e+07  };
	int64_t RoRb[] = { 264121, -655276, 614196, 0, -614196, 655276, -264121  };
	
	// SHD 1280-1420
	int64_t SHDa[] = { 1e+08, -2.82989e+08, 5.622e+08, -6.41657e+08, 5.54692e+08, -2.75475e+08, 9.6045e+07  };
	int64_t SHDb[] = { 117531, -218753, 211416, 0, -211416, 218753, -117531  };
	
	// FNV 2015-2480
	int64_t FnVa[] = { 1e+08, 9.08875e+07, 3.01767e+08, 1.73437e+08, 2.91438e+08, 8.45221e+07, 8.971e+07  };
	int64_t FnVb[] = { 940714, 519237, 538581, 0, -538581, -519237, -940714  };


	int64_t *a, *b;
	int filter_length, NB, NA;
	
	
    int n, nb, na;
    int64_t bsum, asum, amax=0, bmax=0, ymax=0, ymin=0;

	CircBuf ycirc;
	
	if((audio_filter & 0xF) == ROR_FILTER)
	{
		a = RoRa;
		b = RoRb;
		cry_thresholdA = 80;
		cry_thresholdB = 2000;
		filter_length = NB = NA = 7;
//		printf("af is RoR\n");
	}
	else if((audio_filter & 0xF) == SHD_FILTER)
	{
		a = SHDa;
		b = SHDb;
		cry_thresholdA = 150;
		cry_thresholdB = 1500;
		filter_length =	NB = NA = 7;
//		printf("af is SHD\n");
	}
	else if((audio_filter & 0xF) == FNV_FILTER)
	{
		a = FnVa;
		b = FnVb;
		cry_thresholdA = 250;
		cry_thresholdB = 0xFFFF;
		filter_length = NB = NA = 7;
//		printf("af is FnV\n");
	}
	else // ROR_FILTER
	{
		a = RoRa;
		b = RoRb;
		cry_thresholdA = 0;
		cry_thresholdB = 0;
		filter_length = NB = NA = 7;
	}
	if(audio_filter & 0x80)
	{
		cry_thresholdA = INT16_MAX;
		cry_thresholdB = INT16_MAX;
		return 0;
	}
	
	circbuf_init(&ycirc);
	
    for(n=0;n<ADC_BUFFER_SIZE;n++)
    {
		bsum = 0;
		asum = 0;
		for(nb=0;nb<NB;nb++)
		{
			if((n - nb) < 0)
			{
//				bsum += 0;
				bsum += (int64_t) b[nb] * (int64_t) x[0] * GAIN;
//				printf("b[%d] %lld, x[%d] %d, term %d, bsum %lld\n", nb, b[nb], n-nb, 0, 0, bsum);
			}
			else
			{
				bsum += (int64_t) b[nb] * (int64_t) x[n - nb] * GAIN;
//				printf("b[%d] %lld, x[%d] %d, term %lld, bsum %lld\n", nb, b[nb], n-nb, x[n-nb], b[nb]* (int64_t) x[n-nb], bsum);
			}
		}
		for(na=1;na<NA;na++)
		{
			if((n - na) < 0)
			{
				asum += 0;
			}
			else
			{
				asum += (int64_t) a[na] * (int64_t) circbuf_access(&ycirc, na-1, filter_length);
//				printf("na %d, y[%d] %lld, asum %lld\n", na, n-na, y[n-na], asum);
			}
		}
		

		y[n] = (int16_t) ((bsum - asum) / a[0] / (GAIN/10));
//		y[n] = (int16_t) ((bsum - asum) / a[0] / GAIN);
		circbuf_push(&ycirc, (bsum - asum) / a[0], filter_length);
//		printf("y[%d] %d, ycirc[%d] %lld\n", n, y[n], ycirc.head, circbuf_access(&ycirc, 0));
		
		if(amax < asum)
		{
			amax = asum;
		}
		if(bmax < bsum)
		{
			bmax = bsum;
		}
		if(ymax < y[n])
		{
			ymax = y[n];
		}
		if(ymin > y[n])
		{
			ymin = y[n];
		}
//		printf("bsum %lld, asum %lld, y[%d] %lld\n", bsum, asum, n, y[n]);
    }
	return ymax - ymin;
//    printf("bmax %lld, amax %lld\n", bmax, amax);
}

int abs(int number)
{
	if(number < 0)
	{
		return -number;
	}
	else
	{
		return number;
	}
}


#define DETECTWIDTH 3
void Do_Loop(void)
{	
	static int i = 0, k=0, j=0;
	int filt_amp_A = 0;
	static int filt_amp_ave_A=0;
	static int filt_amp_ave_A_circ[] = {0, 0};
	static int filt_amp_ave_B_circ[30] = {0, };

	//Have we collected an array's worth of data?
	if (!collect_samples)
	{ 
		filt_amp_A = elliptic_filter(ADC_Buf_B, BP_Buf_A);
		
		phaseAB = find_phase(ADC_Buf_A, ADC_Buf_B, vol_min_A, vol_min_B);
		phaseAC = find_phase(ADC_Buf_A, ADC_Buf_C, vol_min_A, vol_min_C);
		phaseBC = find_phase(ADC_Buf_B, ADC_Buf_C, vol_min_B, vol_min_C);

		if((phaseAB >= -3) && (phaseAB <= 5) && (phaseAC >= -4) && (phaseAC <= 4) && (phaseBC >= -5) && (phaseBC <= 3)) // RoR and SHD wide zone
		{
			DrvGPIO_SetOutputBit(&GPIOB, DRVGPIO_PIN_11);
			filt_amp_ave_A -= filt_amp_ave_A_circ[j];
			filt_amp_ave_A_circ[j] = filt_amp_A;
			filt_amp_ave_A += filt_amp_ave_A_circ[j];
			
			filt_amp_ave_B -= filt_amp_ave_B_circ[k];
			filt_amp_ave_B_circ[k] = filt_amp_A;
			filt_amp_ave_B += filt_amp_ave_B_circ[k];
		}
		else
		{
			DrvGPIO_ClearOutputBit(&GPIOB, DRVGPIO_PIN_11);
			filt_amp_ave_A -= filt_amp_ave_A_circ[j];
			filt_amp_ave_A_circ[j] = 0;
			filt_amp_ave_A += filt_amp_ave_A_circ[j];
			
			filt_amp_ave_B -= filt_amp_ave_B_circ[k];
			filt_amp_ave_B_circ[k] = 0;
			filt_amp_ave_B += filt_amp_ave_B_circ[k];
		}
		
		j++; 
		if(j>=2)
		{
			j = 0;
		}
		
		k++;
		if(k>=30)
		{
			k = 0;
		}
		
		if(filt_amp_ave_B > cry_thresholdB)
		{
			cry_volume = filt_amp_ave_B - cry_thresholdB;
			cry_volume /= 10;
			if(cry_volume > 0xFF)
			{
				cry_volume = 0xFF;
			}
		}
		else
		{
			cry_volume = 0;
		}
		 
		i++;
//		printf("AB: %d, AC: %d, BC: %d, amp %d\n", phaseAB, phaseAC, phaseBC, filt_amp_ave_B);
//		printf("amp %d, cv %d, thresh %d, af %d\n", filt_amp_ave_B, cry_volume, cry_thresholdB, audio_filter);
//		printf("AB: %d, AC: %d, BC: %d\n", phaseAB, phaseAC, phaseBC);
		
//#endif
//		// print maybe
#if 0
//		if((abs(phaseAB) > 14) || (abs(phaseAC) > 14) || (abs(phaseBC) > 14))
//		if((phaseAB == 5) && (phaseAC == 4) && (phaseBC == -1))
		{
			for(k = P_E_RES; k<ADC_BUFFER_SIZE - P_E_RES; k++)
			{
				printf("%d, %d, %d, %d\n", k, ADC_Buf_A[k], ADC_Buf_B[k], ADC_Buf_C[k]);
			}
		}
#endif

		
		collect_samples = 1; // let the ADC_ISR take data again
		vol_max_A = INT16_MIN;
		vol_max_B = INT16_MIN;
		vol_max_C = INT16_MIN;
		
		vol_min_A = INT16_MAX;
		vol_min_B = INT16_MAX;
		vol_min_C = INT16_MAX;
	}
}

// enable direction detection
void dirDetectInit(void) {
	
	ADC_Buf_A[0] = 0;
	ADC_Buf_A[1] = 0;
	
	init_ADC();
	start_ADC();
	
	vol_max_A = INT16_MIN;
	vol_max_B = INT16_MIN;
	vol_max_C = INT16_MIN;
	
	vol_min_A = INT16_MAX;
	vol_min_B = INT16_MAX;
	vol_min_C = INT16_MAX;
	
	audio_filter = 0;
	cry_volume = 0;
	filt_amp_ave_B = 0;
	phaseAB = 0;
	phaseAC = 0;
	phaseBC = 0;
	
	cry_thresholdA = 0;
	cry_thresholdB = 0;
}
