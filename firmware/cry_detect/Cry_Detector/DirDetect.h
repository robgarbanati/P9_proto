#ifndef __DIRDETECT_H
#define __DIRDETECT_H


//
// Global Defines and Declarations
//
#define SPEED_OF_SOUND 340290 // in mm/s
#define DISTANCE_BETWEEN_MICS 65 // in mm
#define SOUND_TRAVEL_FREQUENCY (SPEED_OF_SOUND/DISTANCE_BETWEEN_MICS + 1) // 1/(amount of time it take sound to travel from one speaker to another)
#define PHASE_ESTIMATION_RESOLUTION 26  //  minimum number of points needed to sample while sound is travelling from one mic to another (in the longest case) to get useful phase estimates
#define P_E_RES PHASE_ESTIMATION_RESOLUTION // easier to read
//#define SAMPLE_FREQUENCY 44100 // necessary sampling frequency to get our phase_estimation_resolution
#define SAMPLE_FREQUENCY 8000 // necessary sampling frequency to get our phase_estimation_resolution
#define CYCLES_PER_CONVERSION 25
#define NUM_CHANNELS 3
#define ADC_CLOCK_FREQUENCY (SAMPLE_FREQUENCY * CYCLES_PER_CONVERSION * NUM_CHANNELS) // necessary clock rate


#define NUM_STF_WAVES_PER_BUFFER 7 // STF = SOUND_TRAVEL_FREQUENCY
#define ADC_BUFFER_SIZE 700 //(NUM_STF_WAVES_PER_BUFFER*PHASE_ESTIMATION_RESOLUTION)
#define NUM_FILTER_COEFFS	9
#define CIRCBUF_LENGTH		NUM_FILTER_COEFFS

#define ROR_FILTER	0x01
#define SHD_FILTER	0x02
#define FNV_FILTER	0x03




//
// Global Functions
//

// Init
void dirDetectInit(void);

void Do_Loop(void);

int elliptic_filter(int16_t* x, int16_t* y);

#endif // __DIRDETECT_H


