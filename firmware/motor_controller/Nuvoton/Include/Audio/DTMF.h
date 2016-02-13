#ifndef __DTMF_H__
#define __DTMF_H__

#include "Platform.h"
#include "System/SysInfra.h"

#ifdef  __cplusplus
extern "C"
{
#endif

#define	DTMF_MAJOR_NUM 3
#define	DTMF_MINOR_NUM 0
#define	DTMF_BUILD_NUM 1
#define DTMF_VERSION_NUM	_SYSINFRA_VERSION(DTMF_MAJOR_NUM, DTMF_MINOR_NUM, DTMF_BUILD_NUM)

#define ADC_FREQENCY 36000 // 36226.415
#define DOWN_SAMPLE_SIZE 9

#define FFT_SAMPLE_COUNT 128
#define FFT_BASE_NUM 7

#define ADC_BUFFER_SIZE (FFT_SAMPLE_COUNT) // 128

#define LAST_FFT_SAMPLE_COUNT (FFT_SAMPLE_COUNT*2) // 256
#define LAST_FFT_BASE_NUM (FFT_BASE_NUM+1) // 8

//#define BASE_BAND 15.625
#define BASE_BAND 15.723

/*#define DTMF_LF0 (697) // 44.608
#define DTMF_LF1 (770) // 49.28
#define DTMF_LF2 (852) // 54.528
#define DTMF_LF3 (941) // 60.224

#define DTMF_HF0 (1209) // 77.376
#define DTMF_HF1 (1336) // 85.504
#define DTMF_HF2 (1477) // 94.528
#define DTMF_HF3 (1633) // 104.512*/

#define DTMF_LF0 (697) // 44.33
#define DTMF_LF1 (770) // 48.97
#define DTMF_LF2 (852) // 54.188
#define DTMF_LF3 (941) // 59.849

#define DTMF_HF0 (1209) // 76.893
#define DTMF_HF1 (1336) // 84.971
#define DTMF_HF2 (1477) // 93.939
#define DTMF_HF3 (1633) // 103.86

/*#define ENG_LF0	((i16FFTEnergy[44]+i16FFTEnergy[45])/2)
#define ENG_LF1	(i16FFTEnergy[49])
#define ENG_LF2	((i16FFTEnergy[54]+i16FFTEnergy[55])/2)
#define ENG_LF3	(i16FFTEnergy[60])

#define ENG_HF0	(i16FFTEnergy[77])
#define ENG_HF1	((i16FFTEnergy[85]+i16FFTEnergy[86])/2)
#define ENG_HF2	((i16FFTEnergy[94]+i16FFTEnergy[95])/2)
#define ENG_HF3	(i16FFTEnergy[104])*/

#define ENG_LF0	((i16FFTEnergy[44]*7+i16FFTEnergy[45]*3)/10)
#define ENG_LF1	(i16FFTEnergy[49])
#define ENG_LF2	((i16FFTEnergy[54]*8+i16FFTEnergy[55]*2)/10)
#define ENG_LF3	(i16FFTEnergy[60])

#define ENG_HF0	(i16FFTEnergy[77])
#define ENG_HF1	(i16FFTEnergy[85])
#define ENG_HF2	(i16FFTEnergy[94])
#define ENG_HF3	(i16FFTEnergy[104])

#define DTMF_KEY_1	(DTMF_LF0+(DTMF_HF0<<16))
#define DTMF_KEY_2	(DTMF_LF0+(DTMF_HF1<<16))
#define DTMF_KEY_3	(DTMF_LF0+(DTMF_HF2<<16))
#define DTMF_KEY_A	(DTMF_LF0+(DTMF_HF3<<16))

#define DTMF_KEY_4	(DTMF_LF1+(DTMF_HF0<<16))
#define DTMF_KEY_5	(DTMF_LF1+(DTMF_HF1<<16))
#define DTMF_KEY_6	(DTMF_LF1+(DTMF_HF2<<16))
#define DTMF_KEY_B	(DTMF_LF1+(DTMF_HF3<<16))

#define DTMF_KEY_7	(DTMF_LF2+(DTMF_HF0<<16))
#define DTMF_KEY_8	(DTMF_LF2+(DTMF_HF1<<16))
#define DTMF_KEY_9	(DTMF_LF2+(DTMF_HF2<<16))
#define DTMF_KEY_C	(DTMF_LF2+(DTMF_HF3<<16))

#define DTMF_KEY_STAR	(DTMF_LF3+(DTMF_HF0<<16))
#define DTMF_KEY_0		(DTMF_LF3+(DTMF_HF1<<16))
#define DTMF_KEY_POUND	(DTMF_LF3+(DTMF_HF2<<16))
#define DTMF_KEY_D		(DTMF_LF3+(DTMF_HF3<<16))

//#define DTMF_THRESHOLD 30
#define DTMF_THRESHOLD 7

#define B32_1	((long)(0.008434215585687*4096.0))	// 20,12
#define B32_2	((long)(0.01680253678403*4096.0))
#define B32_3	((long)(0.05081803670986*4096.0))
#define B32_4	((long)(0.08704822052922*4096.0))
#define B32_5	((long)(0.1489263128471*4096.0))
#define B32_6	((long)(0.2063081917872*4096.0))
#define B32_7	((long)(0.2608638358112*4096.0))
#define B32_8	((long)(0.290515710051*4096.0))
#define B32_9	((long)(0.290515710051*4096.0))
#define B32_a	((long)(0.2608638358112*4096.0))
#define B32_b	((long)(0.2063081917872*4096.0))
#define B32_c	((long)(0.1489263128471*4096.0))
#define B32_d	((long)(0.08704822052922*4096.0))
#define B32_e	((long)(0.05081803670986*4096.0))
#define B32_f	((long)(0.01680253678403*4096.0))
#define B32_g	((long)(0.008434215585687*4096.0))
#define A32_1	((long)(1*4096.0))
#define A32_2	((long)(-2.442269871492*4096.0))
#define A32_3	((long)(5.274235358148*4096.0))
#define A32_4	((long)(-6.520449443674*4096.0))
#define A32_5	((long)(7.381559968222*4096.0))
#define A32_6	((long)(-5.601044570487*4096.0))
#define A32_7	((long)(4.115901683336*4096.0))
#define A32_8	((long)(-1.964487235206*4096.0))
#define A32_9	((long)(1.037296033321*4096.0))
#define A32_a	((long)(-0.273670476904*4096.0))
#define A32_b	((long)(0.1274948394229*4096.0))
#define A32_c	((long)(-0.006562780932886*4096.0))
#define A32_d	((long)(0.009421285851874*4096.0))
#define A32_e	((long)(0.001449411576272*4096.0))
#define A32_f	((long)(0.0004899735484435*4096.0))
#define A32_g	((long)(6.994548005911e-005*4096.0))

#define B3_1	((long)(0.01781199707095*32768.0)) // 17.15
#define B3_2	((long)(0.06634323333861*32768.0))
#define B3_3	((long)(0.1401351686074*32768.0))
#define B3_4	((long)(0.1972791523876*32768.0))
#define B3_5	((long)(0.1972791523876*32768.0))
#define B3_6	((long)(0.1401351686074*32768.0))
#define B3_7	((long)(0.06634323333861*32768.0))
#define B3_8	((long)(0.01781199707095*32768.0))
#define A3_1	((long)(1*32768.0))
#define A3_2	((long)(-1.256139068995*32768.0))
#define A3_3	((long)(1.628496269579*32768.0))
#define A3_4	((long)(-0.9227149650011*32768.0))
#define A3_5	((long)(0.4895605849512*32768.0))
#define A3_6	((long)(-0.1203912733109*32768.0))
#define A3_7	((long)(0.02581960350569*32768.0))
#define A3_8	((long)(-0.001492047919986*32768.0))


#define C0_1	((long)(-0.010973538990026165*32768.0))  // 17.15
#define C0_2	((long)( 0.020308892934519363*32768.0))
#define C0_3	((long)( 0.030703982548687992*32768.0))
#define C0_4	((long)( 0.034936272094112854*32768.0))
#define C0_5	((long)( 0.019653277584088791*32768.0))
#define C0_6	((long)(-0.021422111809785566*32768.0))
#define C0_7	((long)(-0.083688404955026455*32768.0))
#define C0_8	((long)(-0.15178870758520538*32768.0))
#define C0_9	((long)(-0.20476856453507969*32768.0))
#define C0_a	((long)( 0.77523266484655262*32768.0))
#define C0_b	((long)(-0.20476856453507969*32768.0))
#define C0_c	((long)(-0.15178870758520538*32768.0))
#define C0_d	((long)(-0.083688404955026455*32768.0))
#define C0_e	((long)(-0.021422111809785566*32768.0))
#define C0_f	((long)( 0.019653277584088791*32768.0))
#define C0_g	((long)( 0.034936272094112854*32768.0))
#define C0_h	((long)( 0.030703982548687992*32768.0))
#define C0_i	((long)( 0.020308892934519363*32768.0))
#define C0_j	((long)(-0.010973538990026165*32768.0))


#define FIX_POINT long

void DTMF_CalTone(void);
void CalBandEnergy(INT16 *pEng,INT32 m,FIX_POINT *pRe,FIX_POINT *pImg);
INT16 DTMF_NoiseFilter_32(INT16 i16Pcm);
INT16 DTMF_NoiseFilter_31(INT16 i16Pcm);
INT16 DTMF_NoiseFilter_HIGH_PASS(INT16 i16Pcm);
void DTMF_Init(PINT8 ptr);
void DTMF_ProcessPCM(PINT16 ptr);
INT32 DTMF_CheckTone(void);

#ifdef  __cplusplus
}
#endif

#endif
