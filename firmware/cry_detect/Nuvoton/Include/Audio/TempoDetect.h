
#ifndef __BEAT_DET_LIB_H__
#define __BEAT_DET_LIB_H__

#include "Platform.h"
#include "System/SysInfra.h"


#define	TEMPODETECT_MAJOR_NUM 3
#define	TEMPODETECT_MINOR_NUM 10
#define	TEMPODETECT_BUILD_NUM 001
#define TEMPODETECT_VERSION_NUM	_SYSINFRA_VERSION(TEMPODETECT_MAJOR_NUM, TEMPODETECT_MINOR_NUM, TEMPODETECT_BUILD_NUM)

#define Z4_1_LAG			4	// 1-Z**4 delay
#define TD_FRAME_SIZE				128	// TempoDetection FRAME_SIZE. == 8ms @16KHz fs

//#define MAGIC_WEIGHTING_NUM			2048		//
//#define MAGIC_DIV_BIT				11		// 

#define MAGIC_WEIGHTING_NUM			512		//
#define MAGIC_DIV_BIT				9		// 120222

#define NUM_TEMPO_LIST		10
#define NUM_PEAK_TEMPO		4

#define BEAT_ADJUST_LIMIT	2
#define MIN_TEMPO_DURATION_BY_FRAME		40	// 40 * 8ms = 320ms @16KHz & FRAME_SIZE = 128 samples == 8 ms	@16KHz
#define MAX_TEMPO_DURATION_BY_FRAME		200	// 200 * 8ms  i.e. 1600 ms @ FRAME_SIZE = 128 samples == 8 ms	@16KHz

#define MAX_NUM_LED_ON_LOCATION	((3 * MAX_TEMPO_DURATION_BY_FRAME / MIN_TEMPO_DURATION_BY_FRAME) + 1)  // 16

struct TempoDetectStructTemp	
{
	UINT16	SubSlotPeakThRef 		; // 
	// function: InitialTempoDetect will clipping MinTempoDurationByFrame no less than MIN_TEMPO_DURATION_BY_FRAME, and MaxTempoDurationByFrame must L.E than MAX_TEMPO_DURATION_BY_FRAME
	INT16	MinTempoDurationByFrame ;	// (45 frame * 128 sample/frame)/Fs = 5760/16000 = 360ms
	INT16	MaxTempoDurationByFrame ;	//  (160 frame * 128 sample/frame)/Fs = 1280ms
	INT16	LED_OnFrameNum			;

	UINT16 usMaxInFrameAry[MAX_TEMPO_DURATION_BY_FRAME * 3]; // 200 frame * 3 * 2B = 1200 Bytes
	INT16 PotentialLEDOnLocation[MAX_NUM_LED_ON_LOCATION]				;// 16
	UINT16 TempoCandiList[NUM_TEMPO_LIST]	;	// 10 
	UINT16 TempoCandiScore[NUM_TEMPO_LIST]	;	// 10 
	//****************************************************************** 640
	INT16 Prev1MaxValue,Prev2MaxValue,Prev3MaxValue	;
	UINT32 ulSubSlotPeakIndicator	;
	UINT16 SubSlotPeakAry[4],SubSlotPeakTem	;
	//****************************************************************** 650
	UINT16 SubSlotPeakTh	;
	INT16 CorrSquBstMantisa[4],CorrSquBstExp[4]				;
	INT16 EgyBstMantisa[4],EgyBstExp[4]			;
	INT16 PeakNIdx[4],PeakBstIdx[4]	;
	//****************************************************************** 675
	UINT16 usSecondSilenceFlag	;
	INT16 NumLEDOnLocation	;
	INT16 NextBeatLocation				;
	UINT16  NewScore	;
	INT32 lDCValue	; // for old version
	UINT32 ulEgy	;	// can not be local.
	UINT16 PeakInTimeSlot	;
	INT16 LocationPeakInTimeSlot	;
	UINT16 MaxValueInFrameAryIdx	;
	//****************************************************************** 686 * 2B = 1372 Bytes
	INT16 Reserve1	;
	INT16 Reserve2	;
	INT16 Reserve3	;
	INT16 Reserve4	;
};

void TempoDetectI_Initial(
	struct TempoDetectStructTemp *pParaStruct 	
	);

void TempoDetectI(
	INT16 i16ADIsrBuf[],
	INT16 i16ADMainBufPageIdx,
	INT16 *pBeatValidFlag,
	INT16 *pLEDOnOffFlag,
	struct TempoDetectStructTemp *pParaStruct,
	INT16 InDataBufNum)	;

UINT32 TempoDetectI_GetVersion(
	void
	);

// for old TempoDetect
void TempoDetect_Initial(
	struct TempoDetectStructTemp *pParaStruct
	);

void TempoDetect(
	INT16 *pi16ADPCM,
	INT16 *pBeatValidFlag,
	INT16 *pLEDOnOffFlag,
	struct TempoDetectStructTemp *pParaStruct);

UINT32 TempoDetect_GetVersion(
	void
	);

#endif





