#ifndef _AUDIOSYNCEX_H_
#define _AUDIOSYNCEX_H_

#include "Platform.h"

#include "Audio/AudioCommon.h"
#include "Audio/NuSoundEx.h"
#include "Audio/NuVox53Ex.h"
#include "Audio/NuVox63Ex.h"
#ifndef __N571__
#include "Audio/NuOneEx.h"
#endif
#include "Audio/NuLiteEx.h"
#include "Audio/MD4.h"
#include "Audio/LP8.h"
#include "Audio/P16.h"
#include "Audio/MidiSynthEx.h"

extern void MIDISynthEx_Config(UINT8 *pu8DecodeWorkBuf,		
	UINT32 u32StartAddr, UINT32 u32DataSize, PFN_AUDIO_DATAREQUEST pfnReadDataCallback);

#ifdef	__cplusplus
extern "C"
{
#endif

#define AUDIOSYNTHEX_SIL_WORK_BUF_SIZE		4
#define AUDIOSYNTHEX_SIL_TEMP_BUF_SIZE		0
#define AUDIOSYNTHEX_SIL_SAMPLE_PER_FRAME	8


typedef struct sAudioSynthExCallBack
{
	PFN_AUDIO_DATAREQUEST		pfnReadDataCallback;
	PFN_AUDIO_USREVENT			pfnUserEventCallback;
}S_AUDIOSYNTHEX_CALLBACK;


typedef struct sAudioSynthExDecoder
{
	PFN_AUDIO_DECODEINITIATE	pfnDecodeInitiate;
	PFN_AUDIO_DECODEPROCESS		pfnDecodeProcess;
	PFN_AUDIO_DECODEISEND		pfnDecodeIsEnd;
	UINT16 u16FormatID;
}S_AUDIOSYNTHEX_DECODER;

typedef struct sAudioSynthExDesc
{
	UINT16 u16TotalAudioNum;
	UINT16 u16TotalEquationNum;
	UINT32 u32SentenceStartAddr;
	
	UINT32 u32EquDataAddr;
	UINT32 u32AudioChunkAddr;
	UINT32 u32AudioChunkSize;
	S_AUDIOSYNTHEX_DECODER *pfnDecoder;
	UINT16 u16DecodedDataCount;
	UINT16 u16ReadDataCount;

	UINT8  u16Repeat;
	UINT8  u8StorIndex;
	UINT8  u8ChannelID;	
	UINT8  u8CtrlFlag;			// bit0: index which half PCM is using
}S_AUDIOSYNTHEX_DESC;
#define AUDIOSYNTHEX_DESC_CTRLFLAG_DECODING_BUF	BIT0

typedef struct sAudioChannelInfo
{
	S_AUDIOSYNTHEX_DESC*	psDescriptor;
	UINT8*	pu8DecodeWorkBuf;
	UINT8*	pu8DecodeTempBuf;
	INT16*	pi16DecodePCMBuf;
	UINT16  u16DecodePCMBufCount;
}S_AUDIOSYNTHEX_CHANNEL_INFO;
 
#define AUDIOSYNTHEX_P16_DECODER()			{P16_DecodeInitiate, P16_DecodeProcess, P16_DecodeIsEnd, AUDIO_FMT_PCM16}
#define AUDIOSYNTHEX_LP8_DECODER()			{LP8_DecodeInitiate, LP8_DecodeProcess, LP8_DecodeIsEnd, AUDIO_FMT_LP8}
#define AUDIOSYNTHEX_MD4_DECODER()			{MD4_DecodeInitiate, MD4_DecodeProcess, MD4_DecodeIsEnd, AUDIO_FMT_MDPCM4}
#define AUDIOSYNTHEX_MIDISYNTHEX_DECODER()	{MIDISynthEx_DecodeInitiate, MIDISynthEx_DecodeProcess, MIDISynthEx_DecodeIsEnd, AUDIO_FMT_IMFMIDI}
#define AUDIOSYNTHEX_NUSOUNDEX_DECODER()	{NuSoundEx_DecodeInitiate, NuSoundEx_DecodeProcess, NuSoundEx_DecodeIsEnd, AUDIO_FMT_NUSOUND}
#ifndef __N571__
#define AUDIOSYNTHEX_NUONEEX_DECODER()		{NuOneEx_DecodeInitiate, NuOneEx_DecodeProcess, NuOneEx_DecodeIsEnd, AUDIO_FMT_NUONE},{NuOneEx_DecodeInitiate, NuOneEx_DecodeProcess, NuOneEx_DecodeIsEnd, AUDIO_FMT_NUONE_E}
#endif
#define AUDIOSYNTHEX_NULITEEX_DECODER()		{NuLiteEx_DecodeInitiate, NuLiteEx_DecodeProcess, NuLiteEx_DecodeIsEnd, AUDIO_FMT_NULITE},{NuLiteEx_DecodeInitiate, NuLiteEx_DecodeProcess, NuLiteEx_DecodeIsEnd, AUDIO_FMT_NULITE_E}
#define AUDIOSYNTHEX_NUVOX53EX_DECODER()	{NuVox53Ex_DecodeInitiate, NuVox53Ex_DecodeProcess, NuVox53Ex_DecodeIsEnd, AUDIO_FMT_NUVOX53}
#define AUDIOSYNTHEX_NUVOX63EX_DECODER()	{NuVox63Ex_DecodeInitiate, NuVox63Ex_DecodeProcess, NuVox63Ex_DecodeIsEnd, AUDIO_FMT_NUVOX53}

// Initiate library:
//
// There are the following variables needed and must be decaled in program:
//	S_AUDIOSYNTHEX_DECODER g_sAudioSynthExDecoder[];
//	const UINT8 g_u8AudioSynExDecoderCount = (sizeof(g_sAudioSynthExDecoder)/sizeof(S_AUDIOSYNTHEX_DECODER));
//	S_AUDIOSYNTHEX_CALLBACK g_sAudioSynthExCallback[];
//	const UINT8 g_u8AudioSynExCallBackCount = (sizeof(g_sAudioSynthExCallback)/sizeof(S_AUDIOSYNTHEX_CALLBACK));
void AudioSynthEx_Configure(
	S_AUDIOSYNTHEX_DESC *psAudioSynthDesc,		// [Input] Audio's descriptor
	UINT32 u32ChannelID							// [Input] Channel ID;
);

void AudioSynthEx_GetRomHeadInfo(
	S_AUDIOSYNTHEX_DESC *psAudioSynthDesc,		// [Input] Audio's descriptor
	UINT8 u8StorageIndex
);

BOOL AudioSynthEx_GetAudioTrunkInfo(
	S_AUDIOSYNTHEX_DESC *psAudioSynthDesc,		// [Input] Audio's descriptor
	UINT32 u32AudioID,
	S_ROM_AUDIO_CHUNK_INFO *psAudioChunkInfo
);

BOOL AudioSynthEx_PlayAudio(			
	S_AUDIOSYNTHEX_DESC *psAudioSynthDesc,	// [Input] Audio's descriptor
	UINT32 u32AudioID						// [Input] Audio ID(or Play ID)
);

// Stat to play the audio represented by equation ID.
BOOL AudioSynthEx_PlayEquation(			
	S_AUDIOSYNTHEX_DESC *psAudioSynthDesc,	// [Input] Audio's descriptor
	UINT32 u32SentenceID					// [Input] Audio ID(or Play ID)
);

UINT16 AudioSynthEx_DecodeSample(
	S_AUDIOSYNTHEX_DESC *psAudioSynthDesc,
	PINT16 pi16DesAddr,
	UINT32 u32SampleCount
);

UINT16 AudioSynthEx_DecodeFrameGetData(			// Return
												//			TRUE: need to call AudioSynthEx_Decode() to prevent buffer empty
												//			TRUE: no need to call AudioSynthEx_Decode() to prevent buffer empty
	S_AUDIOSYNTHEX_DESC *psAudioSynthDesc,		// [Input] Audio's descriptor
	INT16 **ppi16PcmPtr			// [Output]PCM buffer pointer
);

BOOL AudioSynthEx_DecodeFrame(
	S_AUDIOSYNTHEX_DESC *psAudioSynthDesc		// [Input] Audio's descriptor
);

UINT16 AudioSynthEx_GetFrameData(			// Return
												//			TRUE: need to call AudioSynthEx_Decode() to prevent buffer empty
												//			TRUE: no need to call AudioSynthEx_Decode() to prevent buffer empty
	S_AUDIOSYNTHEX_DESC *psAudioSynthDesc,		// [Input] Audio's descriptor
	INT16 **ppi16PcmPtr							// [Output]PCM buffer pointer
);

#ifdef	__cplusplus
}
#endif


#endif
