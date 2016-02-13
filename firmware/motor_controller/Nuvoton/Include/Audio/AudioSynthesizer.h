/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.  *
 *                                                              *
 ****************************************************************/
#ifndef __AUDIOSYNTHESIZER_H__
#define __AUDIOSYNTHESIZER_H__

#include "Platform.h"
#include "System/SysInfra.h"
#include "Audio/NuOneDecode.h"
#include "Audio/NuSound.h"

#ifdef  __cplusplus
extern "C"
{
#endif

#define AUDIOSYN_MAJOR_NUM		3
#define AUDIOSYN_MINOR_NUM		10
#define AUDIOSYN_BUILD_NUM		002

#define AUDIOSYN_VERSION_NUM _SYSINFRA_VERSION(AUDIOSYN_MAJOR_NUM, AUDIOSYN_MINOR_NUM, AUDIOSYN_BUILD_NUM)

// E_AUDIOSYN_INCORRECTID			Audio or sentence index is incorrect
// E_AUDIOSYN_INVALIDADDR			Argument has invalid address
// E_AUDIOSYN_ENDOFPLAYBACK		Decoder stop because end of playback

// E_AUDIOSYN_UNKNOWNFMT			Decoder can't identify format
// E_AUDIOSYN_INCRRTFROMAT		Format is incorrect
// E_AUDIOSYN_INCRRTFRAME			Frame base decoder find incorrect frame type
// E_AUDIOSYN_ILLEGALACCESS		Access illegal address
#define E_AUDIOSYN_INCORRECTID		_SYSINFRA_ERRCODE(TRUE, MODULE_ID_AUDIOSYN, 1)  
#define E_AUDIOSYN_INVALIDADDR		_SYSINFRA_ERRCODE(TRUE, MODULE_ID_AUDIOSYN, 2) 
#define E_AUDIOSYN_ENDOFPLAYBACK	_SYSINFRA_ERRCODE(TRUE, MODULE_ID_AUDIOSYN, 3)  

#define E_AUDIOSYN_UNKNOWNFMT		_SYSINFRA_ERRCODE(TRUE, MODULE_ID_AUDIOSYN, 11)   
#define E_AUDIOSYN_INCRRTFROMAT		_SYSINFRA_ERRCODE(TRUE, MODULE_ID_AUDIOSYN, 12)  
#define E_AUDIOSYN_INCRRTFRAME		_SYSINFRA_ERRCODE(TRUE, MODULE_ID_AUDIOSYN, 13) 
#define E_AUDIOSYN_ILLEGALACCESS	_SYSINFRA_ERRCODE(TRUE, MODULE_ID_AUDIOSYN, 14)  


// DISCRIMINATE REGISTERD INDEX IS AUDIO OR SENTENCE
//	In NORMALMOD or SENTENCEMOD:
//		Audio synthesizer will preload encoded data into resource buffer.
//	In NORMALMOD_EX or SENTENCEMOD_EX:
//		Audio synthesizer won't preload encode data. application must call XXXXXXX to preload data.
//										
#define AUDIOSYN_NORMALMOD 	 	(0x0)	
#define AUDIOSYN_SENTENCEMOD 	(0x1)
#define AUDIOSYN_NORMALMOD_EX 	(0x2)	
#define AUDIOSYN_SENTENCEMOD_EX	(0x3)


#define AUDIOSYN_DESCRIPTORSIZE (0x44)

#define AUDIOSYN_NUOCHANNELHDRSIZE (0xC)
#define AUDIOSYN_NUOFRAMEBUFSIZE (AUDIOSYN_NUOCHANNELHDRSIZE + NUONEDECODE_FRAME_SAMPLE_SIZE)

#define AUDIOSYN_NUSCHANNELHDRSIZE (0x10)
#define AUDIOSYN_NUSFRAMEBUFSIZE (AUDIOSYN_NUSCHANNELHDRSIZE + DAC_BUF_LEN)

#define AUDIOSYN_NUSCHSTATICBUFSIZE (NUSOUND_CHANNEL_BUFFER_SIZE+FAAD_MIN_STREAMSIZE)

typedef UINT32 (*PFN_AUDIOSYN_GETDATA)(void * pDesAddr, UINT32 u32Position, UINT32 u32ByteNum);
typedef UINT32 (*PFN_AUDIOSYN_USREVENT)(UINT16 u16Index, UINT16 u16SubIndex);

//It is for backward compatible issue
#define AudioSyn_RegMDMDecoder	AudioSyn_RegMD4Decoder

///////////////////////////////////////////////////////////////////////////////////////////////////////
// Description:
// 	Configure audio synthesizer setting and initialize all resource.
// Arguments:
//  pfnGetResInfo[in]:	Call back function to get audio synthesizer resource information.
// Return:
//	Status of initialize audio synthesizer.
///////////////////////////////////////////////////////////////////////////////////////////////////////
UINT32
AudioSyn_Initialize(
	PFN_AUDIOSYN_GETDATA pfnGetResInfo
);

///////////////////////////////////////////////////////////////////////////////////////////////////////
// Description:
// 	Register P16 decoder into Audio Synthesizer. If not using this API to register P16 decoder,
//	Audio synthesizer will crash when data format is P16.
// Arguments:
//	None.
// Return:
//	None.
/////////////////////////////////////////////////////////////////////////////////////////////////////// 
void AudioSyn_RegP16Decoder(void);

///////////////////////////////////////////////////////////////////////////////////////////////////////
// Description:
// 	Register LP8 decoder into Audio Synthesizer. If not using this API to register LP8 decoder,
//	Audio synthesizer will crash when data format is LP8.
// Arguments:
//	None.
// Return:
//	None.
///////////////////////////////////////////////////////////////////////////////////////////////////////
void AudioSyn_RegLP8Decoder(void);

///////////////////////////////////////////////////////////////////////////////////////////////////////
// Description:
// 	Register MDM decoder into Audio Synthesizer. If not using this API to register MDM decoder,
//	Audio synthesizer will crash when data format is MDM.
// Arguments:
//	None.
// Return:
//	None.
/////////////////////////////////////////////////////////////////////////////////////////////////////// 
void AudioSyn_RegMD4Decoder(void);

///////////////////////////////////////////////////////////////////////////////////////////////////////
// Description:
// 	Register NuSound decoder into Audio Synthesizer. If not using this API to register NuSound decoder,
//	Audio synthesizer will crash when data format is NuSound.
// Arguments:
//	pu16FrameBuf[in]: Frame buffer address.
//	u16FrameBufSize[in]:	Frame buffer size.
//	pfnGetData_Callback[in]:	Get data callback function.
//	pfnUsrEvent_Callback[in]:	User event handler callback function.
//	pi8InternalBuf[in]:	buffer address for static and share memory. please refer NuSound definition.
//	u16IntBufSize[in]:	Internal used buffer size. please refer NuSound definition.
// Return:
//	None.
/////////////////////////////////////////////////////////////////////////////////////////////////////// 
void AudioSyn_RegNuSndDecoder(
 	PUINT16 pu16FrameBuf,
	UINT16 u16FrameBufSize,
	PFN_AUDIOSYN_GETDATA  pfnGetData_Callback,
 	PFN_AUDIOSYN_USREVENT pfnUsrEvent_Callback,
	PINT8 pi8InternalBuf,
	UINT16 u16IntBufSize
	);

///////////////////////////////////////////////////////////////////////////////////////////////////////
// Description:
// 	Register NuOne decoder into Audio Synthesizer. If not using this API to register NuOne decoder,
//	Audio synthesizer will crash when data format is NuOne.
// Arguments:
//	pu16FrameBuf[in]: Frame buffer address.
//	u16FrameBufSize[in]:	Frame buffer size.
//	pfnGetData_Callback[in]:	Get data callback function.
//	pfnUsrEvent_Callback[in]:	User event handler callback function.
//	pi8StaticBuf[in]:	Static buffer address. please refer NuOne definition.
//	pi8ShareBuf[in]:	Share buffer address. please refer NuOne definition.
// Return:
//	None.
///////////////////////////////////////////////////////////////////////////////////////////////////////
void AudioSyn_RegNuOneDecoder(
 	PUINT16 pu16FrameBuf,
	UINT16 u16FrameBufSize,
	PFN_AUDIOSYN_GETDATA  pfnGetData_Callback,
 	PFN_AUDIOSYN_USREVENT pfnUsrEvent_Callback,
	PINT8 pi8StaticBuf,
	UINT16 u16StaticBufSize,
	PINT8 pi8ShareBuf
	);

///////////////////////////////////////////////////////////////////////////////////////////////////////
// Description:
// 	Register NuLite decoder into Audio Synthesizer. If not using this API to register NuLite decoder,
//	Audio synthesizer will crash when data format is NuLite.
// Arguments:
//	pu16FrameBuf[in]: Frame buffer address.
//	u16FrameBufSize[in]:	Frame buffer size.
//	pfnGetData_Callback[in]:	Get data callback function.
//	pfnUsrEvent_Callback[in]:	User event handler callback function.
//	pi8StaticBuf[in]:	Static buffer address. please refer NuLite definition.
//	pi8ShareBuf[in]:	Share buffer address. please refer NuLite definition.
// Return:
//	None.
///////////////////////////////////////////////////////////////////////////////////////////////////////
void AudioSyn_RegNuLiteDecoder(
 	PUINT16 pu16FrameBuf,
	UINT16 u16FrameBufSize,
	PFN_AUDIOSYN_GETDATA  pfnGetData_Callback,
 	PFN_AUDIOSYN_USREVENT pfnUsrEvent_Callback,
	PINT8 pi8StaticBuf,
	UINT16 u16StaticBufSize,
	PINT8 pi8ShareBuf
);

///////////////////////////////////////////////////////////////////////////////////////////////////////
// Description:
// 	Register NuVox decoder into Audio Synthesizer. If not using this API to register NuVox decoder,
//	Audio synthesizer will crash when data format is NuVox.
// Arguments:
//	pu16FrameBuf[in]: Frame buffer address.
//	u16FrameBufSize[in]:	Frame buffer size.
//	pfnGetData_Callback[in]:	Get data callback function.
//	pfnUsrEvent_Callback[in]:	User event handler callback function.
//	pi8StaticBuf[in]:	Static buffer address. please refer NuVox definition.
//	pi8ShareBuf[in]:	Share buffer address. please refer NuVox definition.
// Return:
//	None.
///////////////////////////////////////////////////////////////////////////////////////////////////////
void AudioSyn_RegNuVoxDecoder(
 	PUINT16 pu16FrameBuf,
	UINT16 u16FrameBufSize,
	PFN_AUDIOSYN_GETDATA  pfnGetData_Callback,
 	PFN_AUDIOSYN_USREVENT pfnUsrEvent_Callback,
	PINT8 pi8StaticBuf,
	UINT16 u16StaticBufSize,
	PINT8 pi8ShareBuf
);

///////////////////////////////////////////////////////////////////////////////////////////////////////
// Description:
// 	Register NuVox decoder into Audio Synthesizer. If not using this API to register NuVox decoder,
//	Audio synthesizer will crash when data format is NuVox53.
// Arguments:
//	pu16FrameBuf[in]: Frame buffer address.
//	u16FrameBufSize[in]:	Frame buffer size.
//	pfnGetData_Callback[in]:	Get data callback function.
//	pfnUsrEvent_Callback[in]:	User event handler callback function.
//	pi8StaticBuf[in]:	Static buffer address. please refer NuVox definition.
//	pi8ShareBuf[in]:	Share buffer address. please refer NuVox definition.
// Return:
//	None.
///////////////////////////////////////////////////////////////////////////////////////////////////////
void AudioSyn_RegNuVox53Decoder(
 	PUINT16 pu16FrameBuf,
	UINT16 u16FrameBufSize,
	PFN_AUDIOSYN_GETDATA  pfnGetData_Callback,
 	PFN_AUDIOSYN_USREVENT pfnUsrEvent_Callback,
	PINT8 pi8StaticBuf,
	UINT16 u16StaticBufSize,
	PINT8 pi8ShareBuf
);

///////////////////////////////////////////////////////////////////////////////////////////////////////
// Description:
// 	Register Midi decoder into Audio Synthesizer. If not using this API to register Midi decoder,
//	Audio synthesizer will crash when data type is Midi.
// Arguments:
//	u32InternalBufAddr[in]: Internal used buffer address, Application can use MidiSyn_NeedVariableMemSize
//							to get current buffer size.
//	u32OutputSampleRate[in]: Output sample rate.
//	pfnGetData_Callback[in]:	Get data callback function.
//	pfnUsrEvent_Callback[in]:	User event handler callback function.
//	u32WTBAddress[in]:	Wav table address, when decoded midi file, it must refer to it to get instruments/drums
//						data.
// Return:
//	None.
///////////////////////////////////////////////////////////////////////////////////////////////////////
void AudioSyn_RegMidiDecoder(
 	UINT32 u32InternalBufAddr,
	UINT32 u32OutputSampleRate,
	PFN_AUDIOSYN_GETDATA  pfnGetData_Callback,
 	PFN_AUDIOSYN_USREVENT pfnUsrEvent_Callback,
	UINT32 u32WTBAddress
	);

///////////////////////////////////////////////////////////////////////////////////////////////////////
// Description:
// 	To parse audio data or equation information which is generated by audio 
//	tool to get audio files's information then fill them into data descriptor.
// Arguments:
//  u8IndexInfo[in]:	Audio data index, MUST use this index to get data position.
//  eTypeMode[in]:	Decide to play audio file or equation.
// 	pDscptrAddr[in]:	Internal used, library will parse data header to fill then into this memory.
//	pfnGetDataCallback[in]:	Get data callback function, it will be attach into pDscptrAddr.
//	pfnUsrEventHdlr[in]:	User evnet callback function, it will be called as decoder find user event.
//	puResInBuf[in]:	Store encode data during decoding, it will be attach into pDscptrAddr.
//	u16InBufSize[in]: Size of puResInBuf, it will be attach into pDscptrAddr. 
//	u16ChannelIndex[in]: Channel index for NuO, NuS and Midi; P16/LP8/MD4 are depend by pDscptrAddr.
// Return:
//	To initialize data descriptor success or failed.
///////////////////////////////////////////////////////////////////////////////////////////////////////
UINT32
AudioSyn_InitDataDscrptr(
	UINT8 u8IndexInfo,
	UINT8 u8TypeMode,
	void *pDscptrAddr,
	PFN_AUDIOSYN_GETDATA pfnGetDataCallback,
	PFN_AUDIOSYN_USREVENT pfnUsrEventCallback,
	PUINT8 puResInBuf,
	UINT16 u16InBufSize,
	UINT16 u16ChannelIndex
);	

///////////////////////////////////////////////////////////////////////////////////////////////////////
// Description:
// 	Decide decoder to de-compress data, and handle others information in here
//	during decoding.
// Arguments:
//  psDataDsp[in]:	Internal used, library will parse data header to fill then into this memory.
//  pu16DesAddr[in]:	Memory address keeps decoded data.
// 	u32SampleNum[in]:	Wanted sample number.
//	pu32DecSmpCnt[out]: How many samples are decoded by this round. 
///////////////////////////////////////////////////////////////////////////////////////////////////////
UINT32
AudioSyn_DecodeData(
	void *pDscptrAddr,
	PUINT16 pu16DesAddr,
	UINT32 u32SampleNum,
	PUINT32 pu32DecSmpCnt
);

///////////////////////////////////////////////////////////////////////////////////////////////////////
// Description:
// 	Get data sample rate from data descriptor.
// Arguments:
//  pDscptrAddr[in]: Data descriptor address.
//	pu16SampleRate[in]: sample rate after decoded data.
///////////////////////////////////////////////////////////////////////////////////////////////////////
UINT32
AudioSyn_GetDataSampleRate(
	void *pDscptrAddr,
	PUINT16 pu16SampleRate
);

///////////////////////////////////////////////////////////////////////////////////////////////////////
// Description:
// 	Get data current address.
// Arguments:
//  pDscptrAddr[in]:	Data descriptor which keeps all information during decoding.
// Return:
//	Data current address.
///////////////////////////////////////////////////////////////////////////////////////////////////////
UINT32
AudioSyn_GetDataAddress(
	void *pDscptrAddr
);

///////////////////////////////////////////////////////////////////////////////////////////////////////
// Description:
// 	Get data length.
// Arguments:
//  pDscptrAddr[in]:	Data descriptor which keeps all information during decoding.
// Return:
//	Data data length.
/////////////////////////////////////////////////////////////////////////////////////////////////////// 
UINT32
AudioSyn_GetDataLength(
	void *pDscptrAddr
);

///////////////////////////////////////////////////////////////////////////////////////////////////////
// Description:
// 	Preload data into resource buffer, this API only be called after AudioSyn_InitDataDscrptr,
//	and it's mode must be AUDIOSYN_NORMALMOD_EX or AUDIOSYN_SENTENCEMOD_EX.
// Arguments:
//  pDscptrAddr[in]:	Data descriptor which keeps all information during decoding.
// Return:
//	Data data length.
/////////////////////////////////////////////////////////////////////////////////////////////////////// 
void AudioSyn_PreloadData(
	void *pDscptrAddr
);

///////////////////////////////////////////////////////////////////////////////////////////////////////
// Description:
// 	Version control API.
// Arguments:
//  None.
// Return:
//	Version number.
///////////////////////////////////////////////////////////////////////////////////////////////////////
UINT32
AudioSyn_GetVersion(void);

#ifdef  __cplusplus
}
#endif

#endif	//__AUDIOSYNTHESIZER_H__

