/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.  *
 *                                                              *
 ****************************************************************/
#ifndef __UAC_H__
#define __UAC_H__

#include "Platform.h"
#include "Driver/DrvUSB.h"

#ifdef  __cplusplus
extern "C"
{
#endif

#define UAC_MAJOR_NUM	1
#define UAC_MINOR_NUM	0
#define UAC_BUILD_NUM	1

#define UAC_MICROPHONE  0
#define UAC_SPEAKER		1
#define UAC_MIDI		2


// Define module version number.
/*
 	_SYSINFRA_VERSION_DEF(UAC, UAC_MAJOR_NUM, UAC_MINOR_NUM, UAC_BUILD_NUM)
	*/
#define UAC_VERSION_NUM				_SYSINFRA_VERSION(UAC_MAJOR_NUM, UAC_MINOR_NUM, UAC_BUILD_NUM) 

// E_UAC_SAMPLE_NOT_SUPPORT              Do not support the sample rate
// E_UAC_BUFFER_OVERRUN           Received data is more than buffer size
// E_UAC_CTRL_REG_TAB_FULL        Control register table is full
// E_UAC_INVALID_EP_NUM           EndPoint number error
// E_UAC_NULL_POINTER             Callback function is NULL
// E_UAC_UNDEFINE                 Undefined error

/*
	_SYSINFRA_ERRCODE_DEF(UAC, UNDEFINE, TRUE, MODULE_ID_UAC, 0)
	_SYSINFRA_ERRCODE_DEF(UAC, NULL_POINTER, TRUE, MODULE_ID_UAC, 1)
	_SYSINFRA_ERRCODE_DEF(UAC, BUFFER_OVERRUN, TRUE, MODULE_ID_UAC, 2)
	_SYSINFRA_ERRCODE_DEF(UAC, INVALID_EP_NUM, TRUE, MODULE_ID_UAC, 3)
	_SYSINFRA_ERRCODE_DEF(UAC, CTRL_REG_TAB_FULL, TRUE, MODULE_ID_UAC, 4)
	_SYSINFRA_ERRCODE_DEF(UAC, SAMPLE_NOT_SUPPORT, TRUE, MODULE_ID_UAC, 5)
	*/
#define E_UAC_UNDEFINE				_SYSINFRA_ERRCODE(TRUE, MODULE_ID_UAC, 1)
#define E_UAC_NULL_POINTER			_SYSINFRA_ERRCODE(TRUE, MODULE_ID_UAC, 2)
#define E_UAC_BUFFER_OVERRUN   		_SYSINFRA_ERRCODE(TRUE, MODULE_ID_UAC, 3)
#define E_UAC_INVALID_EP_NUM		_SYSINFRA_ERRCODE(TRUE, MODULE_ID_UAC, 4)
#define E_UAC_CTRL_REG_TAB_FULL		_SYSINFRA_ERRCODE(TRUE, MODULE_ID_UAC, 5)
#define E_UAC_SAMPLE_NOT_SUPPORT	_SYSINFRA_ERRCODE(TRUE, MODULE_ID_UAC, 6)

	
#define UAC_RECORDER_MUTE  			0x01
#define UAC_RECORDER_VOLUME			0x02
#define UAC_SPEAKER_MUTE			0x11
#define UAC_LEFT_SPEAKER_VOLUME		0x12
#define UAC_RIGHT_SPEAKER_VOLUME	0x13
#define UAC_MIDI_MUTE				0x21
#define UAC_MIDI_VOLUME				0x22

#define UAC_SET_SAMPLERATE			0xA1
#define UAC_RECORDER_SET_SAMPLERATE 0xA2
#define UAC_SPEAKER_SET_SAMPLERATE	0xA3


#define UAC_MUTE_ADDR			0
#define UAC_LEFT_VOL_ADDR		1
#define UAC_RIGHT_VOL_ADDR		2
#define UAC_MIN_VOL_ADDR		3
#define UAC_MAX_VOL_ADDR		4
#define UAC_RES_VOL_ADDR		5

//***************************************************
// 		Define UAC_Sys.c fucntion
//***************************************************
typedef void
(*PFN_UAC_AUDIO_ENABLE)(UINT8 u8Object, UINT8 u8State);

typedef void
(*PFN_UAC_AUDIO_DISABLE)(UINT8 u8Object);

typedef void								
(*PFN_UAC_AUDIO_SEND_FRAME)(void);

typedef void
(*PFN_UAC_AUDIO_RECEIVE_FRAME)(INT16 *pi16src);

typedef void
(*PFN_UAC_AUDIO_CONTROL_CALLBACK)(INT16 i16ImageID, INT32 i32SetVal);

	UINT32
	UAC_GetVersion(void);

	ERRCODE
	UAC_Open(
    	UINT8 *au8DeviceDescriptor,   
    	UINT8 *au8ConfigDescriptor,
    	UINT8 *au8VendorStringDesc,
    	UINT8 *au8ProductStringDesc,
    	UINT8 *au8StringSerial,
    	UINT8 u8IsoOrBulkOutSize,
    	BOOL  bMIDIFunction
	);

	void UAC_Close(void);


	void UAC_SendOneAudioPacket(
	   BYTE *pbyAudioData, 
	   UINT8 u8AudioSize
	);

       void UAC_AudioCallBackFuncInit(
		PFN_UAC_AUDIO_ENABLE		pfnAudioEnable,
		PFN_UAC_AUDIO_DISABLE		pfnAudioDisable,
		PFN_UAC_AUDIO_SEND_FRAME	pfnSendAudioFrame,
		PFN_UAC_AUDIO_RECEIVE_FRAME	pfnReceiveAudioFrame,
		PFN_UAC_AUDIO_CONTROL_CALLBACK pfnAudioControlCallBack	
       );
    
   void UAC_memcpy_Align4(
		void *dest, 
		void *src, 
		int32_t size
       );

     void  UAC_SetAudioAttribute(
	 	UINT16 u16Audio, 
		INT32 *pi32AudioAttr
		);

#ifdef  __cplusplus
}
#endif

#endif // __UAC_H__




































