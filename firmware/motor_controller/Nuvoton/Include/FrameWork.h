/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) Nuvoton Technology Corp. All rights reserved.                                              */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#ifndef _FRAMEWORK_H_
#define _FRAMEWORK_H_

#include "Platform.h"

#ifdef __N572__
#include "Audio/NuADCFilterEx.h"
#include "Audio/NuDACFilterEx.h"
#endif


// ----------------------------------------------------------------------------------------------
// Key configurations
#if ((CONFIG_MATRIX_KEY>0) || (CONFIG_INPUT_KEY>0))
	#include "Commu\Keypad.h"
	extern void InputKey_Initiate(void);
	#define KEYPAD_INITIATE()					InputKey_Initiate()
#else
	#define KEYPAD_INITIATE() 
#endif


#if (CONFIG_MATRIX_KEY>0)
	#define KEYPAD_CHECK_MATRIX() 				Keypad_ScanKey()
	#define KEYPAD_DEBOUCE_MATRIX()				Keypad_TgrDecDebounceCounter()
#else
	#define KEYPAD_CHECK_MATRIX()
	#define KEYPAD_DEBOUCE_MATRIX()
#endif
#if (CONFIG_INPUT_KEY>0)
	#define KEYPAD_CHECK_INPUT() 			   	Keypad_ScanTgr()
	#define KEYPAD_DEBOUCE_KEY()				Keypad_KeyDecDebounceCounter()
#else
	#define KEYPAD_CHECK_INPUT()
	#define KEYPAD_DEBOUCE_KEY()
#endif
// ----------------------------------------------------------------------------------------------


// ----------------------------------------------------------------------------------------------
// Output pin configurations
#if (CONFIG_OUTPUT_PIN>0)
	extern void OutputPin_Initiate(void);	   
	#define OUTPUTPIN_INITIATE() 				OutputPin_Initiate()
#else
	#define OUTPUTPIN_INITIATE()
#endif
// ----------------------------------------------------------------------------------------------

// ----------------------------------------------------------------------------------------------
// Input state configurations
#if (CONFIG_INPUT_STATE > 0 )
	#include "InState.h"
#endif	
// ----------------------------------------------------------------------------------------------


// ----------------------------------------------------------------------------------------------
// Output state configurations
#if (CONFIG_OUTPUT_STATE > 0 )
	#include "OutState.h"
#endif
// ----------------------------------------------------------------------------------------------


// ----------------------------------------------------------------------------------------------
// UltraIO configurations
#if ( ULTRAIO_FW_CURVE_ENABLE || ULTRAIO_HW_CURVE_ENABLE )
	extern void UltraIO_Initiate(void);
	#define ULTRAIO_INITIATE() UltraIO_Initiate() 
#else
	#define ULTRAIO_INITIATE() 		
#endif
 
#if ( ULTRAIO_FW_CURVE_ENABLE ) 
	extern void UltraIO_OuputCurve(void);
	#define ULTRAIO_OUTCURVE()		UltraIO_OuputCurve()
#else 
	#define ULTRAIO_OUTCURVE()
#endif

#if ( ULTRAIO_FW_CURVE_ENABLE || ULTRAIO_HW_CURVE_ENABLE ) 
	extern void UltraIO_EventHandler(UINT32 u32CmdIndex, UINT32 u32SubIndex);
	#define ULTRAIO_EVENTHANDLER(u32CmdIndex, u32SubIndex)		UltraIO_EventHandler(u32CmdIndex, u32SubIndex)
#else 
	#define ULTRAIO_EVENTHANDLER(u32CmdIndex, u32SubIndex)
#endif
// ----------------------------------------------------------------------------------------------


// ----------------------------------------------------------------------------------------------
// Storage configurations
#if (CONFIG_STORAGE>0)
	extern void Storage_Initiate(void);
	#define STORAGE_INITIATE()		Storage_Initiate()
#else
	#define STORAGE_INITIATE()
#endif
// ----------------------------------------------------------------------------------------------


// ----------------------------------------------------------------------------------------------
// Audio configurations
#if (CONFIG_AUDIO_CHANNEL>0)
	#include "Audio.h"
	extern void Audio_Initiate(void);
	#define AUDIO_INITIATE()		Audio_Initiate()
#else
	#define AUDIO_INITIATE()
#endif 
// ----------------------------------------------------------------------------------------------

// ----------------------------------------------------------------------------------------------
// Multi-function pin configurations
#if ((CONFIG_PORTA_PINS>0) || (USER_PORTA_PINS>0))
	#define MULTI_FUNC_PINS_INITIATE_PORTA()	DrvSYS_EnableMultifunctionGpioa(CONFIG_PORTA_PINS|USER_PORTA_PINS)
#else
	#define MULTI_FUNC_PINS_INITIATE_PORTA()
#endif

#if ((CONFIG_PORTB_PINS>0) || (USER_PORTB_PINS>0))
	#define MULTI_FUNC_PINS_INITIATE_PORTB()	DrvSYS_EnableMultifunctionGpioa(CONFIG_PORTA_PINS|USER_PORTB_PINS)
#else
	#define MULTI_FUNC_PINS_INITIATE_PORTB()
#endif	   
// ----------------------------------------------------------------------------------------------


//-----------------------------------------------------------------------------------------------
// PLL or RC clock configurations
#if (CONFIG_CLK_RC > 0)
	extern void SysClk_ConfigRC(void);
	#define SYSCLK_INITIATE()		SysClk_ConfigRC()
#elif (CONFIG_CLK_PLL2_RC>0)
	extern void  SysClk_ConfigPll2_RC(void);
	#define SYSCLK_INITIATE()		SysClk_ConfigPll2_RC()
#elif (CONFIG_CLK_PLL2_32K>0)
	extern 	void SysClk_ConfigPll2_32K(void);
	#define SYSCLK_INITIATE()		SysClk_ConfigPll2_32K()
#elif ((CONFIG_CLK_PLL2_6M>0) || (CONFIG_CLK_PLL2_12M>0))
	extern void SysClk_ConfigPll2_6M12M(void);
	#define SYSCLK_INITIATE()		SysClk_ConfigPll2_6M12M()
#else
	#define SYSCLK_INITIATE()
#endif 
//-----------------------------------------------------------------------------------------------


//-----------------------------------------------------------------------------------------------
// APU(Speaker) configurations
#if ( CONFIG_APU_CH0_UP_SAMPLING==2 )
	extern UINT8 g_au8Ch0UpSamplingWorkBuf[] ;
	#define UPSAMPLING_INITIATE()	NuDACFilterEx_Up2Initial(g_au8Ch0UpSamplingWorkBuf)
#else
	#define UPSAMPLING_INITIATE()
#endif
//-----------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------------------------
// ADC(MIC) configurations
#if ( CONFIG_ADC_DOWN_SAMPLING ==2 )
	extern UINT8 g_au8DownSamplingWorkBuf[];
	#define DOWNSAMPLING_INITIATE()	NuADCFilterEx_Down2Initial(g_au8DownSamplingWorkBuf)
#else
	#define DOWNSAMPLING_INITIATE()
#endif
//-----------------------------------------------------------------------------------------------

#endif //_FRAMEWORK_H_
