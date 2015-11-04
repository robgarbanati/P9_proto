/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/

#ifndef __KEYPAD_H__
#define __KEYPAD_H__


#include "platform.h"
#include "System/SysInfra.h"


//======================================Define Keypad Version ==============================================
#define KEYPAD_MAJOR_NUM	1
#define KEYPAD_MINOR_NUM	00
#define KEYPAD_BUILD_NUM	001
#define KEYPAD_VERSION_NUM	_SYSINFRA_VERSION(KEYPAD_MAJOR_NUM, KEYPAD_MINOR_NUM, KEYPAD_BUILD_NUM)

#define KEYPAD_GPIOA	0
#define KEYPAD_GPIOB	1

#define KEYPAD_FALLING	0
#define KEYPAD_RISING	1
#define KEYPAD_PRESSING 2

// ------------------------------------------------------------------------------
// Define the Error Code
// ------------------------------------------------------------------------------
// E_KEYPAD_TGR_HANDLER_IS_NOT_FOUND	Trigger handler is not found
// E_KEYPAD_KEY_HANDLER_IS_NOT_FOUND	Key handler is not found
/////////////////////////////////////////////////////////////////////////////////

#define E_KEYPAD_TGR_HANDLER_IS_NOT_FOUND		_SYSINFRA_ERRCODE(TRUE, MODULE_ID_KEYPAD, 1)  
#define E_KEYPAD_KEY_HANDLER_IS_NOT_FOUND		_SYSINFRA_ERRCODE(TRUE, MODULE_ID_KEYPAD, 2)  


//===================================== typedef ============================================================

typedef void (*PFN_KEYPAD_CALLBACK)(UINT32 u32Param);

// Direct trigger event handler
typedef struct 
{
	PFN_KEYPAD_CALLBACK	m_pfnTgr_Callback;		// callback function of event handler. NULL: end of handler array
	UINT32	m_u32Param;							// parameter that will be passed to handler
	UINT16	m_u16Pin;							// Pin bit mask	direct trigger input
	UINT8	m_u8Port;							// GPIO Port. 0: GPIOA, 1: GPIOB
	UINT8	m_u8Mode;							// 0: falling, 1: rising, 2: pressing
} S_KEYPAD_TGR_HANDLER;

// Key matrix event handler
typedef struct 
{
	PFN_KEYPAD_CALLBACK	m_pfnKey_Callback;		// callback function of event handler. NULL: end of handler array
	UINT32	m_u32Param;							// parameter that will be passed to handler
	UINT16	m_u16Pin_I;							// Pin bit mask of key matrix input
	UINT16	m_u16Pin_O;							// Pin bit mask of key matrix output
	UINT8	m_u8Port_I;							// input GPIO port. 0: GPIOA, 1: GPIOB
	UINT8	m_u8Port_O;							// Output GPIO port. 0: GPIOA, 1: GPIOB
	UINT8	m_u8Mode;							// 0: falling, 1: rising, 2: pressing
//	UINT8	m_u8Reserve;
} S_KEYPAD_KEY_HANDLER;


//========================================== API prototype ================================================

//---------------------------------------------------------------------------------------------------------
// Function: 	Keypad_InitKeypad
// Parameter:
//				u16DebounceCount		[in]		Debounce count for changing of pin status. 
//				u16PressingCount		[in]		Interval count between 2 pressing events. Pressing 
//														event handler will be called periodically.
// Description: 	
//				Initiate keypad. Set Debounce and pressing events interval count. Time unit is decided by
//				application. Application should initiate a timer to call Keypad_KeyDecDebounceCounter() 
//				or Keypad_TgrDecDebounceCounter() periodically.
//---------------------------------------------------------------------------------------------------------
void
Keypad_InitKeypad(
	UINT16 u16DebounceCount,
	UINT16 u16PressingCount
);

//---------------------------------------------------------------------------------------------------------
// Function: 	Keypad_InitTgr
// Parameter:
//				u16PinMaskA_I		[in]	GPIOA to be input pins mask.
//				u16PinMaskB_I		[in]	GPIOB to be input pins mask.
//				psTgrHandler		[in]	direct trigger event handler array.
// Description: 	
//				Initiate direct trigger pins and event handlers. The event handlers will be 
//				called as Falling, Rising, and Pressing events happen.
//				Application can do operations according to the arguments (set in event handler array) 
//				of event handler. This fucntion gets initial pins value for next scanning the change.
//---------------------------------------------------------------------------------------------------------
void
Keypad_InitTgr(
	UINT16 u16PinMaskA_I,
	UINT16 u16PinMaskB_I,
	const S_KEYPAD_TGR_HANDLER* psTgrHandler
);

//---------------------------------------------------------------------------------------------------------
// Function: Keypad_ScanTgr
// return:			0: trigger event does not happen or event handler is called,
//					(E_KEYPAD_TGR_HANDLER_IS_NOT_FOUND): event happen but handler is not found.
// Description: 	
//					This function scan GPIOA or GPIOB pins for direct trigger operation. It calls event 
//					handler if Debounce is timer up.
// Note: 			
//					This API can be called anywhere to scan trigger status, and should be called timely
//					to prevent status loss. But should be careful to call it in ISR to prevent
//					block other ISR if event handler will take too much CPU time.
//---------------------------------------------------------------------------------------------------------
ERRCODE
Keypad_ScanTgr(void);

//---------------------------------------------------------------------------------------------------------
// Function: 	Keypad_TgrDecDebounceCounter
// return :		true: decrease Debounce counter (in Debounce), 
//				false: not decrease Debounce counter (not in Debounce)
// Description: 	
//				This API is called in timer interrupt to decrease Debounce counter periodically.
//				The debunce counter may dedunce falling/rising status change or pressing interval.
//---------------------------------------------------------------------------------------------------------
BOOL
Keypad_TgrDecDebounceCounter(void);

//---------------------------------------------------------------------------------------------------------
// Function: 	Keypad_InitKey
// Parameter:
//				u16PinMaskA_I		[in]	GPIOA to be input pins mask of key matrix
//				u16PinMaskB_I		[in]	GPIOB to be input pins mask of key matrix
//				u16PinMaskA_O		[in]	GPIOA to be output pins mask of key matrix
//				u16PinMaskB_O		[in]	GPIOB to be output pins mask of key matrix
//				psKeyHandler		[in]	key matrix event handler array.
//				pu16KeyPinValueBuf	[in]	application provides an array to be pin value buffer. 
//											Array size is equalt to ""output pin number * input port number". 
//											This buffer must "exist" as keypad scanning.
// Description: 	
//				Initial key matix pins and event handlers.
//---------------------------------------------------------------------------------------------------------
void
Keypad_InitKey(
	UINT16 u16PinMaskA_I,
	UINT16 u16PinMaskB_I,
	UINT16 u16PinMaskA_O,
	UINT16 u16PinMaskB_O,
	const S_KEYPAD_KEY_HANDLER* psKeyHandler,
	UINT16* pu16KeyPinValueBuf
);

//---------------------------------------------------------------------------------------------------------
// Function: Keypad_ScanKey
// return:			0: key event does not happen or event handler is called,
//					(E_KEYPAD_KEY_HANDLER_IS_NOT_FOUND): event happen but handler is not found.
// Description: 	
//					Scan key matrix, and call event handler as Debounce time up.
// Note: 			
//					This API can be called anywhere to scan key matrix status, and should be called timely
//					to prevent status loss. But should be careful to call it in interrupt to prevent
//					block other ISR if event handler will take too much CPU time.
//---------------------------------------------------------------------------------------------------------
ERRCODE
Keypad_ScanKey(void);

//---------------------------------------------------------------------------------------------------------
// Function: 	Keypad_KeyDecDebounceCounter
// return :		true: decrease Debounce counter (in Debounce), 
//				false: not decrease Debounce counter (not in Debounce)
// Description: 	
//				This API is called in timer interrupt to decrease Debounce counter periodically.
//				The debunce counter may dedunce falling/rising status change or pressing interval.
//---------------------------------------------------------------------------------------------------------
BOOL
Keypad_KeyDecDebounceCounter(void);


//---------------------------------------------------------------------------------------------------------
// Get keypad library version
//---------------------------------------------------------------------------------------------------------
UINT32
Keypad_GetVersion(void);


#endif
