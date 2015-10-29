/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c)Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/

#ifndef __IRCTRL_H__
#define __IRCTRL_H__

#include "Platform.h"
#include "System/SysInfra.h"

//======================================Define IRCtrl Version ==============================================
#define IRCTRL_MAJOR_NUM	2
#define IRCTRL_MINOR_NUM	00
#define IRCTRL_BUILD_NUM	001
#define IRCTRL_VERSION_NUM	_SYSINFRA_VERSION(IRCTRL_MAJOR_NUM, IRCTRL_MINOR_NUM, IRCTRL_BUILD_NUM)
// ------------------------------------------------------------------------------
// Define the Error Code
// ------------------------------------------------------------------------------
// E_IRCTRL_CARRYPIN_ERR			GPIOB pin13 multi-function did not be enabled.
// E_IRCTRL_TIMEROPEN_ERR			Timer1 is not idle.
// E_IRCTRL_TXSTATUS_ERR			IR controller is transmitting.
// E_IRCTRL_RXPACKET_ERR			Head of packet is not correct.
// E_IRCTRL_RXPACKET_NOTCOMPLETE	Received data is not complete.
#define E_IRCTRL_CARRYPIN_ERR			_SYSINFRA_ERRCODE(TRUE, MODULE_ID_IRCTRL, 1)
#define E_IRCTRL_TIMEROPEN_ERR			_SYSINFRA_ERRCODE(TRUE, MODULE_ID_IRCTRL, 2)
#define E_IRCTRL_TXSTATUS_ERR			_SYSINFRA_ERRCODE(TRUE, MODULE_ID_IRCTRL, 3)
#define E_IRCTRL_RXPACKET_ERR			_SYSINFRA_ERRCODE(TRUE, MODULE_ID_IRCTRL, 4)
#define E_IRCTRL_RXPACKET_NOTCOMPLETE	_SYSINFRA_ERRCODE(TRUE, MODULE_ID_IRCTRL, 5)

// ------------------------------------------------------------------------------
// Macro for IR state getting
// ------------------------------------------------------------------------------
extern volatile UINT8			g_u8IRCtrlState;	
#define IRCTRL_GET_STATE		g_u8IRCtrlState
 
// ------------------------------------------------------------------------------
// Define state
// ------------------------------------------------------------------------------
#define IRCTRL_IDLE_STATE			0
#define IRCTRL_COMPLETED_STATE		1
#define IRCTRL_READY_STATE			2
#define IRCTRL_BUSY_STATE 			3

// ------------------------------------------------------------------------------
// Define transistor switch type
// ------------------------------------------------------------------------------
#define IRCTRL_NPN_SWITCH			0 
#define IRCTRL_PNP_SWITCH			1 
//======================================API prototype================================================
//==========================TX================================
//---------------------------------------------------------------------------------------------------------
// Function: 	IRCtrl_InitTX
// Parameter:
//				u32CarryFeq			[in]		IR LED carry frequency; 
//				u8FlashCount		[in]		Carry conut of IR when '1' send;	
//				u8SwitchType		[in]		Select switch type for IR TX controller; // pnp :1 ; npn: 0
//
// Return: 
//				E_IRCTRL_CARRYPIN_ERR			GPIOB pin13 multi-function did not be enabled.
//				E_IRCTRL_TIMEROPEN_ERR			Timer1 is not idle.
// Description: 	
//				Initialize IR contrller carry frequency for LED . User must enable GPIOB13 for IR carry
//              output pin before this API. Timer1 is IR carry counter, user need to close timer1 application
//				before this API.
//---------------------------------------------------------------------------------------------------------
ERRCODE
IRCtrl_InitTX(
	UINT32	u32CarryFeq,
	UINT8	u8FlashCount,
	UINT8	u8SwitchType						
);

//---------------------------------------------------------------------------------------------------------
// Function: 	IRCtrl_PackData
// Parameter:
//				u16Data				[in]		Transmissive data.For example: ID device+command+reverse.
//
// Return:		
//				E_IRCTRL_TXSTATUS_ERR			IR controller is transmitting(busy).				
// Description: 	
//				Pack data + head into a packet. Data length is less than 16 bits. This API will enable timer1
//				to toggle carry output after packing. In order to avoid noise cause head check pass, u16Data
//				must greater than zero. So, user need to add coding scheme(u16Data>0) if send "zero" command.
//---------------------------------------------------------------------------------------------------------
ERRCODE
IRCtrl_PackData(
	UINT16	u16Data
);

//---------------------------------------------------------------------------------------------------------
// Function: 	IRCtrl_SendPacket
// Parameter:
// Return:				
//				
// Description: 	
//				This API will control carry output via GPIOB pin13. User must call it periodically in timer1
//				interrupt handler. 			
//---------------------------------------------------------------------------------------------------------
void
IRCtrl_SendPacket(void);

//---------------------------------------------------------------------------------------------------------
// Function: 	IRCtrl_UnInitTX
// Parameter:
// Return:				
//				
// Description: 	
//				This API will close timer1 IP and APB clock for timer1.  			
//---------------------------------------------------------------------------------------------------------
void
IRCtrl_UnInitTX(void);

//==========================RX================================
//---------------------------------------------------------------------------------------------------------
// Function: 	IRCtrl_InitRX
// Parameter:
//				u32CarryFeq			[in]		IR LED carry frequency;
//				u8FlashCount		[in]		Carry conut of IR when '1' send;
//				
//				pGPIO				[in]		GPIO port for receiver module output.
//				u16Pins				[in]
// Return:				
//				sample rate of RX. If return value is zero, i16FeqFineTuning is unreasonable. 
// Description: 	
//				This API will initialize timer sampling rate and receiver module output port/pin. User must
//				synchronize u32CarryFeq and u8FlashCount with TX.  			
//---------------------------------------------------------------------------------------------------------
UINT32
IRCtrl_InitRX(
	UINT32	u32CarryFeq,
	UINT8	u8FlashCount,
	GPIO_T* pGPIO,
	UINT16  u16Pins
);						

//---------------------------------------------------------------------------------------------------------
// Function: 	IRCtrl_UnPackData
// Parameter:
//				pu16DataBuf			[out]			Save complete received data.
// Return:				
//				E_IRCTRL_RXPACKET_NOTCOMPLETE 		Received data is not complete.
//				E_IRCTRL_RXPACKET_ERR            	packet is not correct.
// Description: 	
//				This API will unpack TX transmission packet.
//---------------------------------------------------------------------------------------------------------
ERRCODE
IRCtrl_UnPackData(
	PUINT16	pu16DataBuf
);

//---------------------------------------------------------------------------------------------------------
// Function: 	IRCtrl_ReceivePacket
// Parameter:			
// Return:				
//				E_IRCTRL_RXPACKET_ERR				packet is not correct.
// Description: 	
//				This API will get TX transmission packet. It will be invoked in timer interrupt handler to 
//				sample receiver module output waveform.
//---------------------------------------------------------------------------------------------------------
ERRCODE
IRCtrl_ReceivePacket(void);

//---------------------------------------------------------------------------------------------------------
// Function: 	IRCtrl_ReceiverSync
// Parameter:			
// Return:				
//
// Description: 	
//				This API will sync first data. It will be invoked in GPIO interrupt handler to ensure data
//				synchronization.
//---------------------------------------------------------------------------------------------------------
void
IRCtrl_ReceiverSync(void);

//---------------------------------------------------------------------------------------------------------
// Function: 	IRCtrl_GetVersion
// Parameter:			
// Return:				
//
// Description: 	
//				Get IR controller lib version.
//---------------------------------------------------------------------------------------------------------
UINT32
IRCtrl_GetVersion(void);

#endif //__IRCTRL_H__
