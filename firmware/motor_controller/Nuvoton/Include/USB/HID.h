/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.  *
 *                                                              *
 ****************************************************************/
#ifndef __HID_H__
#define __HID_H__

#include "Platform.h"
#include "System/SysInfra.h"

#include "Driver/DrvUSB.h"

#ifdef  __cplusplus
extern "C"
{
#endif

#define HID_MAJOR_NUM	1
#define HID_MINOR_NUM	0
#define HID_BUILD_NUM	1

//***************************************************
// 		HID Class REQUEST
//***************************************************
#define GET_REPORT          0x01
#define GET_IDLE            0x02
#define GET_PROTOCOL        0x03
#define SET_REPORT          0x09
#define SET_IDLE            0x0A
#define SET_PROTOCOL        0x0B

#define HID_RPT_TYPE_INPUT		0x01
#define HID_RPT_TYPE_OUTPUT		0x02
#define HID_RPT_TYPE_FEATURE	0x03

  
  // E_HID_BUFFER_OVERRUN                 Allocated buffer is not enough.
  // E_HID_CTRL_REG_TAB_FULL              Control register table is full.
  // E_HID_EXCEED_INT_IN_PACKET_SIZE      Report size must be less than packet size of interrupt.
  // E_HID_INVALID_EP_NUM                 Invalid EP number.
  // E_HID_MUST_LESS_THAN_PACKET0_SIZE    Data size in control must be less than packet0 size.
  // E_HID_NULL_POINTER                   NULL pointer.
  // E_HID_UNDEFINE                       Undefined error.
  // E_HID_INVALID_REG_NUM		  Invaild register number
/*  
	_SYSINFRA_ERRCODE_DEF(HID, UNDEFINE, TRUE, MODULE_ID_HID, 0)
	_SYSINFRA_ERRCODE_DEF(HID, NULL_POINTER, TRUE, MODULE_ID_HID, 1)
	_SYSINFRA_ERRCODE_DEF(HID, BUFFER_OVERRUN, TRUE, MODULE_ID_HID, 2)
	_SYSINFRA_ERRCODE_DEF(HID, INVALID_EP_NUM, TRUE, MODULE_ID_HID, 3)
	_SYSINFRA_ERRCODE_DEF(HID, MUST_LESS_THAN_PACKET0_SIZE, TRUE, MODULE_ID_HID, 4)
	_SYSINFRA_ERRCODE_DEF(HID, EXCEED_INT_IN_PACKET_SIZE, TRUE, MODULE_ID_HID, 5)
	_SYSINFRA_ERRCODE_DEF(HID, CTRL_REG_TAB_FULL, TRUE, MODULE_ID_HID, 6)
	_SYSINFRA_ERRCODE_DEF(HID, INVALID_REG_NUM, TRUE, MODULE_ID_HID, 7)				
    _SYSINFRA_VERSION_DEF(HID, HID_MAJOR_NUM, HID_MINOR_NUM, HID_BUILD_NUM)
  */
#define E_HID_UNDEFINE						_SYSINFRA_ERRCODE(TRUE, MODULE_ID_HID, 1)
#define E_HID_NULL_POINTER 					_SYSINFRA_ERRCODE(TRUE, MODULE_ID_HID, 2)
#define E_HID_BUFFER_OVERRUN				_SYSINFRA_ERRCODE(TRUE, MODULE_ID_HID, 3)
#define E_HID_INVALID_EP_NUM				_SYSINFRA_ERRCODE(TRUE, MODULE_ID_HID, 4)	
#define E_HID_MUST_LESS_THAN_PACKET0_SIZE	_SYSINFRA_ERRCODE(TRUE, MODULE_ID_HID, 5)
#define E_HID_EXCEED_INT_IN_PACKET_SIZE		_SYSINFRA_ERRCODE(TRUE, MODULE_ID_HID, 6)
#define E_HID_CTRL_REG_TAB_FULL				_SYSINFRA_ERRCODE(TRUE, MODULE_ID_HID, 7)
#define E_HID_INVALID_REG_NUM				_SYSINFRA_ERRCODE(TRUE, MODULE_ID_HID, 8)
											
#define HID_VERSION_NUM						_SYSINFRA_VERSION(HID_MAJOR_NUM, HID_MINOR_NUM, HID_BUILD_NUM)
       

#define HID_RPT_TYPE_INPUT		0x01
#define HID_RPT_TYPE_OUTPUT		0x02
#define HID_RPT_TYPE_FEATURE		0x03       

#define FIRST_PARAM				0xFF

// Max packet size of EP0
#define INT_IN_EP_NUM	  	       0x01    // interrupt endpoint number

#define	HID_MAX_PACKET_SIZE_EP0		64
// Max packet size of EP1
#define HID_MAX_PACKET_SIZE_EP1		8		//8->64 for keyboard

// For Contorl In and Control Out
#define	HID_USB_BUF_0			(USBD_SRAM+0x00)
// For Interrupt In
#define	HID_USB_BUF_1			(HID_USB_BUF_0+HID_MAX_PACKET_SIZE_EP0)
// For Control Setup
#define	HID_USB_BUF_SETUP		(HID_USB_BUF_1+HID_MAX_PACKET_SIZE_EP1)

#define g_HID_ar8UsbBufSetup 		((REG8 *) HID_USB_BUF_SETUP)
#define g_HID_ar8UsbBuf0 		((REG8 *) HID_USB_BUF_0)
#define g_HID_ar8UsbBuf1 		((REG8 *) HID_USB_BUF_1)


	static __inline	
	void
	_HID_CLR_CTRL_READY_AND_TRIG_STALL()
	{
		_DRVUSB_CLEAR_EP_READY_AND_TRIG_STALL(0);
		_DRVUSB_CLEAR_EP_READY_AND_TRIG_STALL(1);
	}

	static __inline
	void
	_HID_CLR_CTRL_READY()
	{
		_DRVUSB_CLEAR_EP_READY(0);
		_DRVUSB_CLEAR_EP_READY(1);
	}


	UINT32
	HID_GetVersion(void);

	ERRCODE
	HID_Open(
	   const UINT8 *au8DeviceDescriptor,   
	   const UINT8 *au8ConfigDescriptor,
       const UINT8 *au8VendorStringDesc,
       const UINT8 *au8ProductStringDesc,
       const UINT8 *au8StringSerial,
           void *pfGetInReport
	);
	
	void HID_Close(void);

	ERRCODE
	HID_SetReportDescriptor(
	    const UINT8* pu8ReportDescriptor,
	    UINT32 u32ReportDescriptorSize
	);

	ERRCODE
	HID_SetReport(
	    const UINT8* pu8Report,
	    UINT32 u32ReportSize
	);


#ifdef  __cplusplus
}
#endif

#endif // __HID_H__











