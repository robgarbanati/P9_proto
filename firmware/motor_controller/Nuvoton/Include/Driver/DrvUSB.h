/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved. *
 *                                                              *
 ****************************************************************/

#ifndef __DRVUSB_H__
#define __DRVUSB_H__

#include "Platform.h"
#include "System/SysInfra.h"

#ifdef  __cplusplus
extern "C"
{
#endif

#define DRVUSB_MAJOR_NUM	1
#define DRVUSB_MINOR_NUM	0
#define DRVUSB_BUILD_NUM	1

//typedef VOID (*PFN_DRVUSB_DELAY) (UINT32);

//////////////////////////////////////////////////////////////////////////////////////
// Error Code and Module Version Number
//////////////////////////////////////////////////////////////////////////////////////

// Error message

// E_DRVUSB_BUFFER_OVERRUN		Received data is more than buffer size
// E_DRVUSB_CTRL_REG_TAB_FULL	Control entry table full	
// E_DRVUSB_INVALID_EP_NUM		EndPoint number error
// E_DRVUSB_NULL_POINTER		Callback function is NULL
// E_DRVUSB_UNDEFINE			Undefined error 

#define E_DRVUSB_BUFFER_OVERRUN			_SYSINFRA_ERRCODE(TRUE, MODULE_ID_DRVUSB, 1)
#define E_DRVUSB_CTRL_REG_TAB_FULL		_SYSINFRA_ERRCODE(TRUE, MODULE_ID_DRVUSB, 2)
#define E_DRVUSB_INVALID_EP_NUM			_SYSINFRA_ERRCODE(TRUE, MODULE_ID_DRVUSB, 3)
#define E_DRVUSB_NULL_POINTER			_SYSINFRA_ERRCODE(TRUE, MODULE_ID_DRVUSB, 4)
#define E_DRVUSB_UNDEFINE				_SYSINFRA_ERRCODE(TRUE, MODULE_ID_DRVUSB, 5)

// Define module version number.
#define DRVUSB_VERSION_NUM _SYSINFRA_VERSION(DRVUSB_MAJOR_NUM, DRVUSB_MINOR_NUM, DRVUSB_BUILD_NUM)

/////////////////////////////////////////////////////////////////////////////////////
//	Marco
///////////////////////////////////////////////////////////////////////////////////// 
#define _DRVUSB_DATA0(Data) ((Data) & 0x37F)
#define _DRVUSB_DATA1(Data) ((Data) | 0x80)

//***************************************************
// 		USB REQUEST
//***************************************************
// Request Type
#define	REQ_STANDARD		0x00
#define	REQ_CLASS			0x20
#define	REQ_VENDOR			0x40

// Request
#define	GET_STATUS			0x00
#define	CLEAR_FEATURE		0x01
//#define					0x02
#define	SET_FEATURE			0x03
//#define					0x04
#define	SET_ADDRESS			0x05
#define	GET_DESCRIPTOR		0x06
#define	SET_DESCRIPTOR		0x07
#define	GET_CONFIGURATION	0x08
#define	SET_CONFIGURATION	0x09
#define	GET_INTERFACE		0x0A
#define	SET_INTERFACE		0x0B
#define	SYNC_FRAME			0x0C

//***************************************************
// USB Descriptor Type
//***************************************************
#define	DESC_DEVICE			0x01
#define	DESC_CONFIG			0x02
#define	DESC_STRING			0x03
#define	DESC_INTERFACE		0x04
#define	DESC_ENDPOINT		0x05
#define	DESC_QUALIFIER		0x06
#define	DESC_OTHERSPEED		0x07
// HID
#define DESC_HID 			0x21
#define DESC_HID_RPT 		0x22

//***************************************************
// USB Descriptor Length
//***************************************************
#define	LEN_DEVICE			18
#define	LEN_CONFIG			9
#define	LEN_INTERFACE		9
#define	LEN_ENDPOINT		7
// HID
#define LEN_HID				0x09

// USB Endpoint Type
#define	EP_ISO				0x01
#define	EP_BULK				0x02
#define	EP_INT				0x03

#define STS_IN_ACK			0x00						//	000: In ACK
#define STS_IN_NAK			0x10						//	001: In NAK
#define STS_OUT0_ACK		0x20						//	010: Out 0 ACK
#define STS_OUT1_ACK		0x60						//	110: Out 1 ACK
#define STS_SETUP_ACK		0x30						//	011: Setup ACK
#define STS_ISO				0x70						//	111: Isochronous translation end

#define CFGx_EPT_OUT		0x20						//	01: Out endpoint
#define CFGx_EPT_IN			0x40						//	10: IN endpoint


//***************************************************
// USB Feature Selector
//***************************************************
#define	FEATURE_DEVICE_REMOTE_WAKEUP	0x01
#define	FEATURE_ENDPOINT_HALT			0x00

//***************************************************
// Define USBINFRA READ/WRITE USB BUFFER
//***************************************************

#define _DRVUSB_READ_USB_BUF(UsbBuf, Mem, Len) \
	memcpy ((PVOID) (Mem), (PVOID) (UsbBuf), (Len))
	
#define _DRVUSB_WRITE_USB_BUF(UsbBuf, Mem, Len) \
	memcpy ((PVOID) (UsbBuf), (PVOID) (Mem), (Len))


///////////////////////////////////////////////////////////////////////////////////////////////
// Type define
//////////////////////////////////////////////////////////////////////////////////////////////

//***************************************************
// USB Device state
//***************************************************

typedef enum
{
	eDRVUSB_USB_STATE_DETACHED 	= 0,
	eDRVUSB_USB_STATE_ATTACHED 	= BIT0,
	eDRVUSB_USB_STATE_POWERED 	= eDRVUSB_USB_STATE_ATTACHED + BIT1,
	eDRVUSB_USB_STATE_DEFAULT 	= eDRVUSB_USB_STATE_POWERED + BIT2,
	eDRVUSB_USB_STATE_ADDRESS 	= eDRVUSB_USB_STATE_DEFAULT + BIT3,
	eDRVUSB_USB_STATE_CONFIGURED 	= eDRVUSB_USB_STATE_ADDRESS + BIT4,
	eDRVUSB_USB_STATE_SUSPENDED 	= BIT5,
} E_DRVUSB_USB_STATE;

//***************************************************
// USB Vender_info descriptor structure
//***************************************************
typedef struct
{
	UINT8 byLength;
	UINT8 byDescType;
	UINT16 	au16UnicodeString[50];
	
} S_DRVUSB_STRING_DESC;

typedef struct
{
	UINT16 u16VendorId;
	UINT16 u16ProductId;
	const S_DRVUSB_STRING_DESC *psVendorStringDesc;
	const S_DRVUSB_STRING_DESC *psProductStringDesc;

} S_DRVUSB_VENDOR_INFO;


//***************************************************
// Typedef USB Callback function
//***************************************************
typedef void (*PFN_DRVUSB_ATTACH_CALLBACK)(PVOID);
typedef void (*PFN_DRVUSB_DETACH_CALLBACK)(PVOID);

typedef void (*PFN_DRVUSB_BUS_RESET_CALLBACK)(PVOID);
typedef void (*PFN_DRVUSB_BUS_SUSPEND_CALLBACK)(PVOID);
typedef void (*PFN_DRVUSB_BUS_RESUME_CALLBACK)(PVOID);

typedef void (*PFN_DRVUSB_SETUP_CALLBACK)(PVOID);
typedef void (*PFN_DRVUSB_EP_IN_CALLBACK)(PVOID);
typedef void (*PFN_DRVUSB_EP_OUT_CALLBACK)(PVOID);

typedef void (*PFN_DRVUSB_CTRL_SETUP_CALLBACK)(PVOID);
typedef void (*PFN_DRVUSB_CTRL_DATA_IN_CALLBACK)(PVOID);
typedef void (*PFN_DRVUSB_CTRL_DATA_OUT_CALLBACK)(PVOID);

//***************************************************
// USB Control callback function structure
//***************************************************
typedef struct
{
	UINT8 u8RequestType;
	UINT8 u8Request;
	PFN_DRVUSB_CTRL_SETUP_CALLBACK 	pfnCtrlSetupCallback;
	PFN_DRVUSB_CTRL_DATA_IN_CALLBACK	pfnCtrlDataInCallback;
	PFN_DRVUSB_CTRL_DATA_OUT_CALLBACK	pfnCtrlDataOutCallback;
	PVOID								pVoid;
} S_DRVUSB_CTRL_CALLBACK_ENTRY;

typedef struct
{
	S_DRVUSB_CTRL_CALLBACK_ENTRY 		sCtrlCallbackEntry;
} S_DRVUSB_CTRL_CALLBACK_ENTRY_REG;


#if (!defined(__N572F065__))
#error "The selected target chip dose not support USB device. Define __N572F065__ to select N572F065 as target chip to use USB device!"
#else


////////////////////////////////////////////////////////////////////////////////////////////////
//			inline function
////////////////////////////////////////////////////////////////////////////////////////////////

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_DRVUSB_GET_MISC_INT
// Return:                                                                                                
//              Miscellaneous interrupts                                                                       
// Description:                                                                                            
//				Get miscellaneous interrupts
/*---------------------------------------------------------------------------------------------------------*/
static __inline
UINT32
_DRVUSB_GET_MISC_INT(void)
{
	return (USBD.IEF.u32Reg & (USBD_IEF_WAKEUP + USBD_IEF_FLD + USBD_IEF_USB + USBD_IEF_BUS));
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_DRVUSB_ENABLE_MISC_INT
// Parameter:  
//				u32Flags [in] USB interrupt events.
// Description:                                                                                            
//				Enable miscellaneous interrups
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void _DRVUSB_ENABLE_MISC_INT(
	UINT32 u32Flags
)
{
	USBD.IEF.u32Reg = (USBD.IEF.u32Reg | (u32Flags & (USBD_IEF_WAKEUP + USBD_IEF_FLD + USBD_IEF_USB + USBD_IEF_BUS)));
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_DRVUSB_DISABLE_MISC_INT
// Parameter:  
//				u32Flags [in] USB interrupt events.
// Description:                                                                                            
//				Disable miscellaneous interrups
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void _DRVUSB_DISABLE_MISC_INT(
	UINT32 u32Flags
)
{
	USBD.IEF.u32Reg = (USBD.IEF.u32Reg & ~(u32Flags & (USBD_IEF_WAKEUP + USBD_IEF_FLD + USBD_IEF_USB + USBD_IEF_BUS)));
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_DRVUSB_ENABLE_WAKEUP
// Description:                                                                                            
//				Enable USB wakeup function
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void _DRVUSB_ENABLE_WAKEUP(void)
{
	USBD.IEF.u32Reg = USBD.IEF.u32Reg | USBD_IEF_WAKEUPEN;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_DRVUSB_DISABLE_WAKEUP
// Description:                                                                                            
//				Disable USB wakeup function
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void _DRVUSB_DISABLE_WAKEUP(void)
{
	USBD.IEF.u32Reg =  USBD.IEF.u32Reg & ~USBD_IEF_WAKEUPEN;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_DRVUSB_ENABLE_WAKEUP_INT
// Description:                                                                                            
//				Enable wakeup interrupt
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void _DRVUSB_ENABLE_WAKEUP_INT(void)
{
	USBD.IEF.u32Reg =  USBD.IEF.u32Reg | USBD_IEF_WAKEUP;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_DRVUSB_DISABLE_WAKEUP_INT
// Description:                                                                                            
//				Disable wakeup interrupt
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void _DRVUSB_DISABLE_WAKEUP_INT(void)
{
	USBD.IEF.u32Reg =  USBD.IEF.u32Reg & ~USBD_IEF_WAKEUP;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_DRVUSB_CLEAR_WAKEUP_INT
// Description:                                                                                            
//				Clear wakeup interrupt
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void _DRVUSB_CLEAR_WAKEUP_INT(void)
{
	USBD.EVF.u32Reg =  USBD.EVF.u32Reg | USBD_EVF_WAKEUP;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_DRVUSB_GET_FLD_STS
// Return:                                                                                                
//              float-detection status                                                                       
// Description:                                                                                            
//				Get float-detection status
/*---------------------------------------------------------------------------------------------------------*/
static __inline
UINT32 
_DRVUSB_GET_FLD_STS(void)
{
	return USBD.FLODETB.u32Reg;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_DRVUSB_ENABLE_FLD_INT
// Description:                                                                                            
//				Enable float-detection interrupt
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void _DRVUSB_ENABLE_FLD_INT(void)
{
	USBD.IEF.u32Reg = USBD.IEF.u32Reg | USBD_IEF_FLD; 
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_DRVUSB_DISABLE_FLD_INT
// Description:                                                                                            
//				Disable float-detection interrupt
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void _DRVUSB_DISABLE_FLD_INT(void)
{
	USBD.IEF.u32Reg = USBD.IEF.u32Reg &~ USBD_IEF_FLD;  
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_DRVUSB_CLEAR_FLD_INT
// Description:                                                                                            
//				Clear float-detection interrupt
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void _DRVUSB_CLEAR_FLD_INT(void)
{
	USBD.EVF.u32Reg = USBD.EVF.u32Reg | USBD_EVF_FLD;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_DRVUSB_ENABLE_USB_INT
// Description:                                                                                            
//				Enable USB interrupt
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void _DRVUSB_ENABLE_USB_INT(void)
{
	USBD.IEF.u32Reg = USBD.IEF.u32Reg | USBD_IEF_USB;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_DRVUSB_DISABLE_USB_INT
// Description:                                                                                            
//				Disable USB interrupt
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void _DRVUSB_DISABLE_USB_INT(void)
{
	USBD.IEF.u32Reg = USBD.IEF.u32Reg &~ USBD_IEF_USB;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_DRVUSB_CLEAR_USB_INT
// Description:                                                                                            
//				Clear USB interrupt
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void _DRVUSB_CLEAR_USB_INT(void)
{
	USBD.EVF.u32Reg = USBD.EVF.u32Reg | USBD_EVF_USB; 
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_DRVUSB_ENABLE_BUS_INT
// Description:                                                                                            
//				Enable bus interrupt
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void _DRVUSB_ENABLE_BUS_INT(void)
{
	USBD.IEF.u32Reg = USBD.IEF.u32Reg | USBD_IEF_BUS;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_DRVUSB_DISABLE_BUS_INT
// Description:                                                                                            
//				Disable bus interrupt
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void _DRVUSB_DISABLE_BUS_INT(void)
{
	USBD.IEF.u32Reg = USBD.IEF.u32Reg &~ USBD_IEF_BUS;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_DRVUSB_CLEAR_BUS_INT
// Description:                                                                                            
//				Clear bus interrupt
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void _DRVUSB_CLEAR_BUS_INT(void)
{
	USBD.EVF.u32Reg = USBD.EVF.u32Reg | USBD_EVF_BUS;
}


/*---------------------------------------------------------------------------------------------------------*/
// Function:	_DRVUSB_GET_BUS_ATTR
// Return:
//				Bus state and attribution
// Description:                                                                                            
//				Get bus state and attribution
/*---------------------------------------------------------------------------------------------------------*/
static __inline
UINT32
_DRVUSB_GET_BUS_ATTR(void)
{
	return USBD.ATTR.u32Reg;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_DRVUSB_SET_BUS_ATTR
// Parameter:
//        u32Val  Bus attribute value
// Description:                                                                                            
//				Set bus state and attribution
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
_DRVUSB_SET_BUS_ATTR(
	UINT32 u32Val
)
{
	USBD.ATTR.u32Reg = u32Val;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_DRVUSB_CLEAR_EP_READY_AND_TRIG_STALL
// Parameter:
//				u32EPNum [in] EP number(0~5)	
// Description:                                                                                            
//				Clear EPx In/Out ready and respons STALL
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void _DRVUSB_CLEAR_EP_READY_AND_TRIG_STALL(
	UINT32 u32EPNum
)
{
	USBD.EP[u32EPNum].CFGP.u32Reg = USBD.EP[u32EPNum].CFGP.u32Reg | (USBD_CFGP_STALL| USBD_CFGP_CFGP);
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_DRVUSB_CLEAR_EP_READY
// Parameter:
//				u32EPNum [in] EP number(0~5)	
// Description:                                                                                            
//				Clear EPx In/Out ready
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void _DRVUSB_CLEAR_EP_READY(
	UINT32 u32EPNum	
)
{
	USBD.EP[u32EPNum].CFGP.u32Reg = USBD.EP[u32EPNum].CFGP.u32Reg | USBD_CFGP_CFGP;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_DRVUSB_SET_SETUP_BUF
// Parameter:
//				u32BufAddr [in] Buffer address for setup token	
// Description:                                                                                            
//				Specify buffer address for Setup transaction
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void _DRVUSB_SET_SETUP_BUF(
	UINT32 u32BufAddr
)
{
	USBD.BUFSEG.u32Reg =  u32BufAddr - USBD_SRAM;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_DRVUSB_SET_EP_BUF
// Parameter:
//				u32EPNum [in] EP number(0~5)
//				u32BufAddr [in] Buffer address for EP	
// Description:                                                                                            
//				Specify buffer address for EPx
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void _DRVUSB_SET_EP_BUF(
    UINT32 u32EPNum,
	UINT32 u32BufAddr
)
{
	USBD.EP[u32EPNum].BUFSEG.u32Reg = u32BufAddr - USBD_SRAM;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_DRVUSB_TRIG_EP
// Parameter:
//				u32EPNum [in] EP number(0~5) for trigger Data In/Out transaction
//				u32TrigSize [in] For Data out transaction, maximum data size transferred from host.
//								 For Data in transaction, how many data transferred to host.	
// Description:                                                                                            
//				Trigger next transaction for EPx
/*---------------------------------------------------------------------------------------------------------*/
static __inline 
void _DRVUSB_TRIG_EP( 
	UINT32 u32EPNum,
	UINT32 u32TrigSize
)
{
	USBD.EP[u32EPNum].MXPLD.u32Reg = u32TrigSize;		 			
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_DRVUSB_SET_EP_TOG_BIT
// Parameter:
//				u32EPNum [in] EP number(0~5)
//				bData0 [in] Specify Data0 or Data1 for Data in transaction	
// Description:                                                                                            
//				Specify Data0 or Data1 after IN token toggle automatically after host ack
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void _DRVUSB_SET_EP_TOG_BIT(
    UINT32 u32EPNum,
	BOOL bData0
)
{
	UINT32 u32CFGn = USBD.EP[u32EPNum].CFG.u32Reg;
	USBD.EP[u32EPNum].CFG.u32Reg = (bData0 ? _DRVUSB_DATA0(u32CFGn) : _DRVUSB_DATA1(u32CFGn));
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_DRVUSB_GET_EP_DATA_SIZE
// Parameter:
//				u32EPNum [in] EP number(0~5)
// Description:                                                                                            
//				Length of data transmitted to or received from HOST for EPx
/*---------------------------------------------------------------------------------------------------------*/
static __inline
UINT32
_DRVUSB_GET_EP_DATA_SIZE(
    UINT32 u32EPNum
)
{
	return USBD.EP[u32EPNum].MXPLD.u32Reg;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_DRVUSB_SET_EVF
// Parameter:
//				u32Data [in] Specify Data IN EVF
// Description:                                                                                            
//				Set interrupt event flag
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void _DRVUSB_SET_EVF(
	UINT32 u32Data
)
{
	USBD.EVF.u32Reg = u32Data;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_DRVUSB_GET_EVF
// Return:
//				EVF register value
// Description:                                                                                            
//				Get interrupt event flag
/*---------------------------------------------------------------------------------------------------------*/
static __inline
UINT32
_DRVUSB_GET_EVF(void)
{
	return USBD.EVF.u32Reg;
}	

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_DRVUSB_CLEAR_EP_STALL
// Parameter:
//				u32EPNum [in] EP number(0~5)
// Description:                                                                                            
//				Clear EPx force device to respone STALL
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void _DRVUSB_CLEAR_EP_STALL(
	UINT32 u32EPNum
)
{
	USBD.EP[u32EPNum].CFGP.u32Reg = USBD.EP[u32EPNum].CFGP.u32Reg &~ USBD_CFGP_STALL;  
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_DRVUSB_TRIG_EP_STALL
// Parameter:
//				u32EPNum [in] EP number(0~5)
// Description:                                                                                            
//				Trigger EPx force device to respone STALL
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void _DRVUSB_TRIG_EP_STALL(
	UINT32 u32EPNum
)
{
	USBD.EP[u32EPNum].CFGP.u32Reg = USBD.EP[u32EPNum].CFGP.u32Reg | USBD_CFGP_STALL;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_DRVUSB_CLEAR_EP_DSQ
// Parameter:
//				u32EPNum [in] EP number(0~5)
// Description:                                                                                            
//				Clear EPx Specify Data 0 or 1 after IN token toggle automatically after host ACK
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void _DRVUSB_CLEAR_EP_DSQ(
	UINT32 u32EPNum
)
{
	USBD.EP[u32EPNum].CFG.u32Reg = USBD.EP[u32EPNum].CFG.u32Reg & ~USBD_CFG_DSQ; 
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_DRVUSB_SET_CFG
// Parameter:
//				u32CFGNum [in] Configuration number(0~5)
//				u32Data [in] Specify the data of CFG
// Description:                                                                                            
//				Set configuration CFG of EPx
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void _DRVUSB_SET_CFG(
    UINT32 u32CFGNum,
	UINT32 u32Data
)
{
	USBD.EP[u32CFGNum].CFG.u32Reg = (u32Data & 0x03ff);
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_DRVUSB_GET_CFG
// Parameter:
//				u32CFGNum [in] Configuration number(0~5)
// Return:
//				CFG registe value
// Description:                                                                                            
//				Get configuration CFG of EPx
/*---------------------------------------------------------------------------------------------------------*/
static __inline
UINT32
_DRVUSB_GET_CFG(		
	UINT32 u32CFGNum
)
{
	return USBD.EP[u32CFGNum].CFG.u32Reg & 0x03ff;
}		

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_DRVUSB_SET_FADDR
// Parameter:
//				u32Addr [in] The address of a device on the USB BUS
// Description:                                                                                            
//				Set function address.
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void _DRVUSB_SET_FADDR(
	UINT32 u32Addr
)
{
	USBD.FADDR.u32Reg = u32Addr & 0x7f;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_DRVUSB_GET_FADDR
// Return:
//				FADDR register value
// Description:                                                                                            
//				Get function address.
/*---------------------------------------------------------------------------------------------------------*/
static __inline
UINT32
_DRVUSB_GET_FADDR(void)
{
	return (USBD.FADDR.u32Reg & 0x7f);
}		

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_DRVUSB_GET_STS
// Return:
//				STS register value
// Description:                                                                                            
//				Get system states
/*---------------------------------------------------------------------------------------------------------*/
static __inline
UINT32
_DRVUSB_GET_STS(void)
{
	return USBD.STS.u32Reg;
}	

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_DRVUSB_SET_CFGP
// Parameter:
//				u32CFGPNum [in] Configuration number(0~5)
//				u32Data [in] Specify the data of CFGP
// Description:                                                                                            
//				Set configuration CFGP of EPx
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void _DRVUSB_SET_CFGP(
    UINT8  u8CFGPNum,
	UINT32 u32Data
)
{
	USBD.EP[u8CFGPNum].CFGP.u32Reg = u32Data & 0x03;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_DRVUSB_SET_CFGP0
// Parameter:
//				u32Data [in] Specify the data of CFGP0
// Description:                                                                                            
//				Set configuration CFGP of EP0
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void _DRVUSB_SET_CFGP0(
	UINT32 u32Data
)
{
	USBD.EP[0].CFGP.u32Reg = u32Data & 0x03;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_DRVUSB_SET_CFGP1
// Parameter:
//				u32Data [in] Specify the data of CFGP1
// Description:                                                                                            
//				Set configuration CFGP of EP1
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void _DRVUSB_SET_CFGP1(
	UINT32 u32Data
)
{
	USBD.EP[1].CFGP.u32Reg = u32Data & 0x03;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_DRVUSB_SET_CFGP2
// Parameter:
//				u32Data [in] Specify the data of CFGP2
// Description:                                                                                            
//				Set configuration CFGP of EP2
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void _DRVUSB_SET_CFGP2(
	UINT32 u32Data
)
{
	USBD.EP[2].CFGP.u32Reg = u32Data & 0x03;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_DRVUSB_SET_CFGP3
// Parameter:
//				u32Data [in] Specify the data of CFGP3
// Description:                                                                                            
//				Set configuration CFGP of EP3
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void _DRVUSB_SET_CFGP3(
	UINT32 u32Data
)
{
	USBD.EP[3].CFGP.u32Reg = u32Data & 0x03;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_DRVUSB_SET_CFGP4
// Parameter:
//				u32Data [in] Specify the data of CFGP4
// Description:                                                                                            
//				Set configuration CFGP of EP4
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void _DRVUSB_SET_CFGP4(
	UINT32 u32Data
)
{
	USBD.EP[4].CFGP.u32Reg = u32Data & 0x03;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_DRVUSB_SET_CFGP5
// Parameter:
//				u32Data [in] Specify the data of CFGP5
// Description:                                                                                            
//				Set configuration CFGP of EP5
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void _DRVUSB_SET_CFGP5(
	UINT32 u32Data
)
{
	USBD.EP[5].CFGP.u32Reg = u32Data & 0x03;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_DRVUSB_GET_CFGP
// Parameter:
//				u32CFGPNum [in] Configuration number(0~5)
// Return:
//				GFGPx register value
// Description:                                                                                            
//				Get configuration CFGP of EPx
/*---------------------------------------------------------------------------------------------------------*/
static __inline
UINT32
_DRVUSB_GET_CFGP(
	UINT32 u32CFGPNum
)
{
	return 	USBD.EP[u32CFGPNum].CFGP.u32Reg & 0x03;
}	
 
/*---------------------------------------------------------------------------------------------------------*/
// Function:	_DRVUSB_ENABLE_USB
// Description:                                                                                            
//				Enable USB,PHY and use remote wake-up
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void _DRVUSB_ENABLE_USB(void)
{
//	USBD.ATTR.u32Reg = 0xD0;
	USBD.ATTR.u32Reg = 0x390;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_DRVUSB_DISABLE_USB
// Description:                                                                                            
//				Disable USB,PHY and use remote wake-up
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void _DRVUSB_DISABLE_USB(void)
{
//	USBD.ATTR.u32Reg = 0x40;
	USBD.ATTR.u32Reg = 0x330;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_DRVUSB_DISABLE_PHY
// Description:                                                                                            
//				Disable PHY and don't use remote wake-up 
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void _DRVUSB_DISABLE_PHY(void)
{
//	USBD.ATTR.u32Reg = 0xC0;
	USBD.ATTR.u32Reg = 0x380;
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_DRVUSB_ENABLE_SE0
// Description:                                                                                            
//				Enable USB PHY to drive SE0 
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void _DRVUSB_ENABLE_SE0(void)
{
	USBD.DRVSE0.u32Reg = 0x01; 
} 

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_DRVUSB_DISABLE_SE0
// Description:                                                                                            
//				Disable USB PHY to drive SE0 
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void _DRVUSB_DISABLE_SE0(void)
{
	USBD.DRVSE0.u32Reg = 0x00; 
} 

//*******************************************************/
// APIs declared					
//*******************************************************/

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvUsb_GetVersion
// Return:
//				Version number
// Description:                                                                                            
//				Get this module version 
/*---------------------------------------------------------------------------------------------------------*/
UINT32
DrvUsb_GetVersion(void);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvUsb_Open
// Return:
//				Successed: E_SUCCESS, Fail: >0 
// Description:                                                                                            
//				Open USB functions 
/*---------------------------------------------------------------------------------------------------------*/
ERRCODE
DrvUsb_Open(
	PVOID pVoid
);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvUsb_Close
// Description:                                                                                            
//				Close USB functions 
/*---------------------------------------------------------------------------------------------------------*/
void DrvUsb_Close(void);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvUsb_RegisterUsbAttachCallback
// Parameter:
//				pfnUsbAttachCallback [in] Callback function to register
//				pUsbAttachCallbackArgu [in] Argument passed to pfnUsbAttachCallback
// Return:
//				Successed: E_SUCCESS, Fail: >0 
// Description:                                                                                            
//				Register PFN_DRVUSB_ATTACH_CALLBACK to be called on USB attach
/*---------------------------------------------------------------------------------------------------------*/
ERRCODE
DrvUsb_RegisterUsbAttachCallback(
    PFN_DRVUSB_ATTACH_CALLBACK pfnUsbAttachCallback,
    PVOID pUsbAttachCallbackArgu
);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvUsb_RegisterUsbDetachCallback
// Parameter:
//				pfnUsbDetachCallback [in] Callback function to register
//				pUsbDetachCallbackArgu [in] Argument passed to pfnUsbDetachCallback
// Return:
//				Successed: E_SUCCESS, Fail: >0 
// Description:                                                                                            
//				Register PFN_DRVUSB_DETACH_CALLBACK to be called on USB detach
/*---------------------------------------------------------------------------------------------------------*/
ERRCODE
DrvUsb_RegisterUsbDetachCallback(
    PFN_DRVUSB_DETACH_CALLBACK pfnUsbDetachCallback,
    PVOID pUsbDetachCallbackArgu
);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvUsb_RegisterUsbBusResetCallback
// Parameter:
//				pfnUsbBusResetCallback [in] Callback function to register
//				pUsbBusResetCallbackArgu [in] Argument passed to pfnUsbBusResetCallback
// Return:
//				Successed: E_SUCCESS, Fail: >0 
// Description:                                                                                            
//				Register PFN_DRVUSB_BUS_RESET_CALLBACK to be called on USB bus reset
/*---------------------------------------------------------------------------------------------------------*/
ERRCODE
DrvUsb_RegisterUsbBusResetCallback(
    PFN_DRVUSB_BUS_RESET_CALLBACK pfnUsbBusResetCallback,
    PVOID pUsbBusResetCallbackArgu
);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvUsb_RegisterUsbBusSuspendCallback
// Parameter:
//				pfnUsbBusSuspendCallback [in] Callback function to register
//				pUsbBusSuspendCallbackArgu [in] Argument passed to pfnUsbBusSuspendCallback
// Return:
//				Successed: E_SUCCESS, Fail: >0 
// Description:                                                                                            
//				Register PFN_DRVUSB_BUS_SUSPEND_CALLBACK to be called on USB bus suspend
/*---------------------------------------------------------------------------------------------------------*/
ERRCODE
DrvUsb_RegisterUsbBusSuspendCallback(
    PFN_DRVUSB_BUS_SUSPEND_CALLBACK pfnUsbBusSuspendCallback,
    PVOID pUsbBusSuspendCallbackArgu
);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvUsb_RegisterUsbBusResumeCallback
// Parameter:
//				pfnUsbBusResumeCallback [in] Callback function to register
//				pUsbBusResumeCallbackArgu [in] Argument passed to pfnUsbBusResumeCallback
// Return:
//				Successed: E_SUCCESS, Fail: >0 
// Description:                                                                                            
//				Register PFN_DRVUSB_BUS_RESUME_CALLBACK to be called on USB bus resume
/*---------------------------------------------------------------------------------------------------------*/
ERRCODE
DrvUsb_RegisterUsbBusResumeCallback(
    PFN_DRVUSB_BUS_RESUME_CALLBACK pfnUsbBusResumeCallback,
    PVOID pUsbBusResumeCallbackArgu
);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvUsb_RegisterUsbSetupCallback
// Parameter:
//				pfnUsbSetupCallback [in] Callback function to register
//				pUsbSetupCallbackArgu [in] Argument passed to pfnUsbSetupCallback
// Return:
//				Successed: E_SUCCESS, Fail: >0 
// Description:                                                                                            
//				Register PFN_DRVUSB_SETUP_CALLBACK to be called on USB setup transaction
/*---------------------------------------------------------------------------------------------------------*/
ERRCODE
DrvUsb_RegisterUsbSetupCallback(
    PFN_DRVUSB_SETUP_CALLBACK pfnUsbSetupCallback,
    PVOID pUsbSetupCallbackArgu
);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvUsb_RegisterUsbEPInCallback
// Parameter:
//				u32EPNum [in] On which EP number to register
//				pfnUsbEPInCallback [in] Callback function to register
//				pUsbEPInCallbackArgu [in] Argument passed to pfnUsbEPInCallback
// Return:
//				Successed: E_SUCCESS, Fail: >0 
// Description:                                                                                            
//				Register PFN_DRVUSB_EP_IN_CALLBACK to be called on USB Data In transaction completed 
//				of the specified EP number
/*---------------------------------------------------------------------------------------------------------*/
ERRCODE
DrvUsb_RegisterUsbEPInCallback(
    UINT32 u32EPNum,
    PFN_DRVUSB_EP_IN_CALLBACK pfnUsbEPInCallback,
    PVOID pUsbEPInCallbackArgu
);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvUsb_RegisterUsbEPOutCallback	
// Parameter:
//				u32EPNum [in] On which EP number to register
//				pfnUsbEPOutCallback [in] Callback function to register
//				pUsbEPOutCallbackArgu [in] Argument passed to pfnUsbEPOutCallback
// Return:
//				Successed: E_SUCCESS, Fail: >0 
// Description:                                                                                            
//				Register PFN_DRVUSB_EP_OUT_CALLBACK to be called on USB Data Out transaction completed 
//				of the specified EP number
/*---------------------------------------------------------------------------------------------------------*/
ERRCODE
DrvUsb_RegisterUsbEPOutCallback(
    UINT32 u32EPNum,
    PFN_DRVUSB_EP_OUT_CALLBACK pfnUsbEPOutCallback,
    PVOID pUsbEPOutCallbackArgu
);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvUsb_PreDispatchEvent	
// Description:                                                                                            
//				Pre-dispatch USB events 
/*---------------------------------------------------------------------------------------------------------*/
void DrvUsb_PreDispatchEvent(void);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvUsb_DispatchEvent	
// Description:                                                                                            
//				Dispatch USB events 
/*---------------------------------------------------------------------------------------------------------*/
void DrvUsb_DispatchEvent(void);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvUsb_IsData0Out	
// Parameter:
//				u32EPNum [in] EP number
// Return:
//				TRUE: Data0 transaction, FALSE: Data1 transaction 
// Description:                                                                                            
//				Tell this is Data0 or Data1 on Data Out transaction complete
/*---------------------------------------------------------------------------------------------------------*/
BOOL
DrvUsb_IsData0Out(
    UINT32 u32EPNum
);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvUsb_GetUsbState	
// Return:
//				USB state 
// Description:                                                                                            
//				Get USB state
/*---------------------------------------------------------------------------------------------------------*/
E_DRVUSB_USB_STATE
DrvUsb_GetUsbState(void);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvUsb_SetUsbState	
// Parameter:
//				eUsbState [in] USB state to set
// Description:                                                                                            
//				Set USB state
/*---------------------------------------------------------------------------------------------------------*/
void DrvUsb_SetUsbState(
	E_DRVUSB_USB_STATE eUsbState
);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvUsb_InitCtrlHandler	
// Parameter:
//				u32SetupBufAddr [in] The setup buffer address
//				u8CtrlDataInEP [in] Control pipe Data In EP number 
//				u8CtrlDataOutEP [in] Control pipe Data Out EP number
// Return:
//				Successed: E_SUCCESS, Fail: >0 
// Description:                                                                                            
//				Initiate control pipe handler
/*---------------------------------------------------------------------------------------------------------*/
ERRCODE
DrvUsb_InitCtrlHandler(
	UINT32 u32SetupBufAddr,
	UINT8 u8CtrlDataInEP,
	UINT8 u8CtrlDataOutEP
);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvUsb_RegisterCtrHandler	
// Parameter:
//				pasCtrlCallbackEntry [in] The pointer of control callback entry table
//				u32RegCnt [in] The count of control callback entries on callback entry table
// Return:
//				Successed: E_SUCCESS, Fail: >0 
// Description:                                                                                            
//				Register control pipe callback entry table to control pipe handler.
//				The life scale of callback entry table should be global.
/*---------------------------------------------------------------------------------------------------------*/
ERRCODE
DrvUsb_RegisterCtrHandler(
    S_DRVUSB_CTRL_CALLBACK_ENTRY *pasCtrlCallbackEntry,
    UINT32 u32RegCnt
);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvUsb_CtrlSetupAck	
// Parameter:
//				pArgu [in] Argument				
// Description:                                                                                            
//				This function is one of control pipe handler for setup token.
//				It is a callback function for USB setup transaction.
//				It should be as an argument of DrvUsb_RegisterUsbSetupCallback().
/*---------------------------------------------------------------------------------------------------------*/
void DrvUsb_CtrlSetupAck(
    PVOID pArgu
);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvUsb_CtrlGetSetupData	
// Return:
//				The pointer of Setup token data. It points to a UINT8 array which stores Setup token data. 
//				The size of this array is 8 byte.				
// Description:                                                                                            
//				Get Setup token data.
/*---------------------------------------------------------------------------------------------------------*/
PUINT8 
DrvUsb_CtrlGetSetupData(void);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvUsb_CtrlDataInAck	
// Parameter:
//				pArgu [in] Argument 
// Description:                                                                                            
//				This function is one of control pipe handler for Data In transaction.
//				It is a callback function for USB Data In transaction.
//				It should be as an argument of DrvUsb_RegisterUsbEPInCallback().
/*---------------------------------------------------------------------------------------------------------*/
void DrvUsb_CtrlDataInAck(
    PVOID pArgu
);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvUsb_CtrlDataOutAck	
// Parameter:
//				pArgu [in] Argument 
// Description:                                                                                            
//				This function is one of control pipe handler for Data Out transaction.
//				It is a callback function for USB Data Out transaction.
//				It should be as an argument of DrvUsb_RegisterUsbEPOutCallback().
/*---------------------------------------------------------------------------------------------------------*/
void DrvUsb_CtrlDataOutAck(
	PVOID pArgu
);

#endif // (!defined(__N572F065__))

#ifdef  __cplusplus
}
#endif //__cplusplus


#endif // __DRVUSB_H__
















