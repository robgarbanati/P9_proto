/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.  *
 *                                                              *
 ****************************************************************/
#ifndef __MASSSTOR_H__
#define __MASSSTOR_H__

#include "Platform.h"
#include "System/SysInfra.h"

#include "Driver/DrvUSB.h"


#include "MassStorDef.h"
#include "MassStorInfra.h"


#ifdef  __cplusplus
extern "C"
{
#endif

#define MASSSTOR_MAJOR_NUM	1
#define MASSSTOR_MINOR_NUM  0
#define MASSSTOR_BUILD_NUM	1


  // E_MASSSTOR_UNDEFINE	              Undefined error.
  // E_MASSSTOR_NULL_POINTER	          NULL pointer.
  // E_MASSSTOR_BUFFER_OVERRUN	        Allocated buffer is not enough.
  // E_MASSSTOR_INVALID_EP_NUM	        Invalid EP number.
  // E_MASSSTOR_SCSI_PDT_NOT_SUPPORT	  Non-support SCSI PDT.
  // E_MASSSTOR_INVALID_LUN	            Invalid LUN.
  // E_MASSSTOR_CTRL_REG_TAB_FULL	      Control register table is full.
  // E_MASSSTOR_SCSI_REG_TAB_FULL	      SCSI register table is full.

#define E_MASSSTOR_UNDEFINE				_SYSINFRA_ERRCODE(TRUE, MODULE_ID_MASSSTOR, 1)
#define E_MASSSTOR_NULL_POINTER			_SYSINFRA_ERRCODE(TRUE, MODULE_ID_MASSSTOR, 2)
#define E_MASSSTOR_BUFFER_OVERRUN 		_SYSINFRA_ERRCODE(TRUE, MODULE_ID_MASSSTOR, 3)
#define	E_MASSSTOR_INVALID_EP_NUM		_SYSINFRA_ERRCODE(TRUE, MODULE_ID_MASSSTOR, 4)
#define	E_MASSSTOR_SCSI_PDT_NOT_SUPPORT _SYSINFRA_ERRCODE(TRUE, MODULE_ID_MASSSTOR, 5)
#define	E_MASSSTOR_INVALID_LUN			_SYSINFRA_ERRCODE(TRUE, MODULE_ID_MASSSTOR, 6)
#define	E_MASSSTOR_CTRL_REG_TAB_FULL	_SYSINFRA_ERRCODE(TRUE, MODULE_ID_MASSSTOR, 7)
#define	E_MASSSTOR_SCSI_REG_TAB_FULL	_SYSINFRA_ERRCODE(TRUE, MODULE_ID_MASSSTOR, 8)

  // Define module version number.
#define MASSSTOR_VERSION_NUM			_SYSINFRA_VERSION(MASSSTOR_MAJOR_NUM, MASSSTOR_MINOR_NUM, MASSSTOR_BUILD_NUM)	
// Max packet size of EP0
#define	MASSSTOR_MAX_PACKET_SIZE_EP0		64
// Max packet size of EP1 and EP2
#define	MASSSTOR_MAX_PACKET_SIZE_EP1_EP2	64

// For Contorl In and Control Out
#define	MASSSTOR_USB_BUF_0			(USBD_SRAM+0x00)
// For Bulk In/Out
#define	MASSSTOR_USB_BUF_1			(MASSSTOR_USB_BUF_0+MASSSTOR_MAX_PACKET_SIZE_EP0)
#define	MASSSTOR_USB_BUF_2			(MASSSTOR_USB_BUF_1+MASSSTOR_MAX_PACKET_SIZE_EP1_EP2)
// For Control Setup
#define	MASSSTOR_USB_BUF_SETUP		(MASSSTOR_USB_BUF_2+MASSSTOR_MAX_PACKET_SIZE_EP1_EP2)

#define g_MassStor_ar8UsbBufSetup 	((REG8 *) MASSSTOR_USB_BUF_SETUP)
#define g_MassStor_ar8UsbBuf0 		((REG8 *) MASSSTOR_USB_BUF_0)
#define g_MassStor_ar8UsbBuf1 		((REG8 *) MASSSTOR_USB_BUF_1)
#define g_MassStor_ar8UsbBuf2 		((REG8 *) MASSSTOR_USB_BUF_2)

#define MASSSTOR_BULK_IN_EP_NUM			0x02
#define MASSSTOR_BULK_OUT_EP_NUM		0x03

#define MASSSTOR_BUILTIN_SCSI_CMD		10			//The number of mass stor built in scsi cmd

//#define MASSSTOR_USB_SRAM_BYTE_WRITE

#ifndef __inline
  #define inline  __inline
#endif

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_MASSSTOR_TRIG_CBW
// Description:                                                                                            
//				Trigger to receive the next CBW packet
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void _MASSSTOR_TRIG_CBW(void)
{
	_DRVUSB_SET_EP_BUF(MASSSTOR_BULK_OUT_EP_NUM,MASSSTOR_USB_BUF_1);
	//_DRVUSB_TRIG_EP3(MASSSTOR_MAX_PACKET_SIZE_EP1_EP2);
	//_DRVUSB_TRIG_EP3(31);
	_DRVUSB_TRIG_EP(MASSSTOR_BULK_OUT_EP_NUM,31);
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_MASSSTOR_TRIG_CSW
// Description:                                                                                            
//				Trigger the next CSW packet to transfer to HOST
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void _MASSSTOR_TRIG_CSW(void)
{
	_DRVUSB_SET_EP_BUF(MASSSTOR_BULK_IN_EP_NUM,MASSSTOR_USB_BUF_1);
	//_DRVUSB_TRIG_EP2(13);
	_DRVUSB_TRIG_EP(MASSSTOR_BULK_IN_EP_NUM,13);
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_MASSSTOR_SET_DATA_IN_BUF
// Parameter:
//				u32Addr [in] USB buffer address
// Description:                                                                                            
//				Specify USB buffer address for data in transfer
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void _MASSSTOR_SET_DATA_IN_BUF(
	UINT32 u32Addr
)
{
	_DRVUSB_SET_EP_BUF(MASSSTOR_BULK_IN_EP_NUM,u32Addr);
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_MASSSTOR_TRIG_DATA_IN
// Parameter:
//				u32TrigSize [in] Data packet size is ready to send to HOST 
// Description:                                                                                            
//				Request USB IP to trigger next Data In transaction
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void _MASSSTOR_TRIG_DATA_IN(
	UINT32 u32TrigSize
)
{
	//_DRVUSB_TRIG_EP2(u32TrigSize);
	_DRVUSB_TRIG_EP(MASSSTOR_BULK_IN_EP_NUM,u32TrigSize);
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_MASSSTOR_SET_DATA_OUT_BUF
// Parameter:
//				u32Addr [in] USB buffer address
// Description:                                                                                            
//				Specify USB buffer address for data out transfer
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void _MASSSTOR_SET_DATA_OUT_BUF(
	UINT32 u32Addr
)
{
	_DRVUSB_SET_EP_BUF(MASSSTOR_BULK_OUT_EP_NUM,u32Addr);
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_MASSSTOR_TRIG_DATA_OUT
// Parameter:
//				u32TrigSize [in] Maximum packet size in Data Out transaction
// Description:                                                                                            
//				Request USB IP to trigger next Data Out transaction
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void _MASSSTOR_TRIG_DATA_OUT(
	UINT32 u32TrigSize
)
{
	//_DRVUSB_TRIG_EP3(u32TrigSize);
	_DRVUSB_TRIG_EP(MASSSTOR_BULK_OUT_EP_NUM,u32TrigSize);
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_MASSSTOR_TRIG_BULK_IN_STALL
// Description:                                                                                            
//				Trigger a STALL packet to bulk in endpoint
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void _MASSSTOR_TRIG_BULK_IN_STALL(void)
{
	_DRVUSB_CLEAR_EP_READY_AND_TRIG_STALL(MASSSTOR_BULK_IN_EP_NUM);
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_MASSSTOR_TRIG_BULK_OUT_STALL
// Description:                                                                                            
//				Trigger a STALL packet to bulk out endpoint
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void _MASSSTOR_TRIG_BULK_OUT_STALL(void)
{
	_DRVUSB_CLEAR_EP_READY_AND_TRIG_STALL(MASSSTOR_BULK_OUT_EP_NUM);
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_MASSSTOR_CLR_BULK_IN_READY
// Description:                                                                                            
//				Clear bulk in endpoint register
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void _MASSSTOR_CLR_BULK_IN_READY(void)
{
	_DRVUSB_CLEAR_EP_READY(MASSSTOR_BULK_IN_EP_NUM);
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_MASSSTOR_CLR_BULK_OUT_READY
// Description:                                                                                            
//				Clear bulk out endpoint register
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void _MASSSTOR_CLR_BULK_OUT_READY(void)
{
	_DRVUSB_CLEAR_EP_READY(MASSSTOR_BULK_OUT_EP_NUM);
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_MASSSTOR_GET_BULK_IN_SIZE
// Return:
//				Bulk in endpoint data size
// Description:                                                                                            
//				Get bulk in endpoint data size
/*---------------------------------------------------------------------------------------------------------*/
static __inline
UINT32
_MASSSTOR_GET_BULK_IN_SIZE(void)
{
	return _DRVUSB_GET_EP_DATA_SIZE(MASSSTOR_BULK_IN_EP_NUM);
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	_MASSSTOR_GET_BULK_OUT_SIZE
// Return:
//				Bulk out endpoint data size
// Description:                                                                                            
//				Get bulk out endpoint data size
/*---------------------------------------------------------------------------------------------------------*/
static __inline
UINT32
_MASSSTOR_GET_BULK_OUT_SIZE(void)
{
	return _DRVUSB_GET_EP_DATA_SIZE(MASSSTOR_BULK_OUT_EP_NUM);
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	MassStor_GetVersion
// Return:
//				Version number
// Description:                                                                                            
//				Get version number of this module
/*---------------------------------------------------------------------------------------------------------*/
UINT32
MassStor_GetVersion(void);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	MassStor_Open
// Parameter:
//				pasMassStorScsiReg [in] The pointer of scsi command register table
//				u8MaxScsiRegCnt [in] The count of scsi command register table
//				pasMassStorLuReg [in] The pointer of mass storage logical unit register table
//				u8MaxLuRegCnt [in] The count of mass storage logical unit register table
//				pVoid [in] Reserved and assign a NULL value
// Return:
//				Successed: E_SUCCESS, Fail: >0 
// Description:                                                                                            
//				Open mass storage
/*---------------------------------------------------------------------------------------------------------*/
ERRCODE
MassStor_Open(
	S_MASSSTORINFRA_SCSI_REG *pasMassStorScsiReg,
	UINT8 u8MaxScsiRegCnt,
	S_MASSSTORINFRA_LU_REG	*pasMassStorLuReg,
	UINT8 u8MaxLuRegCnt,
	PVOID pVoid
);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	MassStor_Close
// Description:                                                                                            
//				Close mass storage
/*---------------------------------------------------------------------------------------------------------*/
void MassStor_Close(void);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	MassStor_GetScsiVendorInfo
// Parameter:
//				psScsiVendorInfo [out] Caller-allocated structure where current customizable SCSI vendor
//									   information will be placed
// Return:
//				Successed: E_SUCCESS, Fail: >0 
// Description:                                                                                            
//				Get current customizable SCSI vendor information of the USB mass storage device
/*---------------------------------------------------------------------------------------------------------*/
ERRCODE
MassStor_GetScsiVendorInfo(
    S_MASSSTORINFRA_SCSI_VENDOR_INFO *psScsiVendorInfo
);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	MassStor_SetScsiVendorInfo
// Parameter:
//				psScsiVendorInfo [in] Caller-allocated structure where caller places customizable SCSI 
//									  vendor information									  
// Return:
//				Successed: E_SUCCESS, Fail: >0 
// Description:                                                                                            
//				Set current customizable SCSI vendor information of the USB mass storage device
/*---------------------------------------------------------------------------------------------------------*/
ERRCODE
MassStor_SetScsiVendorInfo(
    const S_MASSSTORINFRA_SCSI_VENDOR_INFO *psScsiVendorInfo
);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	MassStor_RegisterLu
// Parameter:
//				psLuEntry [in] Caller-allocated array of structure where caller will fill in register
//							   information for each USB mass storage logical unit 
//				u32LuCnt [in] Number of USB mass storage logical units to register					  
// Return:
//				Successed: E_SUCCESS, Fail: >0 
// Description:                                                                                            
//				Register USB mass storage logical units
/*---------------------------------------------------------------------------------------------------------*/
ERRCODE
MassStor_RegisterLu(
    S_MASSSTORINFRA_LU_ENTRY *psLuEntry,
    UINT32 u32LuCnt
);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	MassStor_RegisterIsWriteProtectCallback
// Parameter:
//				u32Lun [in] Specify logical unit number of which "Is write-protected" callback function.
//							Must be between 0 and u16LuCnt, which is the argument passed in 
//							MassStor_RegisterLu.
//				pfnIsWriteProtectCallback [in] "Is write-protected" callback function.							 
//				pVoid [in] Argument passed to pfnIsWriteProtectCallback callback function when it is called.					  
// Return:
//				Successed: E_SUCCESS, Fail: >0 
// Description:                                                                                            
//				Register "Is write-protected" callback function
/*---------------------------------------------------------------------------------------------------------*/
ERRCODE
MassStor_RegisterIsWriteProtectCallback(
    UINT32 u32Lun,
    PFN_MASSSTORINFRA_IS_WRITE_PROTECT_CALLBACK pfnIsWriteProtectCallback,
    PVOID pVoid
);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	MassStor_RegisterSCSICallback
// Parameter:
//				psEntry [in] Caller-allocated array of structure where caller will fill in registered
//							 SCSI vendor commands
//				u32RegCnt [in] How many SCSI vendor commands are register							 
// Return:
//				Successed: E_SUCCESS, Fail: >0 
// Description:                                                                                            
//				Register SCSI vendor command implementations
/*---------------------------------------------------------------------------------------------------------*/
ERRCODE
MassStor_RegisterSCSICallback(
    S_MASSSTORINFRA_SCSI_ENTRY *psEntry,
    UINT32 u32RegCnt
);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	MassStor_MediaNotReady
// Return:
//				Successed: E_SUCCESS, Fail: >0 
// Description:                                                                                            
//				Media not ready
/*---------------------------------------------------------------------------------------------------------*/
ERRCODE
MassStor_MediaNotReady(void);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	MassStor_FailSCSI
// Return:
//				Successed: E_SUCCESS, Fail: >0 
// Description:                                                                                            
//				This SCSI vommand fails
/*---------------------------------------------------------------------------------------------------------*/
ERRCODE
MassStor_FailSCSI(void);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	MassStor_RevokeAllLu
// Return:
//				Successed: E_SUCCESS, Fail: >0 
// Description:                                                                                            
//				Revoke all registered USB mass storage logical units
/*---------------------------------------------------------------------------------------------------------*/
ERRCODE
MassStor_RevokeAllLu(void);
	
/*---------------------------------------------------------------------------------------------------------*/
// Function:	MassStor_GetCBW
// Return:
//				Return a pointer to a CBW structure 
// Description:                                                                                            
//				Get pointer to CBW structure. It's used to retrieve data of CBW packet of current BOT session
/*---------------------------------------------------------------------------------------------------------*/
S_MASSSTORINFRA_CBW *
MassStor_GetCBW(void);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	MassStor_GetCSW
// Return:
//				Return a pointer to a CSW structure 
// Description:                                                                                            
//				Get pointer to CSW structure. It's used to return status of this SCSI command handling of
//				current BOT session
/*---------------------------------------------------------------------------------------------------------*/
S_MASSSTORINFRA_CSW *
MassStor_GetCSW(void);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	MassStor_WriteCSWToUSBBuf
// Description:                                                                                            
//				Perpare CSW data on USB buffer
/*---------------------------------------------------------------------------------------------------------*/
void MassStor_WriteCSWToUSBBuf(void);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	MassStor_ReadySense
// Parameter:
//				u8SK [in] Sense key
//				u8ASC [in] Additional sense code							 
//				u8ASCQ [in] Additional sense code qualifier
// Description:                                                                                            
//				Ready sense code in a callee-maintained buffer, which will be transferred on SCSI Requese
//				Sense command		   
/*---------------------------------------------------------------------------------------------------------*/
void MassStor_ReadySense(
    UINT8 u8SK,
    UINT8 u8ASC,
    UINT8 u8ASCQ
);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	MassStor_NextBotCallback
// Parameter:
//				eBotCallbackId [in] A ID to identify which BOT stage to enter
// Description:                                                                                            
//				Specify which BOT stage to enter next
/*---------------------------------------------------------------------------------------------------------*/
void MassStor_NextBotCallback(
    E_MASSSTORINFRA_BOT_CALLBACK_ID eBotCallbackId
);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	MassStor_SetBulkOutData0
// Description:                                                                                            
//				Set bulk out Data stage is Data0
/*---------------------------------------------------------------------------------------------------------*/
void MassStor_SetBulkOutData0(void);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	MassStor_Reset
// Description:                                                                                            
//				Reset mass storage
/*---------------------------------------------------------------------------------------------------------*/
void MassStor_Reset(void);
	
/*---------------------------------------------------------------------------------------------------------*/
// Function:	MassStor_Reset
// Description:                                                                                            
//				Start mass storage BOT transfer
/*---------------------------------------------------------------------------------------------------------*/
void MassStor_Start(void);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	MassStor_GetRegLUN
//Return:
//				How many logical units are register
// Description:                                                                                            
//				Get current registered logical units
/*---------------------------------------------------------------------------------------------------------*/
UINT8
MassStor_GetRegLUN(void);


#ifdef  __cplusplus
}
#endif

#endif // __MASSSTOR_H__














