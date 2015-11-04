/****************************************************************
 *                                                              *
 *  Copyright (c) Nuvoton Technology Corp. All rights reserved. *
 *                                                              *
 ****************************************************************/
 
#ifndef __DRVSDCARD_H__
#define __DRVSDCARD_H__

// Include header file
#include "Platform.h"
#include "System/SysInfra.h"
#include "Driver/DrvSPI.h"

#ifdef  __cplusplus
extern "C" {
#endif	// __cplusplus

#define SDCARD_MAJOR_NUM 2
#define SDCARD_MINOR_NUM 00
#define SDCARD_BUILD_NUM 002
#define SDCARD_VERSION_NUM    _SYSINFRA_VERSION(SDCARD_MAJOR_NUM, SDCARD_MINOR_NUM, SDCARD_BUILD_NUM)

// ------------------------------------------------------------------------------
// Define the Error Code
// ------------------------------------------------------------------------------
// E_SDCARD_INITIAL_FAIL				Invalid SDCard Fail
// E_SDCARD_READ_ERROR					Read Data From SDCard Fail
// E_SDCARD_READ_BUSY						Read Data Busy
// E_SDCARD_WRITE_ERROR					Write Data to SDCard Fail
// E_SDCARD_WRITE_BUSY 					Write Data Busy
#define E_SDCARD_INITIAL_FAIL        _SYSINFRA_ERRCODE(TRUE, MODULE_ID_SDCARD, 1)
#define E_SDCARD_READ_ERROR          _SYSINFRA_ERRCODE(TRUE, MODULE_ID_SDCARD, 2)
#define E_SDCARD_READ_BUSY           _SYSINFRA_ERRCODE(TRUE, MODULE_ID_SDCARD, 3)
#define E_SDCARD_WRITE_ERROR         _SYSINFRA_ERRCODE(TRUE, MODULE_ID_SDCARD, 4)
#define E_SDCARD_WRITE_BUSY          _SYSINFRA_ERRCODE(TRUE, MODULE_ID_SDCARD, 5)

//---------------------------------------------------------------
// SD Card NB State 
//---------------------------------------------------------------
typedef enum{
	eSDCARD_IDLE_STATE = 0,
	eSDCARD_READ_SUCCESS,
	eSDCARD_READ_BUSY,
	eSDCARD_WRITE_SUCCESS,
	eSDCARD_WRITE_BUSY
} E_SDCARD_STATE;

//---------------------------------------------------------------
// SD Card Type 
//---------------------------------------------------------------
typedef enum{
	eTYPE_UNKNOWN_CARD = 0,
	eTYPE_SD_CARD,
	eTYPE_MMC_CARD,
	eTYPE_SDIO_CARD,
	eTYPE_SD20_HIGH_CARD
} E_SDCAR_TYPE;

#define SDCard_Open(u32SpiPortSelect,eDeviceSelect)	SDCard_OpenEx(u32SpiPortSelect,eDeviceSelect,6000000) 

typedef void (*PFN_SDCARD_DELAY) (UINT32);
typedef void (*PFN_SDCARD_WAIT_BUSY_CB) (void);

void SDCard_RegWaitBusyCallBackFunc(
	PFN_SDCARD_WAIT_BUSY_CB		pfnSDCardWaitBusyCB
);

ERRCODE 
SDCard_OpenEx(
	UINT32			u32SpiPortSelect,
	E_DRVSPI_DEVICE	eDeviceSelect,
	UINT32			u32SpiClock	
);

void SDCard_Close(void);

BOOL 
SDCard_GetCardSize(
	PUINT32 pu32TotCardSize
);

void SDCard_ISR_NBK(void);

E_SDCARD_STATE
SDCard_GetState_NBK(void);

ERRCODE 
SDCard_Read(
	UINT32 u32SecAddr, 
	UINT32 u32SecCnt, 
	PUINT8 pu8Buffer
);

ERRCODE 
SDCard_Read_NBK(
	UINT32 u32SecAddr, 
	UINT32 u32SecCnt, 
	PUINT8 pu8Buffer
);

ERRCODE 
SDCard_Write(
	UINT32 u32SecAddr,
	UINT32 u32SecCnt,
	PUINT8 pu8Buffer
);

ERRCODE 
SDCard_Write_NBK(
	UINT32 u32SecAddr,
	UINT32 u32SecCnt,
	PUINT8 pu8Buffer
);

UINT32 
SDCard_GetVersion(void);

E_SDCAR_TYPE 
SDCard_GetCardType(void);

BOOL
SDCard_WriteStart(
	UINT32 u32SecAddr
);

void SDCard_WriteStartToken(void);

void SDCard_WriteData(
	PUINT32 pu32Buffer
);

void SDCard_WriteCRC(void);

BOOL
SDCard_CheckWriteEndBusy(void);

BOOL
SDCard_WriteEnd(void);

BOOL
SDCard_ReadStart(
	UINT32 u32SecAddr
);

void SDCard_ReadStartToken(void);

BOOL
SDCard_ReadData(
	PUINT32 pu32Buffer
);

BOOL
SDCard_ReadEnd(void);

#ifdef  __cplusplus
}
#endif	// __cplusplus

#endif //__DRVSDCARD_H__




