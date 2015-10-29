/********************************************************************
 *																	*
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.	*
 *																	*
 ********************************************************************/

#ifndef _DRVFMC_H
#define _DRVFMC_H

// --------------------------------------------------------------
// Include Header Files
// --------------------------------------------------------------
#include "Platform.h"
#include "System/SysInfra.h"
#include "Driver/DrvCLK.h"
#include "Driver/DrvSYS.h"

#ifdef  __cplusplus
extern "C"
{
#endif

// --------------------------------------------------------------
// Version Definition and Error Code
// --------------------------------------------------------------
#define	DRVFMC_MAJOR_NUM 2
#define	DRVFMC_MINOR_NUM 00
#define	DRVFMC_BUILD_NUM 001
#define DRVFMC_VERSION_NUM	_SYSINFRA_VERSION(DRVFMC_MAJOR_NUM, DRVFMC_MINOR_NUM, DRVFMC_BUILD_NUM)

// #define Error Code
// E_DRVFMC_ERR_ISP_FAIL		ISP Failed when illegal condition occurs
#define E_DRVFMC_ERR_ISP_FAIL	_SYSINFRA_ERRCODE(TRUE, MODULE_ID_DRVFMC, 1)

// --------------------------------------------------------------
// Define Parameter
// --------------------------------------------------------------
//#define CONFIG0	0x00300000	//0xA5A5A5A5
//#define CONFIG1	0x00300004	//0x5A5A5A5A
#define DRVFMC_CONFIG0_ADDR		0x00300000
// Write parameters for Config0 
#define DRVFMC_CONFIG0_CWDTEN(on)	((on==1)?BIT31:0)//Watchdog enable/disable after power on
#define DRVFMC_CONFIG0_CVDEN(on)	((on==1)?BIT23:0)//Voltage detector enable/disable after power on
#define DRVFMC_CONFIG0_CVDTV(on)	((on==1)?BIT21:0)//Voltage detector threshold 0:2.7V 1:3.0V
#define DRVFMC_CONFIG0_LOCK(on)		((on==1)?BIT1:0)//Security Lock 0: flash data are locked  1: Flash data are not locked.
#if defined (__N572F064__)||defined (__N572F065__)
#define DRVFMC_CONFIG0_CKF(on)      ((on==1)?BIT28:0)//Clock filter enable/disable
#define DRVFMC_CONFIG0_CFOSC_EX		0x01000000		//Clock source 12M/6M after power on   
#define DRVFMC_CONFIG0_CFOSC_RC 	0x07000000		//Clock source RC_24M after power on
#else
#define DRVFMC_CONFIG0_PRTB(on)		((on==1)?BIT2:0)//Protection On 8K Bytes Flash 0:64k 1:72k
//Coarse timing control for SPI0 data receiving after power on.
#define DRVFMC_CONFIG0_SPIOCT0		0x00000000		// 00: receiving data clock is same as the SPI0_CLK.
#define DRVFMC_CONFIG0_SPIOCT1		0x000000c0		// 11: receiving data clock is delayed 1 half SPI0_CLK clock cycle.
#define DRVFMC_CONFIG0_SPIOCT2		0x00000040		// 01: receiving data clock is delayed 2 half SPI0_CLK clock cycle.
#define DRVFMC_CONFIG0_SPIOCT3		0x00000080		// 10: receiving data clock is delayed 3 half SPI0_CLK clock cycle.
//Fine timing control for SPI0 data receiving after power on.
#define DRVFMC_CONFIG0_SPIOFT75		0x00000000		// 00: receiving data clock has extra 7.5nS delay.
#define DRVFMC_CONFIG0_SPIOFT50		0x00000010		// 01: receiving data clock has extra 5.0nS delay. 
#define DRVFMC_CONFIG0_SPIOFT25		0x00000020		// 10: receiving data clock has extra 2.5nS delay. 
#define DRVFMC_CONFIG0_SPIOFT0		0x00000030		// 11: receiving data clock has no extra delay. 
#endif

typedef enum{
  	eDRVFMC_ERASE_TIME_20MS = 0,
	eDRVFMC_ERASE_TIME_25MS,
	eDRVFMC_ERASE_TIME_30MS,
	eDRVFMC_ERASE_TIME_35MS,
	eDRVFMC_ERASE_TIME_3MS,
	eDRVFMC_ERASE_TIME_5MS,
	eDRVFMC_ERASE_TIME_10MS,
	eDRVFMC_ERASE_TIME_15MS, 
} E_DRVFMC_FLASH_ERASE_TIME;

typedef enum{
  	eDRVFMC_PROGRAM_TIME_40US = 0,
	eDRVFMC_PROGRAM_TIME_45US,
	eDRVFMC_PROGRAM_TIME_50US,
	eDRVFMC_PROGRAM_TIME_55US,
	eDRVFMC_PROGRAM_TIME_20US,
	eDRVFMC_PROGRAM_TIME_25US,
	eDRVFMC_PROGRAM_TIME_30US,
    eDRVFMC_PROGRAM_TIME_35US 	
} E_DRVFMC_FLASH_PROGRAM_TIME;

#if defined (__N572F064__)||defined (__N572F065__)
typedef enum{
	eDRVFMC_ACCESS_TIME_WINDOW_40NS = 0,
	eDRVFMC_ACCESS_TIME_WINDOW_50NS,
	eDRVFMC_ACCESS_TIME_WINDOW_60NS,
	eDRVFMC_ACCESS_TIME_WINDOW_70NS,
	eDRVFMC_ACCESS_TIME_WINDOW_80NS,
	eDRVFMC_ACCESS_TIME_WINDOW_90NS,
	eDRVFMC_ACCESS_TIME_WINDOW_100NS
} E_DRVFMC_FLASH_ACCESS_TIME;
#endif

// --------------------------------------------------------------
// Function Prototypes 
// --------------------------------------------------------------

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvFMC_Close
//
// Description:
//				Disable IP clock.
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvFMC_Close(void)
{
	DrvCLK_DisableAhbDeviceClock(DRVCLK_CLK_ISP);
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvFMC_DisableIspControl_P
//
// Description:
//				Disable ISP control.
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvFMC_DisableIspControl_P(void)
{
	FMC.ISPCON.u32Reg &= ~(0x1f);
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvFMC_DisablePowerSaving
//
// Description:
//				Disable flash power saving function.
/*---------------------------------------------------------------------------------------------------------*/
#if defined (__N572F064__)||defined (__N572F065__)
static __inline
void DrvFMC_DisablePowerSaving(void)
{
	FMC.FATCON.FPSEN = 0;
}
#endif
/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvFMC_EnableIspControl_P
//
// Parameter:  
//				u8FrashEraseTime			[in] flash erase time
//												  Please refer to E_DRVFMC_FLASH_ERASE_TIME	 
//				u8FlashProgramTime			[in] flash program time
//												  Please refer to E_DRVFMC_FLASH_PROGRAM_TIME	
//				u8FlashAccessTimeWindow		[in] flash access time window
//												  Please refer to E_DRVFMC_FLASH_ACCESS_TIME
//				bEnableUpdateConfig0		[in] 1: enable update config0, 0: disable update config0
//
// Description:
//				Enable ISP control.
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void
DrvFMC_EnableIspControl_P(
	E_DRVFMC_FLASH_ERASE_TIME eFrashEraseTime,
	E_DRVFMC_FLASH_PROGRAM_TIME eFlashProgramTime,
#if defined (__N572F064__)||defined (__N572F065__)
	E_DRVFMC_FLASH_ACCESS_TIME eFlashAccessTimeWindow,
#endif
	BOOL bEnableUpdateConfig0)
{
	FMC.ISPCON.u32Reg = (eFrashEraseTime << 12) + (eFlashProgramTime << 8) + 0x43 + (bEnableUpdateConfig0<<4);
#if defined (__N572F064__)||defined (__N572F065__)
	FMC.FATCON.FATS = eFlashAccessTimeWindow;
#endif
}



/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvFMC_EnablePowerSaving
//
// Description:
//				Enable flash power saving function.
/*---------------------------------------------------------------------------------------------------------*/
#if defined (__N572F064__)||defined (__N572F065__)
static __inline
void DrvFMC_EnablePowerSaving(void)
{
	FMC.FATCON.FPSEN = 1;
}
#endif
/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvFMC_ErasePage
// Parameter:  
//				u32Addr  		[in]	Address where to be erased.
// Return:
//				0		Success
//				0>		Failed when illegal condition occurs
// Description:
//				Erase Flash page.
/*---------------------------------------------------------------------------------------------------------*/
ERRCODE
DrvFMC_ErasePage(
	UINT32	u32Addr
);

/*---------------------------------------------------------------------------------------------------------*/
// Get FMC driver version
/*---------------------------------------------------------------------------------------------------------*/
UINT32
DrvFMC_GetVersion(void);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvFMC_Open
//
// Description:
//				Enable IP clock.
/*---------------------------------------------------------------------------------------------------------*/
static __inline
void 
DrvFMC_Open(void)
{
	DrvCLK_EnableAhbDeviceClock(DRVCLK_CLK_ISP);
}

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvFMC_Read
// Parameter:  
//				u32Addr  		[in]	Address where to be read.
//				pu32Data			[in]	Read out data.
// Return:
//				0		Success
//				0>		Failed when illegal condition occurs
// Description:
//				Read data from Flash.
/*---------------------------------------------------------------------------------------------------------*/
ERRCODE
DrvFMC_Read(
	UINT32	u32Addr,
	PUINT32	pu32Data
);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvFMC_ReadBuffer
// Parameter:  
//				u32Addr  			[in]	Address where to be read.
//				pu32Data			[in]	Read out data.
//				u32BufferSize		[in]	Buffer size.
// Return:
//				0		Success
//				0>		Failed when illegal condition occurs
// Description:
//				Read data to buffer from Flash.
/*---------------------------------------------------------------------------------------------------------*/
ERRCODE
DrvFMC_ReadBuffer(
	UINT32  u32Addr, 
	PUINT32 pu32Buffer,
	UINT32  u32BufferSize
);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvFMC_ReadCID
// Parameter:  
//				pu32Data			[in]	Address where to put the read out CID.
// Return:
//				0		Success
//				0>		Failed when illegal condition occurs
// Description:
//				Read the Company ID.
/*---------------------------------------------------------------------------------------------------------*/
ERRCODE
DrvFMC_ReadCID(
	PUINT32	pu32Data
);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvFMC_ReadDID
// Parameter:  
//				pu32Data			[in]	Address where to put the read out DID.
// Return:
//				0		Success
//				0>		Failed when illegal condition occurs
// Description:
//				Read the Device ID.
/*---------------------------------------------------------------------------------------------------------*/
ERRCODE
DrvFMC_ReadDID(
	PUINT32	pu32Data
);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvFMC_WriteConfig
// Parameter:  
//				u32Data0  		[in]	Config0 data
// Return:
//				0		Success
//				0>		Failed when illegal condition occurs
// Description:
//				Erase Config and write data into Config0.
/*---------------------------------------------------------------------------------------------------------*/
ERRCODE
DrvFMC_WriteConfig(
	UINT32	u32Data0
);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvFMC_ReadConfig
// Parameter:  
//				pu32Data  		[out]	Config0 data
// Return:
//				0		Success
//				0>		Failed when illegal condition occurs
// Description:
//				Read data from Config0.
/*---------------------------------------------------------------------------------------------------------*/
ERRCODE 
DrvFMC_ReadConfig(
	PUINT32 pu32Data
);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvFMC_Write
// Parameter:  
//				u32Addr  		[in]	Address where to be written.
//				u32Data			[in]	Data to be written down.
// Return:
//				0		Success
//				0>		Failed when illegal condition occurs.
// Description:
//				Write data into Flash.
/*---------------------------------------------------------------------------------------------------------*/
ERRCODE
DrvFMC_Write(
	UINT32	u32Addr,
	UINT32	u32Data
);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	DrvFMC_ReadBuffer
// Parameter:  
//				u32Addr  		[in]	Address where to be written.
//				u32Data			[in]	Data to be written down.
//	            u32BufferSize	[in]	buffer size.	
// Return:
//				0		Success
//				0>		Failed when illegal condition occurs
// Description:
//				Write data of buffer into Flash.
/*---------------------------------------------------------------------------------------------------------*/
ERRCODE 
DrvFMC_WriteBuffer(
	UINT32  u32Addr, 
	PUINT32 pu32Buffer,
	UINT32  u32BufferSize
);

#ifdef  __cplusplus
}
#endif

#endif	// __DRVFMC_H__


