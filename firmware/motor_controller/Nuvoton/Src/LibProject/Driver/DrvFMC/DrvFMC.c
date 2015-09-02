/********************************************************************
 *																	*
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.	*
 *																	*
 ********************************************************************/

#include <stdio.h>
#include "Driver/DrvFMC.h"

#define ISP_CMD_STANDBY			0x30 
#define ISP_CMD_READ_CID		0x0b
#define ISP_CMD_READ_DID		0x0c
#define ISP_CMD_ERASE_PAGE		0x22	// Erase   Flash page or User Configuration block 
#define ISP_CMD_PROGRAM			0x21	// Program Flash page or User Configuration block
#define ISP_CMD_READ_DATA		0x00	// Read    Flash page or User Configuration Block


#define _DRVFMC_CLEAR_FAIL_FLAG()		FMC.ISPCON.ISPFF = 1

/*---------------------------------------------------------------------------------------------------------*/
/* Function: DrvFMC_Write				                                               	   		   		   */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*               u32addr 	- [in]		Flash address include Config register                              */
/*               u32data 	- [in]		Data     				   										   */
/*                     		                           	   												   */
/*               									      		                           				   */
/* Returns:                                                                                                */
/*               0		 Success																		   */
/*              <0       Failed when illegal condition occurs					                  		   */
/*               							                                                               */
/* Description:                                                                                            */
/*               Write data into Flash include Config                                                      */
/*---------------------------------------------------------------------------------------------------------*/
ERRCODE 
DrvFMC_Write(
	UINT32 u32Addr, 
	UINT32 u32Data)
{
    FMC.ISPCMD.u32Reg = ISP_CMD_PROGRAM;
    FMC.ISPADR = u32Addr;
	FMC.ISPDAT = u32Data;
	FMC.ISPTRG.ISPGO = 1;	
	while(FMC.ISPTRG.ISPGO == 1);

	if (FMC.ISPCON.ISPFF)
	{
		_DRVFMC_CLEAR_FAIL_FLAG();
		return E_DRVFMC_ERR_ISP_FAIL;
	}
	return E_SUCCESS;
}

ERRCODE 
DrvFMC_WriteBuffer(
	UINT32  u32Addr, 
	PUINT32 pu32Buffer,
	UINT32  u32BufferSize)
{
	FMC.ISPCMD.u32Reg = ISP_CMD_PROGRAM;
	while( u32BufferSize ) 
	{ 
		FMC.ISPDAT = *pu32Buffer ++;
	    FMC.ISPADR = u32Addr;
		FMC.ISPTRG.ISPGO = 1;
		while(FMC.ISPTRG.ISPGO == 1);
		u32Addr += 4;
		u32BufferSize -= 4;		
	
		if (FMC.ISPCON.ISPFF)
		{
			_DRVFMC_CLEAR_FAIL_FLAG();
			return E_DRVFMC_ERR_ISP_FAIL;
		}
	}
	return E_SUCCESS;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function: DrvFMC_Read				                                               	   		   		   */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*               u32addr 	- [in]		Flash address include eDRVFMC_APROM, eDRVFMC_LDROM, Data Flash, and Config         */
/*               u32data 	- [out]		Data     				   										   */
/*                     		                           	   												   */
/*               									      		                           				   */
/* Returns:                                                                                                */
/*               0		 Success																		   */
/*              <0       Failed when illegal condition occurs					                  		   */
/*               							                                                               */
/* Description:                                                                                            */
/*               Read data from Flash include eDRVFMC_APROM, eDRVFMC_LDROM, Data Flash, and Config	                       */
/*---------------------------------------------------------------------------------------------------------*/
ERRCODE 
DrvFMC_Read(
	UINT32 u32Addr, 
	PUINT32 pu32Data)
{ 
    FMC.ISPCMD.u32Reg = ISP_CMD_READ_DATA;
    FMC.ISPADR = u32Addr;
	FMC.ISPTRG.ISPGO = 1;    
	while(FMC.ISPTRG.ISPGO == 1);

	if (FMC.ISPCON.ISPFF)
	{
		_DRVFMC_CLEAR_FAIL_FLAG();
		return E_DRVFMC_ERR_ISP_FAIL;
	}
	*pu32Data = FMC.ISPDAT;
	return E_SUCCESS;
}
 
ERRCODE 
DrvFMC_ReadBuffer(
	UINT32  u32Addr, 
	PUINT32 pu32Buffer,
	UINT32  u32BufferSize)
{
    FMC.ISPCMD.u32Reg = ISP_CMD_READ_DATA;
	while( u32BufferSize ) 
	{
	    FMC.ISPADR = u32Addr;
		FMC.ISPTRG.ISPGO = 1; 	
		while(FMC.ISPTRG.ISPGO == 1);

		if (FMC.ISPCON.ISPFF)
		{
			_DRVFMC_CLEAR_FAIL_FLAG();
			return E_DRVFMC_ERR_ISP_FAIL;
		}
		*pu32Buffer ++ = FMC.ISPDAT;
		u32Addr += 4;
		u32BufferSize -= 4;	
	}
	return E_SUCCESS;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function: DrvFMC_ErasePage				                                               	   		   		   */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*               u32addr 	- [in]		Flash page base address or Config0 address                         */
/*                     		                           	   												   */
/*               									      		                           				   */
/* Returns:                                                                                                */
/*               0		 Success																		   */
/*              <0       Failed when illegal condition occurs					                  		   */
/*               							                                                               */
/* Description:                                                                                            */
/*             Page Erase Flash include eDRVFMC_APROM, eDRVFMC_LDROM, Data Flash, and Config. The erase unit is 512 bytes  */
/*---------------------------------------------------------------------------------------------------------*/
ERRCODE 
DrvFMC_ErasePage(
	UINT32 u32Addr)
{
    FMC.ISPCMD.u32Reg = ISP_CMD_ERASE_PAGE;
    FMC.ISPADR = u32Addr;
	FMC.ISPTRG.ISPGO = 1;	
	while(FMC.ISPTRG.ISPGO == 1);

	if (FMC.ISPCON.ISPFF)
	{
		_DRVFMC_CLEAR_FAIL_FLAG();
		return E_DRVFMC_ERR_ISP_FAIL;
	}
	return E_SUCCESS;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function: DrvFMC_ReadCID				                                               	   		   		   */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*               u32data 	- [out]		Data     				   										   */
/*                     		                           	   												   */
/*               									      		                           				   */
/* Returns:                                                                                                */
/*               0		 Success																		   */
/*              <0       Failed when illegal condition occurs					                  		   */
/*               							                                                               */
/* Description:                                                                                            */
/*               Read Company ID                             	                       					   */
/*---------------------------------------------------------------------------------------------------------*/
ERRCODE 
DrvFMC_ReadCID(
	PUINT32 pu32Data)
{
    FMC.ISPCMD.u32Reg = ISP_CMD_READ_CID;
    FMC.ISPTRG.ISPGO = 1;
	while(FMC.ISPTRG.ISPGO == 1);
	if (FMC.ISPCON.ISPFF)
	{
		_DRVFMC_CLEAR_FAIL_FLAG();
		return E_DRVFMC_ERR_ISP_FAIL;
	} 
	*pu32Data = FMC.ISPDAT; 
    return E_SUCCESS;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function: DrvFMC_ReadDID				                                               	   		   		   */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*               u32data 	- [out]		Data     				   										   */
/*                     		                           	   												   */
/*               									      		                           				   */
/* Returns:                                                                                                */
/*               0		 Success																		   */
/*              <0       Failed when illegal condition occurs					                  		   */
/*               							                                                               */
/* Description:                                                                                            */
/*               Read Device ID                             	                       					   */
/*---------------------------------------------------------------------------------------------------------*/
ERRCODE 
DrvFMC_ReadDID(
	PUINT32 pu32Data)
{
	FMC.ISPCMD.u32Reg = ISP_CMD_READ_DID;
    FMC.ISPTRG.ISPGO = 1;
	while(FMC.ISPTRG.ISPGO == 1);
	if (FMC.ISPCON.ISPFF)
	{
		_DRVFMC_CLEAR_FAIL_FLAG();
		return E_DRVFMC_ERR_ISP_FAIL;
	} 
    
	*pu32Data = FMC.ISPDAT; 
    return E_SUCCESS;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function: DrvFMC_WriteConfig				                                               	  	   		   */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*               u32data0 	- [in]		Config0 data       												   */
/*                     		                           	   												   */
/*               									      		                           				   */
/* Returns:                                                                                                */
/*               0		 Success																		   */
/*              <0       Failed when illegal condition occurs					                  		   */
/*               							                                                               */
/* Description:                                                                                            */
/*               Erase Config and write data into Config0 and Config1	                       			   */
/*---------------------------------------------------------------------------------------------------------*/
ERRCODE 
DrvFMC_WriteConfig(
	UINT32 u32Data)
{		
	if (DrvFMC_ErasePage(DRVFMC_CONFIG0_ADDR) != E_SUCCESS)
		return E_DRVFMC_ERR_ISP_FAIL;
	
	return DrvFMC_Write(DRVFMC_CONFIG0_ADDR, u32Data);
}

ERRCODE 
DrvFMC_ReadConfig(
	PUINT32 pu32Data)
{
	return DrvFMC_Read(DRVFMC_CONFIG0_ADDR,pu32Data); 
}


/*---------------------------------------------------------------------------------------------------------*/
/* Function: DrvFMC_GetVersion                                                                             */
/*                                                                                                         */
/* Parameters:        	                                                                                   */
/*               None	                                                         						   */
/*                                                                                                         */
/* Returns:                                                                                                */
/*               The DrvFMC version number                                                                 */
/*                                                                                                         */
/* Description:                                                                                            */
/*               This function is used to get the DrvFMC version number                                    */
/*---------------------------------------------------------------------------------------------------------*/
UINT32 DrvFMC_GetVersion(void)
{
	return DRVFMC_VERSION_NUM;
}

