//---------------------------------------------------------------------------------------------------------
//
// Copyright(c) Nuvoton Technology Corp. All rights reserved.
//
//---------------------------------------------------------------------------------------------------------

#include <stdio.h>
#include "Driver/DrvSys.h"


//---------------------------------------------------------------------------------------------------------
// Global file scope (static) variables
//---------------------------------------------------------------------------------------------------------
// none

//---------------------------------------------------------------------------------------------------------
// Function: DrvSYS_GetVersion
//
// Parameters:
//               None
//
// Returns:
//               The DrvSYS_ version number
//
// Description:
//               This function is used to get the DrvSYS_ version number
//---------------------------------------------------------------------------------------------------------
UINT32
DrvSYS_GetVersion(void)
{
	return DRVSYS_VERSION_NUM;
}


//---------------------------------------------------------------------------------------------------------
// Function: DrvSYS_UnLockKeyReg
//
// Parameters:
//				None
//
// Returns:
//				None
// Description:
//				Unlock protected registers
//---------------------------------------------------------------------------------------------------------
void DrvSYS_UnlockKeyReg(void)
{
	SYS.REGLOCKADDR.u32Reg	= 0x59;
	SYS.REGLOCKADDR.u32Reg	= 0x16;
	SYS.REGLOCKADDR.u32Reg	= 0x88;
}

