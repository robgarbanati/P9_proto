/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.  *
 *                                                              *
 ****************************************************************/

#include "Platform.h"
#include "System/SysInfra.h"

#include "Driver/DrvUSB.h"
#include "DevFramework.h"
#include "CtrlFramework.h"

void DrvUsb_CtrlDataInDefault(PVOID pVoid)
{
	if (g_DrvUsb_sCtrl.au8SetupData[0] & 0x80)
	{
		_DRVUSB_TRIG_EP(g_DrvUsb_sCtrl.sCtrlEntry.u8CtrlDataOutEP, 0x00);
	}
}

void DrvUsb_CtrlDataOutDefault(PVOID pVoid)
{
	if ((g_DrvUsb_sCtrl.au8SetupData[0] & 0x80) == 0)
	{
//		_DRVUSB_TRIG_EP(g_DrvUsb_sCtrl.u8CtrlDataInEP, 0x00);

		_DRVUSB_SET_EP_TOG_BIT(g_DrvUsb_sCtrl.sCtrlEntry.u8CtrlDataInEP,FALSE);
		//_DRVUSB_TRIG_EP0(u32TrigSize);
		_DRVUSB_TRIG_EP(g_DrvUsb_sCtrl.sCtrlEntry.u8CtrlDataInEP,0x00);

	}
}


