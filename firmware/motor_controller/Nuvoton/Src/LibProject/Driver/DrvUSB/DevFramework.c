/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.  *
 *                                                              *
 ****************************************************************/

#include "Platform.h"

#include "Driver/DrvUSB.h"
#include "DevFramework.h"


void DrvUsb_EnableUsb(
	S_DRVUSB_DEVICE *psDevice
)
{
	_DRVUSB_ENABLE_USB(); // enable USB & PHY
}

void DrvUsb_DisableUsb(
	S_DRVUSB_DEVICE *psDevice
)
{
	_DRVUSB_DISABLE_USB();// disable USB & PHY
}

void DrvUsb_PreDispatchWakeupEvent(
    S_DRVUSB_DEVICE *psDevice
)
{
	//non-use
}

void DrvUsb_PreDispatchFdtEvent(
    S_DRVUSB_DEVICE *psDevice
)
{
	// Clear all pending events on USB attach/detach to
	// handle the scenario that the time sequence of event happening
	// is different from that of event handling.

	UINT32 u32FLODET = psDevice->u32FLODET;
	if (u32FLODET & 1)
	{
		// attached
		if (psDevice->eUsbState == eDRVUSB_USB_STATE_DETACHED)
		{
			psDevice->eUsbState = eDRVUSB_USB_STATE_ATTACHED;
			// FIXME
			DrvUsb_EnableUsb(psDevice);
		}

		psDevice->u32MiscEventFlags |= DRVUSB_EVENT_FLAG_ATTACH;
	}
	else
	{
		// detached
		psDevice->eUsbState = eDRVUSB_USB_STATE_DETACHED;
		DrvUsb_DisableUsb(psDevice);

		psDevice->u32MiscEventFlags |= DRVUSB_EVENT_FLAG_DETACH;

	}
}

void DrvUsb_PreDispatchBusEvent(
    S_DRVUSB_DEVICE *psDevice
)
{
	UINT32 u32Attr = psDevice->u32ATTR;

	if (psDevice->eUsbState == eDRVUSB_USB_STATE_DETACHED)
	{
		// Clear all pending events on USB attach/detach to
		// handle the scenario that the time sequence of event happening
		// is different from that of event handling.
		return;
	}

	if (u32Attr & USBD_ATTR_USBRST)
	{
		// reset
		_DRVUSB_ENABLE_USB(); // enable PHY
		psDevice->eUsbState = eDRVUSB_USB_STATE_DEFAULT;
		psDevice->u32MiscEventFlags |= DRVUSB_EVENT_FLAG_BUS_RESET;
	}
	else if (u32Attr & USBD_ATTR_SUSPEND)
	{
		// suspend
		_DRVUSB_DISABLE_PHY(); // disable PHY
		if (psDevice->eUsbState >= eDRVUSB_USB_STATE_ATTACHED)
		{
			psDevice->eUsbState |= eDRVUSB_USB_STATE_SUSPENDED;
		}
		psDevice->u32MiscEventFlags |= DRVUSB_EVENT_FLAG_BUS_SUSPEND;

	}
	else if (u32Attr & USBD_ATTR_RESUME)
	{
		// resume
		_DRVUSB_ENABLE_USB(); // enable PHY
		if (psDevice->eUsbState >= eDRVUSB_USB_STATE_ATTACHED)
		{
			psDevice->eUsbState &= ~eDRVUSB_USB_STATE_SUSPENDED;
		}
		psDevice->u32MiscEventFlags |= DRVUSB_EVENT_FLAG_BUS_RESUME;
	}
	//else if (u32Attr & ATTR_Timeout)
	//;	// time-out
}

void DrvUsb_PreDispatchEPEvent(
    S_DRVUSB_DEVICE *psDevice
)
{
	UINT32 u32EPNum;
	UINT32 u32EPSettingNum;
	UINT32 u32EVF = psDevice->u32EVF;
	UINT32 u32STS = psDevice->u32STS;
	UINT32 u32PacketType;
	UINT32 u32ISOPacketType;
	
	if (psDevice->eUsbState == eDRVUSB_USB_STATE_DETACHED)
	{
		// Clear all pending events on USB attach/detach to
		// handle the scenario that the time sequence of event happening
		// is different from that of event handling.
		
		return;
	}

	// Only care EP events and Setup event
	u32EVF &= (USBD_EVF_EPTF0 | USBD_EVF_EPTF1 | USBD_EVF_EPTF2 | USBD_EVF_EPTF3 | USBD_EVF_EPTF4 | USBD_EVF_EPTF5 | USBD_EVF_SETUP);

	if (u32EVF & USBD_EVF_SETUP)
	{
		psDevice->u32MiscEventFlags |= DRVUSB_EVENT_FLAG_SETUP;
		u32EVF &= ~USBD_EVF_SETUP;
		
		_DRVUSB_SET_EVF(USBD_EVF_SETUP);
	}

	while (1)
	{
		u32EPSettingNum = SysInfra_CountTrailingZero(u32EVF);
		if (u32EPSettingNum >= 32)
		{
			break;
		}
		// Clear this EP event as pre-handled.
		u32EVF &= ~(1 << u32EPSettingNum);
		
		_DRVUSB_SET_EVF(1 << u32EPSettingNum);

		u32EPSettingNum -= 16; // 0 ~ 5

		u32EPNum = (_DRVUSB_GET_CFG(u32EPSettingNum) & USBD_CFG_EPT);		
		
		u32PacketType = ((u32STS >> (4 + u32EPSettingNum * 3)) & USBD_STS_STS);
		if (u32PacketType == STS_IN_ACK)
		{
			// Set this EP event as non-handled.
			if ( _DRVUSB_GET_CFG(u32EPSettingNum) & BIT4 )  // Iso IN.
			{
			    psDevice->u32EPEventFlags = 0x4000;
			}
			else
			{
			    psDevice->u32EPEventFlags |= (1 << u32EPNum);
			}    
		}
		else if (u32PacketType == STS_OUT0_ACK)
		{
			// Set this EP event as non-handled.
			psDevice->u32EPEventFlags |= (1 << (u32EPNum + MAX_EP_NUM));
			psDevice->abData0Out[u32EPNum] = TRUE;
		}
		else if (u32PacketType == STS_OUT1_ACK)
		{
			// Set this EP event as non-handled.
			psDevice->u32EPEventFlags |= (1 << (u32EPNum + MAX_EP_NUM));
			psDevice->abData0Out[u32EPNum] = FALSE;
		}
		else if (u32PacketType == STS_ISO)
		{
			u32ISOPacketType = (_DRVUSB_GET_CFG(u32EPSettingNum) & USBD_CFG_STATE);
			if (u32ISOPacketType == 0x20)	// Out endpoint
			{
				//psDevice->abData0Out[u32EPNum] = TRUE;
				// Set this EP event as non-handled.
				psDevice->u32EPEventFlags |= (1 << u32EPNum);//1205 modified remove (u32EPNum + 16)
			}
			else if (u32ISOPacketType == 0x40)	// In endpoint
			{
				//psDevice->abData0Out[u32EPNum] = FALSE;
				// Set this EP event as non-handled.
				psDevice->u32EPEventFlags |= (1 << u32EPNum);
			}
		}
	}
	return;
}

void DrvUsb_DispatchWakeupEvent(
    S_DRVUSB_DEVICE *psDevice
)
{
	//non-use
}

void DrvUsb_DispatchMiscEvent(
    S_DRVUSB_DEVICE *psDevice
)
{
	UINT32 u32EventId = 0;
	UINT32 u32MiscIntFlag = _DRVUSB_GET_MISC_INT();		
	while (1)
	{
		u32EventId = SysInfra_CountTrailingZero(psDevice->u32MiscEventFlags);
		if (u32EventId >= MAX_MISC_EVENT_NUM)
		{
			break;
		}
// TODO: lock usb interrupt to sync data between ISR an main routine
//		_DRVUSB_DISABLE_MISC_INT(u32MiscIntFlag);
//		NVIC_DisableIRQ(USBD_IRQn);
		psDevice->u32MiscEventFlags &= ~(1 << u32EventId);
//		_DRVUSB_ENABLE_MISC_INT(u32MiscIntFlag);
//		NVIC_EnableIRQ(USBD_IRQn);
		// Call the corresponding event handler.
		if (psDevice->apfnMiscCallback[u32EventId] != 0)
		{
			psDevice->apfnMiscCallback[u32EventId](psDevice->apMiscCallbackArgu[u32EventId]);
		}
	}
}

void DrvUsb_DispatchEPEvent(
    S_DRVUSB_DEVICE *psDevice
)
{
	UINT32 u32EventId = 0;

	while (1)
	{
		u32EventId = SysInfra_CountTrailingZero(psDevice->u32EPEventFlags);
		if (u32EventId >= MAX_EP_NUM*2)
		{
			break;
		}
// TODO: lock usb interrupt to sync data between ISR an main routine
//		_DRVUSB_DISABLE_USB_INT();
//		NVIC_DisableIRQ(USBD_IRQn);
		psDevice->u32EPEventFlags &= ~(1 << u32EventId);
//		_DRVUSB_ENABLE_USB_INT();
//		NVIC_EnableIRQ(USBD_IRQn);
		
		// Call the corresponding event handler.
		if (psDevice->apfnEPCallback[u32EventId] != 0)
		{
			psDevice->apfnEPCallback[u32EventId](psDevice->apEPCallbackArgu[u32EventId]);
		}
	}
}












