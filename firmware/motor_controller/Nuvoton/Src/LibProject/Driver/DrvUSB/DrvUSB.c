#include "Platform.h"
#include "Driver/DrvUSB.h"

#include "DevFramework.h"
#include "CtrlFramework.h"

#if defined ( __CC_ARM   )
__align(4) S_DRVUSB_DEVICE g_DrvUsb_sDevice;
__align(4) S_DRVUSB_CTRL	g_DrvUsb_sCtrl; 
#else
S_DRVUSB_DEVICE g_DrvUsb_sDevice;
S_DRVUSB_CTRL	g_DrvUsb_sCtrl; 
#endif

//======================================================
static void RoughDelay(UINT32 u32Time)
{
    volatile INT32 i32Delay;

    i32Delay = u32Time;

    while(i32Delay-- >= 0);
}

UINT32
DrvUsb_GetVersion(void)
{
	return (DRVUSB_MAJOR_NUM << 16) | (DRVUSB_MINOR_NUM << 8) | DRVUSB_BUILD_NUM;
}

//======================================================
ERRCODE
DrvUsb_Open_Ex(PVOID pVoid)
{
 	/* Enable USB device clock */
    CLK.APBCLK.USBD_EN = 1; 

	/* Reset USB */
	SYS.IPRSTC2.USBD_RST = 1;
	RoughDelay(500);
	SYS.IPRSTC2.USBD_RST = 0;

	_DRVUSB_ENABLE_USB();
	_DRVUSB_ENABLE_SE0();	
	RoughDelay(100);
	_DRVUSB_DISABLE_SE0();

	return E_SUCCESS;
}


void
DrvUsb_Close_Ex(void)
{
	_DRVUSB_DISABLE_USB();
 	/* Disable USB device clock */
    CLK.APBCLK.USBD_EN = 0; 
}


ERRCODE
DrvUsb_Open(PVOID pVoid)
{
	ERRCODE errCode;
	UINT32 i;
	errCode = DrvUsb_Open_Ex(pVoid);
	if(errCode != E_SUCCESS)
		return errCode;

	g_DrvUsb_sDevice.u32MiscEventFlags = 0;
	g_DrvUsb_sDevice.u32EPEventFlags = 0;
	for (i = 0; i < MAX_MISC_EVENT_NUM; i ++)
	{
		g_DrvUsb_sDevice.apfnMiscCallback[i] = 0;
		g_DrvUsb_sDevice.apMiscCallbackArgu[i] = 0;
		
	}
	for(i=0;i<MAX_EP_NUM*2;i++) //MAX_EP_NUM*2
	{
		g_DrvUsb_sDevice.apfnEPCallback[i] = 0;
		g_DrvUsb_sDevice.apEPCallbackArgu[i] = 0;
	}


	g_DrvUsb_sDevice.eUsbState = eDRVUSB_USB_STATE_DETACHED;
	g_DrvUsb_sDevice.u32FLODET = _DRVUSB_GET_FLD_STS();

	g_DrvUsb_sCtrl.u32SetupBufAddr = USBD_SRAM;

	g_DrvUsb_sCtrl.sCtrlEntry.pasCtrlCallbackEntry = 0;
	g_DrvUsb_sCtrl.sCtrlEntry.u32MaxCtrlCallbackReg = 0;
	g_DrvUsb_sCtrl.sCtrlEntry.u8CtrlDataInEP = 0;
	g_DrvUsb_sCtrl.sCtrlEntry.u8CtrlDataOutEP = 0;


	DrvUsb_PreDispatchFdtEvent(&g_DrvUsb_sDevice);
	DrvUsb_DispatchMiscEvent(&g_DrvUsb_sDevice);
	return E_SUCCESS;
}


void DrvUsb_Close(void)
{
	DrvUsb_Close_Ex();
}


ERRCODE
DrvUsb_RegisterUsbAttachCallback(
    PFN_DRVUSB_ATTACH_CALLBACK pfnUsbAttachCallback,
    PVOID pUsbAttachCallbackArgu
)
{
	UINT32 u32EventId = SysInfra_CountTrailingZero(DRVUSB_EVENT_FLAG_ATTACH);

	g_DrvUsb_sDevice.apfnMiscCallback[u32EventId] = pfnUsbAttachCallback;
	g_DrvUsb_sDevice.apMiscCallbackArgu[u32EventId] = pUsbAttachCallbackArgu;

	return E_SUCCESS;
}

ERRCODE
DrvUsb_RegisterUsbDetachCallback(
    PFN_DRVUSB_DETACH_CALLBACK pfnUsbDetachCallback,
    PVOID pUsbDetachCallbackArgu
)
{
	UINT32 u32EventId = SysInfra_CountTrailingZero(DRVUSB_EVENT_FLAG_DETACH);

	g_DrvUsb_sDevice.apfnMiscCallback[u32EventId] = pfnUsbDetachCallback;
	g_DrvUsb_sDevice.apMiscCallbackArgu[u32EventId] = pUsbDetachCallbackArgu;

	return E_SUCCESS;
}

ERRCODE
DrvUsb_RegisterUsbBusResetCallback(
    PFN_DRVUSB_BUS_RESET_CALLBACK pfnUsbBusResetCallback,
    PVOID pUsbBusResetCallbackArgu
)
{
	UINT32 u32EventId = SysInfra_CountTrailingZero(DRVUSB_EVENT_FLAG_BUS_RESET);

	g_DrvUsb_sDevice.apfnMiscCallback[u32EventId] = pfnUsbBusResetCallback;
	g_DrvUsb_sDevice.apMiscCallbackArgu[u32EventId] = pUsbBusResetCallbackArgu;

	return E_SUCCESS;
}

ERRCODE
DrvUsb_RegisterUsbBusSuspendCallback(
    PFN_DRVUSB_BUS_SUSPEND_CALLBACK pfnUsbBusSuspendCallback,
    PVOID pUsbBusSuspendCallbackArgu
)
{
	UINT32 u32EventId = SysInfra_CountTrailingZero(DRVUSB_EVENT_FLAG_BUS_SUSPEND);

	g_DrvUsb_sDevice.apfnMiscCallback[u32EventId] = pfnUsbBusSuspendCallback;
	g_DrvUsb_sDevice.apMiscCallbackArgu[u32EventId] = pUsbBusSuspendCallbackArgu;

	return E_SUCCESS;
}

ERRCODE
DrvUsb_RegisterUsbBusResumeCallback(
    PFN_DRVUSB_BUS_RESUME_CALLBACK pfnUsbBusResumeCallback,
    PVOID pUsbBusResumeCallbackArgu
)
{
	UINT32 u32EventId = SysInfra_CountTrailingZero(DRVUSB_EVENT_FLAG_BUS_RESUME);

	g_DrvUsb_sDevice.apfnMiscCallback[u32EventId] = pfnUsbBusResumeCallback;
	g_DrvUsb_sDevice.apMiscCallbackArgu[u32EventId] = pUsbBusResumeCallbackArgu;

	return E_SUCCESS;
}

ERRCODE
DrvUsb_RegisterUsbSetupCallback(
    PFN_DRVUSB_SETUP_CALLBACK pfnUsbSetupCallback,
    PVOID pUsbSetupCallbackArgu
)
{
	UINT32 u32EventId = SysInfra_CountTrailingZero(DRVUSB_EVENT_FLAG_SETUP);

	g_DrvUsb_sDevice.apfnMiscCallback[u32EventId] = pfnUsbSetupCallback;
	g_DrvUsb_sDevice.apMiscCallbackArgu[u32EventId] = pUsbSetupCallbackArgu;

	return E_SUCCESS;
}

ERRCODE
DrvUsb_RegisterUsbEPInCallback(
    UINT32 u32EPNum,
    PFN_DRVUSB_EP_IN_CALLBACK pfnUsbEPInCallback,
    PVOID pUsbEPInCallbackArgu
)
{
	if (u32EPNum >= MAX_EP_NUM)
	{
		return E_DRVUSB_INVALID_EP_NUM;
	}

	g_DrvUsb_sDevice.apfnEPCallback[u32EPNum] = pfnUsbEPInCallback;
	g_DrvUsb_sDevice.apEPCallbackArgu[u32EPNum] = pUsbEPInCallbackArgu;

	return E_SUCCESS;
}

ERRCODE
DrvUsb_RegisterUsbEPOutCallback(
    UINT32 u32EPNum,
    PFN_DRVUSB_EP_OUT_CALLBACK pfnUsbEPOutCallback,
    PVOID pUsbEPOutCallbackArgu
)
{
	if (u32EPNum >= MAX_EP_NUM)
	{
		return E_DRVUSB_INVALID_EP_NUM;
	}

	g_DrvUsb_sDevice.apfnEPCallback[u32EPNum + MAX_EP_NUM] = pfnUsbEPOutCallback;
	g_DrvUsb_sDevice.apEPCallbackArgu[u32EPNum + MAX_EP_NUM] = pUsbEPOutCallbackArgu;

	return E_SUCCESS;
}

void DrvUsb_PreDispatchEvent(void)
{
	g_DrvUsb_sDevice.u32EVF = _DRVUSB_GET_EVF();

	// [CCLi8] (2007.1.6)
	// In W55PA71 Ver.B, read FLODET, ATTR, and STS won't clear corresponding
	// events. This is different from W55U02 and W55PA71 Ver.A.
	if (g_DrvUsb_sDevice.u32EVF & USBD_IEF_WAKEUP)
	{
		// Clear wakeup event.
		_DRVUSB_SET_EVF(USBD_EVF_WAKEUP);
		// Pre-dispatch wakeup event.
		DrvUsb_PreDispatchWakeupEvent(&g_DrvUsb_sDevice);
	}
	else if (g_DrvUsb_sDevice.u32EVF & USBD_IEF_FLD)
	{
		g_DrvUsb_sDevice.u32FLODET = _DRVUSB_GET_FLD_STS();
		// Clear float-detection event.		
		_DRVUSB_SET_EVF(USBD_EVF_FLD);
		// Pre-dispatch float-detection event.
		DrvUsb_PreDispatchFdtEvent(&g_DrvUsb_sDevice);
	}
	else if (g_DrvUsb_sDevice.u32EVF & USBD_IEF_BUS)
	{
		g_DrvUsb_sDevice.u32ATTR = _DRVUSB_GET_BUS_ATTR();
		// Clear bus event.		
		_DRVUSB_SET_EVF(USBD_EVF_BUS);
		// Pre-dispatch bus event.
		DrvUsb_PreDispatchBusEvent(&g_DrvUsb_sDevice);
	}
	else if (g_DrvUsb_sDevice.u32EVF & USBD_IEF_USB)
	{
		g_DrvUsb_sDevice.u32STS = _DRVUSB_GET_STS();
		// Clear USB event.
		
		// Clear USB events individually instead of in total.
		// Otherwise, incoming USB events may be cleared mistakenly.
		// Pre-dispatch USB event.
		DrvUsb_PreDispatchEPEvent(&g_DrvUsb_sDevice);
	}
}

void DrvUsb_DispatchEvent(void)
{
	DrvUsb_DispatchMiscEvent(&g_DrvUsb_sDevice);
	DrvUsb_DispatchEPEvent(&g_DrvUsb_sDevice);
}

// DATA0 or DATA1 Out transaction when Data Out ACK happens.
BOOL
DrvUsb_IsData0Out(
    UINT32 u32EPNum
)
{
//	INT32 i32Ret = E_SUCCESS;
	BOOL bData0Out = FALSE;

//	if (u32EPNum >= MAX_EP_NUM)
//	{
//		i32Ret = E_DRVUSB_INVALID_EP_NUM;
//	}
//	else
//	{
		bData0Out = g_DrvUsb_sDevice.abData0Out[u32EPNum];
//	}

//	SysInfra_SetLastErrorCode(i32Ret);
	return bData0Out;
}

// Get USB device state.
E_DRVUSB_USB_STATE
DrvUsb_GetUsbState(void)
{
	return (E_DRVUSB_USB_STATE)g_DrvUsb_sDevice.eUsbState;
}

// Set USB device state.
void DrvUsb_SetUsbState(
	E_DRVUSB_USB_STATE eUsbState
)
{
	g_DrvUsb_sDevice.eUsbState = eUsbState;
}
//TODO: reduce table operation
ERRCODE
DrvUsb_InitCtrlHandler(
	UINT32 u32SetupBufAddr,
	UINT8 u8CtrlDataInEP,
	UINT8 u8CtrlDataOutEP
)
{
	
	g_DrvUsb_sCtrl.sCtrlEntry.pasCtrlCallbackEntry = 0;
	g_DrvUsb_sCtrl.sCtrlEntry.u32MaxCtrlCallbackReg = 0;
	g_DrvUsb_sCtrl.sCtrlEntry.u8CtrlDataInEP = u8CtrlDataInEP;
	g_DrvUsb_sCtrl.sCtrlEntry.u8CtrlDataOutEP = u8CtrlDataOutEP;
	g_DrvUsb_sCtrl.u32SetupBufAddr = u32SetupBufAddr;
	_DRVUSB_SET_SETUP_BUF(u32SetupBufAddr);

	return E_SUCCESS;
}
//TODO: reduce table process
ERRCODE
DrvUsb_RegisterCtrHandler(
    S_DRVUSB_CTRL_CALLBACK_ENTRY *pasCtrlCallbackEntry,
    UINT32 u32RegCnt
)
{
	if (u32RegCnt == 0)
	{
		return E_SUCCESS;
	}
	if (pasCtrlCallbackEntry == 0)
	{
		return E_DRVUSB_NULL_POINTER;
	}
	g_DrvUsb_sCtrl.sCtrlEntry.u32MaxCtrlCallbackReg = u32RegCnt;
	g_DrvUsb_sCtrl.sCtrlEntry.pasCtrlCallbackEntry = pasCtrlCallbackEntry;
	return E_SUCCESS;
}


void DrvUsb_CtrlSetupAck(
    PVOID pArgu
)
{
	UINT32 i;
	E_DRVUSB_USB_STATE eUsbState;
	S_DRVUSB_CTRL_CALLBACK_ENTRY *psEntryRet = 0;
	
	

	_DRVUSB_CLR_CTRL_READY();

	// check if after DEFAULT state (RESET)
	eUsbState = DrvUsb_GetUsbState();
	if (eUsbState < eDRVUSB_USB_STATE_DEFAULT)
	{
		_DRVUSB_CLR_CTRL_READY_AND_TRIG_STALL();
		return;
	}
	
	// read request
	for (i = 0; i < 8; i++)
	{
		g_DrvUsb_sCtrl.au8SetupData[i] = *(PUINT8)(g_DrvUsb_sCtrl.u32SetupBufAddr+i);
	}	


	psEntryRet = g_DrvUsb_sCtrl.sCtrlEntry.pasCtrlCallbackEntry;
	if(psEntryRet == 0)
		return;

	for (i = 0; i < g_DrvUsb_sCtrl.sCtrlEntry.u32MaxCtrlCallbackReg; i++)
	{
		psEntryRet = g_DrvUsb_sCtrl.sCtrlEntry.pasCtrlCallbackEntry + i;
		if (psEntryRet->u8RequestType == (g_DrvUsb_sCtrl.au8SetupData[0] & 0x60) &&
		        psEntryRet->u8Request == g_DrvUsb_sCtrl.au8SetupData[1])
		{
			psEntryRet->pfnCtrlSetupCallback(psEntryRet->pVoid);
			return;
		}
	}
	_DRVUSB_CLR_CTRL_READY_AND_TRIG_STALL();
	return;
}

PUINT8 
DrvUsb_CtrlGetSetupData(void)
{
	return &(g_DrvUsb_sCtrl.au8SetupData[0]); //TODO: au8SetupData[] should be align 4 byte
}

//======================================================
void
DrvUsb_CtrlDataInAck(
    PVOID pArgu
)
{
	UINT32 i;
	S_DRVUSB_CTRL_CALLBACK_ENTRY *psEntryRet = 0;

	psEntryRet = g_DrvUsb_sCtrl.sCtrlEntry.pasCtrlCallbackEntry;
	if(psEntryRet == 0)
		return;


	for (i = 0; i < g_DrvUsb_sCtrl.sCtrlEntry.u32MaxCtrlCallbackReg; i++)
	{
		psEntryRet = g_DrvUsb_sCtrl.sCtrlEntry.pasCtrlCallbackEntry + i;
		if (psEntryRet->u8RequestType == (g_DrvUsb_sCtrl.au8SetupData[0] & 0x60) &&
		        psEntryRet->u8Request == g_DrvUsb_sCtrl.au8SetupData[1])
		{
			if(psEntryRet->pfnCtrlDataInCallback)
				psEntryRet->pfnCtrlDataInCallback(psEntryRet->pVoid);
			else
				DrvUsb_CtrlDataInDefault(0);
			return;
		}
	}
	return;
}
	

//======================================================
void DrvUsb_CtrlDataOutAck(
	PVOID pArgu
)
{
	UINT32 i;
	S_DRVUSB_CTRL_CALLBACK_ENTRY *psEntryRet = 0;

	psEntryRet = g_DrvUsb_sCtrl.sCtrlEntry.pasCtrlCallbackEntry;
	if(psEntryRet == 0)
		return;


	for (i = 0; i < g_DrvUsb_sCtrl.sCtrlEntry.u32MaxCtrlCallbackReg; i++)
	{
		psEntryRet = g_DrvUsb_sCtrl.sCtrlEntry.pasCtrlCallbackEntry + i;
		if (psEntryRet->u8RequestType == (g_DrvUsb_sCtrl.au8SetupData[0] & 0x60) &&
		        psEntryRet->u8Request == g_DrvUsb_sCtrl.au8SetupData[1])
		{
			if(psEntryRet->pfnCtrlDataOutCallback)
				psEntryRet->pfnCtrlDataOutCallback(psEntryRet->pVoid);
			else
				DrvUsb_CtrlDataOutDefault(0);			
			return;
		}
	}
	return;
}

