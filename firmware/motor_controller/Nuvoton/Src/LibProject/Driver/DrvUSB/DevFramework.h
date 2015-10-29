/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.  *
 *                                                              *
 ****************************************************************/


#ifndef __DEV_FRAMEWORK_H__
#define __DEV_FRAMEWORK_H__

#ifdef  __cplusplus
extern "C"
{
#endif

#define MAX_EP_NUM		6
#define MAX_MISC_EVENT_NUM		6

#define	DRVUSB_EVENT_FLAG_ATTACH			BIT0
#define	DRVUSB_EVENT_FLAG_DETACH			BIT1
#define	DRVUSB_EVENT_FLAG_BUS_RESET			BIT2
#define	DRVUSB_EVENT_FLAG_BUS_SUSPEND		BIT3
#define	DRVUSB_EVENT_FLAG_BUS_RESUME		BIT4
#define	DRVUSB_EVENT_FLAG_SETUP				BIT5

	typedef void (*PFN_DRVUSB_CALLBACK)(PVOID);

	typedef struct
	{
		UINT32 u32ATTR;
		UINT32 u32STS;
		UINT32 u32FLODET;
		UINT32 u32EVF;

		BOOL abData0Out[MAX_EP_NUM];
		INT32 eUsbState;

		// BIT0		USB attach
		// BIT1		USB detach
		// BIT2		bus reset
		// BIT3		bus suspend
		// BIT4		bus resume
		// BIT5		Setup
		UINT32	u32MiscEventFlags;

		// BIT0		EP0	In
		// BIT1		EP1	In
		// ...
		// BIT15	EP15 In
		// BIT16	EP0 Out
		// BIT17	EP1 Out
		// ...
		// BIT31	EP15 Out
		UINT32	u32EPEventFlags;
		
		// for main routine
		PFN_DRVUSB_CALLBACK	apfnMiscCallback[MAX_MISC_EVENT_NUM];
		PVOID					apMiscCallbackArgu[MAX_MISC_EVENT_NUM];
		
		PFN_DRVUSB_CALLBACK	apfnEPCallback[MAX_EP_NUM*2];
		PVOID					apEPCallbackArgu[MAX_EP_NUM*2];
	} S_DRVUSB_DEVICE;
	
//***************************************************
// Define framework.c function
//***************************************************
	void DrvUsb_EnableUsb(
		S_DRVUSB_DEVICE *psDevice
	);
	
	void DrvUsb_DisableUsb(
		S_DRVUSB_DEVICE *psDevice
	);

	void DrvUsb_PreDispatchWakeupEvent(
		S_DRVUSB_DEVICE *psDevice
	);
	
	void DrvUsb_PreDispatchFdtEvent(
		S_DRVUSB_DEVICE *psDevice
	);
	
	void DrvUsb_PreDispatchEPEvent(
		S_DRVUSB_DEVICE *psDevice
	);
	
	void DrvUsb_PreDispatchBusEvent(
		S_DRVUSB_DEVICE *psDevice
	);
	
	void DrvUsb_DispatchWakeupEvent(
	    S_DRVUSB_DEVICE *psDevice
	);
	
	void DrvUsb_DispatchMiscEvent(
		S_DRVUSB_DEVICE *psDevice
	);
	
	void DrvUsb_DispatchEPEvent(
		S_DRVUSB_DEVICE *psDevice
	);


	
#ifdef  __cplusplus
}
#endif

#endif // __DEV_FRAMEWORK_H__




































