/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.  *
 *                                                              *
 ****************************************************************/


#ifndef __CTRL_FRAMEWORK_H__
#define __CTRL_FRAMEWORK_H__

#ifdef  __cplusplus
extern "C"
{
#endif

typedef struct
{
	S_DRVUSB_CTRL_CALLBACK_ENTRY *pasCtrlCallbackEntry;
	UINT32 u32MaxCtrlCallbackReg;
	UINT8 u8CtrlDataInEP;
	UINT8 u8CtrlDataOutEP;
}S_DRVUSB_CTRL_ENTRY;



typedef struct
{
	UINT8 au8SetupData[8];
	S_DRVUSB_CTRL_ENTRY sCtrlEntry;
	UINT32 u32SetupBufAddr;
}S_DRVUSB_CTRL;

extern S_DRVUSB_CTRL	g_DrvUsb_sCtrl;

void DrvUsb_CtrlDataInDefault(PVOID pVoid);

void DrvUsb_CtrlDataOutDefault(PVOID pVoid);

static __inline
void
_DRVUSB_CLR_CTRL_READY_AND_TRIG_STALL()
{
	_DRVUSB_CLEAR_EP_READY_AND_TRIG_STALL(g_DrvUsb_sCtrl.sCtrlEntry.u8CtrlDataInEP);
	_DRVUSB_CLEAR_EP_READY_AND_TRIG_STALL(g_DrvUsb_sCtrl.sCtrlEntry.u8CtrlDataOutEP);
}

static __inline
void
_DRVUSB_CLR_CTRL_READY()
{
	_DRVUSB_CLEAR_EP_READY(g_DrvUsb_sCtrl.sCtrlEntry.u8CtrlDataInEP);
	_DRVUSB_CLEAR_EP_READY(g_DrvUsb_sCtrl.sCtrlEntry.u8CtrlDataOutEP);
}



#ifdef  __cplusplus
}
#endif

#endif  // __CTRL_FRAMEWORK_H__
