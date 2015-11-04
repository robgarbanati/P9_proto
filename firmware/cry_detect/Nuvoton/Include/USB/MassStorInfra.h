/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.  *
 *                                                              *
 ****************************************************************/
#ifndef __MASSSTORINFRA_H__
#define __MASSSTORINFRA_H__


#ifdef  __cplusplus
extern "C"
{
#endif

#define MASSSTORINFRA_MAX_LU_CNT 	16

typedef __packed  struct {
	__packed  union {
	UINT32 	u32StorType;
	UINT8	u8StorType;
	};
	__packed  union {
	UINT32 	u32HeadCnt;
	UINT16	u16HeadCnt;
	};
	__packed  union {
	UINT32 	u32TrkCnt;
	UINT16	u16TrkCnt;	
	};
	__packed  union {
	UINT32 	u32SecPerTrk;
	UINT16 	u16SecPerTrk;
	};
	__packed  union {
	UINT32 	u32BytePerSec;
	UINT16	u16BytePerSec;
	};
	// media size may be u16SecPerTrk * u16TrkCnt * u16HeadCnt
	UINT32 u32StorSz;
} S_MASSSTORINFRA_STOR_INFO;

typedef INT32 (*PFN_MASSSTORINFRA_STORIF_OPEN) (PVOID);
typedef INT32 (*PFN_MASSSTORINFRA_STORIF_CLOSE) (void);
typedef INT32 (*PFN_MASSSTORINFRA_STORIF_GETSTORINFO) (S_MASSSTORINFRA_STOR_INFO *psStorInfo);
typedef INT32 (*PFN_MASSSTORINFRA_STORIF_READLBA) (UINT32 u32StartLba, UINT32 u32BlkCnt, PUINT8 buffer);
typedef INT32 (*PFN_MASSSTORINFRA_STORIF_WRITELBA) (UINT32 u32StartLba, UINT32 u32BlkCnt, const PUINT8 buffer);
typedef INT32 (*PFN_MASSSTORINFRA_STORIF_FLUSH) (void);

typedef __packed struct {
	__packed  union {
	PFN_MASSSTORINFRA_STORIF_OPEN			pfnOpen;
	PFN_MASSSTORINFRA_STORIF_OPEN				pfnInit;
	};
	__packed  union {
	PFN_MASSSTORINFRA_STORIF_CLOSE		pfnClose;
	PFN_MASSSTORINFRA_STORIF_CLOSE		pfnFinal;
	};
	PFN_MASSSTORINFRA_STORIF_GETSTORINFO	pfnGetStorInfo;
	PFN_MASSSTORINFRA_STORIF_READLBA		pfnReadLba;
	PFN_MASSSTORINFRA_STORIF_WRITELBA		pfnWriteLba;
	PFN_MASSSTORINFRA_STORIF_FLUSH		pfnFlush;
} S_MASSSTORINFRA_STOR_IF;


typedef INT32 (*PFN_MASSSTORINFRA_GET_STOR_IF) (S_MASSSTORINFRA_STOR_IF *);


	typedef struct 
	{
		UINT32	dCBWSignature;
		UINT32	dCBWTag;
		UINT32	dCBWDataTransferLength;
		UINT8	bmCBWFlags;
		UINT8	bCBWLUN;
		UINT8	bCBWCBLength;
		UINT8	CBWCB[16];
	} S_MASSSTORINFRA_CBW;

	typedef __packed struct 
	{
		UINT32	dCSWSignature;
		UINT32	dCSWTag;
		UINT32	dCSWDataResidue;
		UINT8	bCSWStatus;
	} S_MASSSTORINFRA_CSW;


	typedef __packed struct
	{
		CHAR pszVendorId[8];
		CHAR pszProductId[16];
		CHAR pszProductRevision[4];
	} S_MASSSTORINFRA_SCSI_VENDOR_INFO;



	typedef struct
	{
		UINT8 u8PDT;
		PFN_MASSSTORINFRA_GET_STOR_IF pfnGetStorIf;
		PVOID pVoid;
		BOOL bHide;
	} S_MASSSTORINFRA_LU_ENTRY;


	typedef BOOL
	(*PFN_MASSSTORINFRA_IS_WRITE_PROTECT_CALLBACK)(
	    PVOID pVoid
	);

	typedef void (*PFN_MASSSTORINFRA_BOT_CBW_CALLBACK)(
	    PVOID pVoid
	);

	typedef void (*PFN_MASSSTORINFRA_BOT_DATA_CALLBACK)(
	    PVOID pVoid
	);

	typedef void (*PFN_MASSSTORINFRA_BOT_CSW_CALLBACK)(
	    PVOID pVoid
	);
/*
	typedef struct
	{
		UINT8 au8PDTs[MASSSTORINFRA_MAX_LU_CNT];
		UINT8 *apu8Inquiry[MASSSTORINFRA_MAX_LU_CNT];

		S_STORIF_STOR_IF asStorIf[MASSSTORINFRA_MAX_LU_CNT];
		PVOID apVoid[MASSSTORINFRA_MAX_LU_CNT];
		UINT16 au16LogiBlkSzs[MASSSTORINFRA_MAX_LU_CNT];
		UINT32 au32MassStorFlags[MASSSTORINFRA_MAX_LU_CNT];
		BOOL abHide[MASSSTORINFRA_MAX_LU_CNT];
		PFN_MASSSTORINFRA_IS_WRITE_PROTECT_CALLBACK apfnIsWriteProtectCallback[MASSSTORINFRA_MAX_LU_CNT];
		PVOID apIsWriteProtectArgu[MASSSTORINFRA_MAX_LU_CNT];
	} S_MASSSTORINFRA_LU_REG;

*/
	typedef __packed struct
	{
		UINT8 u8PDT;
		UINT8 *pu8Inquiry;

		S_MASSSTORINFRA_STOR_IF sStorIf;
		PVOID pVoid;
		UINT16 u16LogiBlkSz;
		UINT32 u32MassStorFlag;
		BOOL bHide;
		PFN_MASSSTORINFRA_IS_WRITE_PROTECT_CALLBACK pfnIsWriteProtectCallback;
		PVOID pIsWriteProtectArgu;
	} S_MASSSTORINFRA_LU_REG;


	typedef __packed struct
	{
		UINT8 u8Cmd;
		PFN_MASSSTORINFRA_BOT_CBW_CALLBACK 		pfnCBWCallback;
		PFN_MASSSTORINFRA_BOT_DATA_CALLBACK 	pfnDataCallback;
		PFN_MASSSTORINFRA_BOT_CSW_CALLBACK 		pfnCSWCallback;
		PVOID pVoid;
	} S_MASSSTORINFRA_SCSI_ENTRY;

	typedef struct//clyu
	{
		UINT8 u8Cmd;
		PFN_MASSSTORINFRA_BOT_CBW_CALLBACK 		pfnCBWCallback;
		PFN_MASSSTORINFRA_BOT_DATA_CALLBACK 	pfnDataCallback;
		PFN_MASSSTORINFRA_BOT_CSW_CALLBACK 		pfnCSWCallback;
		PVOID pVoid;
	} S_MASSSTORINFRA_VCOM_ENTRY;//clyu

	typedef __packed struct
	{
		BOOL bReg;
		S_MASSSTORINFRA_SCSI_ENTRY sScsiEntry;
	} S_MASSSTORINFRA_SCSI_REG;
	typedef struct
	{
		BOOL bReg;
		S_MASSSTORINFRA_VCOM_ENTRY sVComEntry;
	} S_MASSSTORINFRA_VCOM_REG;//clyu

	typedef enum
	{
		eMASSSTORINFRA_BOT_CALLBACK_ID_CBW,
		eMASSSTORINFRA_BOT_CALLBACK_ID_DATA_IN,
		eMASSSTORINFRA_BOT_CALLBACK_ID_DATA_OUT,
		eMASSSTORINFRA_BOT_CALLBACK_ID_CSW,

		eBOT_HANDLER_ID_CBW = eMASSSTORINFRA_BOT_CALLBACK_ID_CBW,
		eBOT_HANDLER_ID_DATA_IN = eMASSSTORINFRA_BOT_CALLBACK_ID_DATA_IN,
		eBOT_HANDLER_ID_DATA_OUT = eMASSSTORINFRA_BOT_CALLBACK_ID_DATA_OUT,
		eBOT_HANDLER_ID_CSW = eMASSSTORINFRA_BOT_CALLBACK_ID_CSW

	} E_MASSSTORINFRA_BOT_CALLBACK_ID;

#ifdef  __cplusplus
}
#endif


#endif // __MASSSTORINFRA_H__
















