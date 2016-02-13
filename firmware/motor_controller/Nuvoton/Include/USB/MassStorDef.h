/****************************************************************
 *                                                              *
 * Copyright (c) Nuvoton Technology Corp. All rights reserved.  *
 *                                                              *
 ****************************************************************/
#ifndef __MASSSTORDEF_H__
#define __MASSSTORDEF_H__

#ifdef  __cplusplus
extern "C"
{
#endif

// USB mass storage class commands
#define	MASS_STOR_RESET					0xFF
#define	MASS_STOR_GET_MAX_LUN			0xFE

#define	CBW_SIGNATURE	0x43425355
#define	CSW_SIGNATURE	0x53425355

#define 	CBW_LENG 31
#define 	CSW_LENG 13

#define	CSW_STATUS_PASSED	0x00
#define	CSW_STATUS_FAILED	0x01
#define	CSW_STATUS_ERROR	0x02

//==============================================================
#define	SPC_VENDOR_WB_WTR									0x06
#define	VA75_OPEN_CLOSE		0x51
#define	VA75_SET_FLASH_TYPE	0x52
#define VA75_WHOLE_ERASE	0x53
#define	VA75_ADDRESS		0x55
#define VA75_CHECK_DONE		0x56
#define VA75_USB_SELF_TEST	0x5B
#define VA75_GET_STOR_SIZE	0x61
#define	VA75_READ			0x71
#define	VA75_WRITE			0x72
#define	VA75_EXECUTE		0x60
#define	VA75_SET_USB_UBP_MODE	0x62

#define VA92_GPU_SET		0x63
#define VA92_WRITE_REGISTER	0x64
#define VA92_READ_REGISTER	0x65

#define	SPC_TEST_UNIT_READY									0x00
//#define	UFI_REZERO_UNIT									0x01
#define	SPC_REQUEST_SENSE									0x03
//#define	UFI_FORMAT_UNIT									0x04
//#define	UFI_INITIALIZE_ELEMENT_STATUS					0x07
#define	SPC_INQUIRY											0x12
#define	SPC_MODE_SELECT_6									0x15
//#define	UFI_RESERVE_6									0x16
//#define	UFI_RELEASE_6									0x17
//#define	UFI_OBSOLETE									0x18
#define	SPC_MODE_SENSE_6									0x1A
//#define	UFI_START_STOP									0x1B
//#define	UFI_RECEIVE_DIAGNOISTIC_RESULTS					0x1C
//#define	UFI_SEND_DIAGNOSTIC								0x1D
#define	MMC_PREVENT_ALLOW_MEDIUM_REMOVAL					0x1E
#define	MMC_READ_FORMAT_CAPACITY							0x23
#define	SBC_READ_CAPACITY									0x25
#define	MMC_READ_CAPACITY									0x25
#define	SBC_READ_10											0x28
#define	SBC_WRITE_10										0x2A
//#define	UFI_SEEK										0x2B
//#define	UFI_ERASE_10									0x2C
//#define	UFI_WRITE_AND_VERIFY_10							0x2E
#define	SBC_VERIFY_10										0x2F
#define	MMC_VERIFY_10										0x2F
//#define	UFI_SYNCHRONIZE_CACHE							0x35
//#define	UFI_INITIALIZE_ELEMENT_STATUS_WITH_RANGE		0x37
//#define	UFI_OBSOLETE									0x39
//#define	UFI_OBSOLETE									0x3A
//#define	UFI_WRITE_BUFFER								0x3B
//#define	UFI_READ_BUFFER									0x3C
//#define	UFI_OBSOLETE									0x40
//#define	UFI_READ_SUB_CHANNEL							0x42
#define	MMC_READ_TOC_PMA_ATIP								0x43
//#define	UFI_READ_HEADER									0x44
//#define	UFI_PLAY_AUDIO_10								0x45
#define	UFI_GET_CONFIGURATION								0x46
//#define	UFI_PLAY_AUDIO_MSF								0x47
#define	UFI_GET_EVENT_STATUS_NOTIFICATION					0x4A
//#define	UFI_PAUSE_RESUME								0x4B
//#define	UFI_LOG_SELECT									0x4C
//#define	UFI_LOG_SENSE									0x4D
//#define	UFI_STOP_PLAY_SCAN								0x4E
#define	UFI_READ_DISC_INFORMATION						0x51
//#define	UFI_READ_TRACK_INFORMATION						0x52
//#define	UFI_RESERVE_TRACK								0x53
//#define	UFI_SEND_OPC_INFORMATION						0x54
#define	SPC_MODE_SELECT_10								0x55
//#define	UFI_RESERVE_10									0x56
//#define	UFI_RELEASE_10									0x57
//#define	UFI_REPAIR_TRACK								0x58
#define	SPC_MODE_SENSE_10									0x5A
//#define	UFI_CLOSE_TRACK_SESSION							0x5B
//#define	UFI_READ_BUFFER_CAPACITY						0x5C
//#define	UFI_SEND_CUE_SHEET								0x5D
//#define	UFI_PERSISTENT_RESERVE_IN						0x5E
//#define	UFI_PERSISTENT_RESERVE_OUT						0x5F
//#define	UFI_EXTENED_COPY								0x83
//#define	UFI_ACCESS_CONTROL_IN							0x86
//#define	UFI_ACCESS_CONTROL_OUT							0x87
//#define	UFI_READ_ATTRIBUTE								0x8C
//#define	UFI_WRITE_ATTRIBUTE								0x8D
//#define	UFI_REPORT_LUNS									0xA0
//#define	UFI_BLANK										0xA1
//#define	UFI_SEND_EVENT									0xA2
//#define	UFI_SEND_KEY									0xA3
//#define	UFI_REPORT_DEVICE_IDENTIFIER					0xA3/0x05
//#define	UFI_REPORT_TARGET_PORT_GROUPS					0xA3/0x0A
//#define	UFI_REPORT_ALIASES								0xA3/0x0B
//#define	UFI_REPORT_SUPPORTED_OPERATION_CODES			0xA3/0x0C
//#define	UFI_REPORT_SUPPORTED_TASK_MANAGEMENT_FUNCTIONS	0xA3/0x0D
//#define	UFI_REPORT_KEY									0xA4
//#define	UFI_SET_DEVICE_IDENTIFIER						0xA4/0x06
//#define	UFI_SET_TARGET_PORT_GROUPS						0xA4/0x0A
//#define	UFI_CHANGE_ALIASES								0xA4/0x0B
//#define	UFI_SET_PRIORITY								0xA4/0x0E
//#define	UFI_SET_TIMESTAMP								0xA4/0x0F
//#define	UFI_MOVE_MEDIUM									0xA5
//#define	UFI_PLAY_AUDIO_12								0xA5
//#define	UFI_EXCHANGE_MEDIUM								0xA6
//#define	UFI_LOAD_UNLOAD_CD								0xA6
//#define	UFI_MOVE_MEDIUM_ATTACHED						0xA7
//#define	UFI_SET_READ_AHEAD								0xA7
//#define	UFI_READ_12										0xA8
//#define	UFI_WRITE_12									0xAA
//#define	UFI_READ_MEDIA_SERIAL_NUMBER					0xAB/0x01
//#define	UFI_GET_PERFORMANCE								0xAC
//#define	UFI_READ_DVD_STRUCTURE							0xAD
//#define	UFI_REVEIVE_COPY_RESULTS						0xB4
//#define	UFI_REQUEST_VOLUME_ELEMENT_ADDRESS				0xB5
//#define	UFI_SEND_VOLUME_TAG								0xB6
//#define	UFI_SET_STREAMING								0xB6
//#define	UFI_READ_ELEMENT_STATUS							0xB8
//#define	UFI_READ_CD_MSF									0xB9
//#define	UFI_SCAN										0xBA
#define	SPC_SET_CD_SPEED								0xBB
//#define	UFI_PLAY_CD										0xBC
//#define	UFI_MECHANISM_STATUS							0xBD
#define	SPC_READ_CD										0xBE
//#define	UFI_SEND_DISC_STRUCTURE							0xBF
//==============================================================

#define SCSI_VENDOR_06								0x06

#define SCSI_VENDOR_BEGIN							0xC0

#define SCSI_VENDOR_WB_BEGIN						0xC0
#define SCSI_VENDOR_WB_INIT							0xC0
#define SCSI_VENDOR_WB_FINAL						0xC1
#define SCSI_VENDOR_WB_READ_CAPACITY				0xC2
#define SCSI_VENDOR_WB_READ_10						0xC3
#define SCSI_VENDOR_WB_WRITE_10						0xC4
#define SCSI_VENDOR_WB_FLUSH						0xC5
#define SCSI_VENDOR_WB_ERROR						0xDF
#define SCSI_VENDOR_WB_END							0xDF

#define SCSI_VENDOR_END								0xFF

#define	SK_NO_SENSE			0x00
#define	SK_RECOVERED_ERROR	0x01
#define	SK_NOT_READY		0x02
#define	SK_MEDIUM_ERROR		0x03
#define	SK_HARDWARE_ERROR	0x04
#define	SK_ILLEGAL_REQUEST	0x05
#define	SK_UNIT_ATTENTION	0x06
#define	SK_DATA_PROTECT		0x07
#define	SK_BLANK_CHECK		0x08
#define	SK_VENDOR_SPECIFIC	0x09
#define	SK_RESERVED			0x0A
#define	SK_ABORT_COMMAND	0x0B

#define SCSI_INQUIRY_LEN			0x24
#define SCSI_PDT5TOC_LEN			0x14
#define SCSI_PDT5MODESENSE_LEN		0x1C

#ifdef  __cplusplus
}
#endif

#endif // __MASSSTORDEF_H__
