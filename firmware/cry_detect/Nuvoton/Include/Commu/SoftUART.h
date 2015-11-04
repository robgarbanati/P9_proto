/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright (c) Nuvoton Technology Corp. All rights reserved.                                             */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#ifndef __SOFT_UART_H__
#define __SOFT_UART_H__

#include "Platform.h"
#include "System/SysInfra.h"

#ifdef  __cplusplus
extern "C"
{
#endif

/*---------------------------------------------------------------------------------------------------------*/
/*  Define Version number								                                                   */
/*---------------------------------------------------------------------------------------------------------*/

#define SOFTUART_MAJOR_NUM	2
#define SOFTUART_MINOR_NUM	0
#define SOFTUART_BUILD_NUM	1

/* --------------------------------------------------------------------------------------------------------*/
/* Define the Error Code																																									 */
/* --------------------------------------------------------------------------------------------------------*/

// E_SOFTUART_ERR_PARAM					Parameter error

#define E_SOFTUART_ERR_PARAM		_SYSINFRA_ERRCODE(TRUE, MODULE_ID_SOFTUART, 1)  


/* --------------------------------------------------------------------------------------------------------*/
/* Define the Version number																																									 */
/* --------------------------------------------------------------------------------------------------------*/
// Define module version number.
#define SOFTUART_VERSION_NUM		_SYSINFRA_VERSION(SOFTUART_MAJOR_NUM, SOFTUART_MINOR_NUM, SOFTUART_BUILD_NUM)	

/*---------------------------------------------------------------------------------------------------------*/
/* DATA BIT                                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
#define SOFTUART_DATABITS_5		0x5
#define SOFTUART_DATABITS_6		0x6
#define SOFTUART_DATABITS_7		0x7
#define SOFTUART_DATABITS_8		0x8

/*---------------------------------------------------------------------------------------------------------*/
/* PARITY Setting                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
#define SOFTUART_PARITY_NONE		0x00
#define SOFTUART_PARITY_EVEN		0x01
#define SOFTUART_PARITY_ODD			0x02

/*---------------------------------------------------------------------------------------------------------*/
/* STOP BIT                                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
#define SOFTUART_STOPBITS_1		0x1
#define SOFTUART_STOPBITS_2		0x2

/*---------------------------------------------------------------------------------------------------------*/
/* BAUD RATE                                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
#define SOFTUART_BAUD_600			600
#define SOFTUART_BAUD_1200			1200
#define SOFTUART_BAUD_2400			2400
#define SOFTUART_BAUD_4800			4800
#define SOFTUART_BAUD_9600			9600
#define SOFTUART_BAUD_19200			19200
#define SOFTUART_BAUD_38400			38400
#define SOFTUART_BAUD_57600			57600
#define SOFTUART_BAUD_115200		115200
#define SOFTUART_BAUD_230400		230400


/*---------------------------------------------------------------------------------------------------------*/
/* Structures                                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
typedef struct
{
    UINT32	u32BaudRate;
    UINT8		u8DataBits;
    UINT8		u8Parity;
    UINT8		u8StopBits;
}S_SOFTUART_UART_CONF;


typedef struct
{
	SPI_T* psSPIIntf;
}S_SOFTUART_SPI;


typedef struct
{
	TMR_T* psTimer;
	union{
		S_SOFTUART_SPI sSPIIntf;
	};
}S_SOFTUART_INTF;

/*---------------------------------------------------------------------------------------------------------*/
/* APIs		                                                                                               */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
// Function:	SoftUART_Open
// Parameter:	
//				psUartConf [in] The pointer of uart configuration.
//				psUartIntf [in] The pointer of uart hardware resource.
//				u32Reserve [in]	Reserve argument
// Return:
//				Success: E_SUCCESS, Fail: >0 
// Description:                                                                                            
//				Open and initiate the software uart module.
/*---------------------------------------------------------------------------------------------------------*/
ERRCODE
SoftUART_Open(
	S_SOFTUART_UART_CONF *psUartConf,
	S_SOFTUART_INTF	*psUartIntf,
	UINT32 u32Reserve
);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	SoftUART_Close
// Description:                                                                                            
//				Close the software uart module.
/*---------------------------------------------------------------------------------------------------------*/
void SoftUART_Close(void);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	SoftUART_PutChar
// Parameter:	ch [in] The character will be transmitted.€	
// Description:                                                                                            
//				Transmit a character.
/*---------------------------------------------------------------------------------------------------------*/
void 
SoftUART_PutChar(
	CHAR ch
);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	SoftUART_GetChar
// Parameter:	
//				pch [out] The pointer of the receive character. 
// Return:
//				TRUE: Success; FALSE: Fail.	
// Description:                                                                                            
//				Receive a character.
/*---------------------------------------------------------------------------------------------------------*/
BOOL 
SoftUART_GetChar(
	PCHAR pch
);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	SoftUART_SpiIsr
// Description:                                                                                            
//				Should be called by SPI interrupt service routine when softuart operate at SPI mode.
/*---------------------------------------------------------------------------------------------------------*/
void SoftUART_SpiIsr(void);

/*---------------------------------------------------------------------------------------------------------*/
// Function:	SoftUART_GpioIsr
// Description:                                                                                            
//				Should be called by GPIO interrupt service routine when softuart operate at SPI mode.
/*---------------------------------------------------------------------------------------------------------*/
void SoftUART_GpioIsr(void);


/*---------------------------------------------------------------------------------------------------------*/
// Function:	SoftUART_GetVersion
// Return:
//				Version number
// Description:                                                                                            
//				Get version number of this module
/*---------------------------------------------------------------------------------------------------------*/
UINT32
SoftUART_GetVersion(void);


#ifdef  __cplusplus
}
#endif

#endif
