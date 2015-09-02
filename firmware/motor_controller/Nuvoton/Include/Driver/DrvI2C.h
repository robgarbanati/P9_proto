/*---------------------------------------------------------------------------------------------------------*/
/*                                                                                                         */
/* Copyright(c) 2010 Nuvoton Technology Corp. All rights reserved.                                         */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
#ifndef __DRVI2C_H__
#define __DRVI2C_H__

#include "Platform.h"
#include "System/SysInfra.h"

//---------------------------------------------------------------------------------------------------------
// Macro, type and constant definitions
//---------------------------------------------------------------------------------------------------------
#define DRVI2C_MAJOR_NUM 2
#define DRVI2C_MINOR_NUM 00
#define DRVI2C_BUILD_NUM 001

#define DRVI2S_VERSION_NUM    _SYSINFRA_VERSION(DRVI2C_MAJOR_NUM,DRVI2C_MINOR_NUM,DRVI2C_BUILD_NUM)

typedef enum{
	 E_DRVI2C_CLK_10K = 50,	//CycleTime: High/Low - 50 Microsecond 
	 E_DRVI2C_CLK_20K = 25	//CycleTime: High/Low - 25 Microsecond
} E_DRVI2C_CLK;

typedef enum{
	 E_DRVI2C_GPIOPORTA = 0,
	 E_DRVI2C_GPIOPORTB
} E_DRVI2C_GPIOPORT;

typedef enum{
	 E_DRVI2C_GPIOPIN0 = 0,
	 E_DRVI2C_GPIOPIN1,
	 E_DRVI2C_GPIOPIN2,
	 E_DRVI2C_GPIOPIN3,
	 E_DRVI2C_GPIOPIN4,
	 E_DRVI2C_GPIOPIN5,
	 E_DRVI2C_GPIOPIN6,
	 E_DRVI2C_GPIOPIN7,
	 E_DRVI2C_GPIOPIN8,
	 E_DRVI2C_GPIOPIN9,
	 E_DRVI2C_GPIOPIN10,
	 E_DRVI2C_GPIOPIN11,
	 E_DRVI2C_GPIOPIN12,
	 E_DRVI2C_GPIOPIN13,
	 E_DRVI2C_GPIOPIN14,
	 E_DRVI2C_GPIOPIN15
} E_DRVI2C_GPIOPIN;

/*---------------------------------------------------------------------------------------------------------*/
/* Function: DrvI2C_Open                                                                                   */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*              eSCKPort - [in], The bit for SCK IO pin.                                                   */
/*              eSCKPin  - [in], The GPIO port where the SCK IO pin belong to.                             */
/*              eSDAPort - [in], The bit for SDA IO pin.                                                   */
/*              eSDAPin  - [in], The GPIO port where the SDA IO pin belong to.                             */
/*              eI2cClk - [in], Set the S/W I2c Clk.                                                       */
/*                                                                                                         */
/* Returns:                                                                                                */
/*              None                                                                                       */
/*                                                                                                         */
/* Description:                                                                                            */
/*              To specify the GPIO pins for SDA/SCK of I2C bus and initial the software I2C.              */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void DrvI2C_Open(
	E_DRVI2C_GPIOPORT eSCKPort,
    E_DRVI2C_GPIOPIN eSCKPin,
    E_DRVI2C_GPIOPORT eSDAPort,
    E_DRVI2C_GPIOPIN eSDAPin,
    E_DRVI2C_CLK eI2cClk
);

/*---------------------------------------------------------------------------------------------------------*/
/* Function: DrvI2C_Close                                                                                 */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*              None                                                                                       */
/*                                                                                                         */
/* Returns:                                                                                                */
/*              None                                                                                       */
/*                                                                                                         */
/* Description:                                                                                            */
/*              Reset the SDA/SCK IO pins to be input to disable software I2C.                             */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void DrvI2C_Close(void);

/*---------------------------------------------------------------------------------------------------------*/
/* Function: DrvI2C_SendStart                                                                             */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*              None                                                                                       */
/*                                                                                                         */
/* Returns:                                                                                                */
/*              None                                                                                       */
/*                                                                                                         */
/* Description:                                                                                            */
/*              To send START signal to I2C bus.                                                           */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void DrvI2C_SendStart(void);

/*---------------------------------------------------------------------------------------------------------*/
/* Function: DrvI2C_SendStop                                                                              */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*              None                                                                                       */
/*                                                                                                         */
/* Returns:                                                                                                */
/*              None                                                                                       */
/*                                                                                                         */
/* Description:                                                                                            */
/*              To send STOP signal to I2C bus.                                                            */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void DrvI2C_SendStop(void);

/*---------------------------------------------------------------------------------------------------------*/
/* Function: DrvI2C_WriteByte                                                                              */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*              bStart - [in], Enable to send START signal before send data.                               */
/*              u8Data - [in], The byte to send through I2C bus.                                           */
/*              bCheckAck - [in], Enable to check ACK after send data.                                     */
/*              bStop - [in], Enable to send STOP signal at the end.                                       */
/*                                                                                                         */
/* Returns:                                                                                                */
/*              TRUE          Success                                                                      */
/*              FALSE         Fail                                                                         */
/*                                                                                                         */
/* Description:                                                                                            */
/*              Send a byte to I2C bus.                                                                    */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
BOOL 
DrvI2C_WriteByte(
	BOOL bStart,
	UINT8 u8Data,
	BOOL bCheckAck,
	BOOL bStop
);

/*---------------------------------------------------------------------------------------------------------*/
/* Function: DrvI2C_ReadByte                                                                              */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*              bStart - [in], Enable to send START signal before send data.                               */
/*              pu8ReadData - [out], The pointer to store the byte read through I2C bus.                   */
/*              bSendAck - [in], Enable to send ACK after read data.                                       */
/*              bStop - [in], Enable to send STOP signal at the end.                                       */
/*                                                                                                         */
/* Returns:                                                                                                */
/*              TRUE          Success                                                                      */
/*              FALSE         Fail                                                                         */
/*                                                                                                         */
/* Description:                                                                                            */
/*              Read a byte from I2C bus (MSB first).                                                      */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
BOOL 
DrvI2C_ReadByte(
	BOOL bStart, 
	PUINT8 pu8ReadData, 
	BOOL bSendAck, 
	BOOL bStop
);

/*---------------------------------------------------------------------------------------------------------*/
/* Function: DrvI2C_GetVersion                                                                             */
/*                                                                                                         */
/* Parameters:                                                                                             */
/*              None                                                                                       */
/*                                                                                                         */
/* Returns:                                                                                                */
/*              The version number of I2C driver.                                                          */
/*                                                                                                         */
/* Description:                                                                                            */
/*              Get the version number of I2C driver.                                                      */
/*                                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
UINT32
DrvI2C_GetVersion(void);

#endif
