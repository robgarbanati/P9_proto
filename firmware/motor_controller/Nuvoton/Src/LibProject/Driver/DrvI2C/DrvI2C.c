//---------------------------------------------------------------------------------------------------------
//                                                                                                         
// Copyright(c) 2010 Nuvoton Technology Corp. All rights reserved.                                         
//                                                                                                         
//---------------------------------------------------------------------------------------------------------
#include "Driver/DrvTimer.h"
#include "Driver/DrvGPIO.h"
#include "Driver/DrvI2C.h"
//---------------------------------------------------------------------------------------------------------
// Global file scope (static) variables                                                                    
//---------------------------------------------------------------------------------------------------------
static GPIO_T *s_GpioPort[] = {&GPIOA, &GPIOB};

static E_DRVI2C_GPIOPORT s_eSCKPort;
static E_DRVI2C_GPIOPORT s_eSDAPort;
static E_DRVI2C_GPIOPIN s_eSCKPin;
static E_DRVI2C_GPIOPIN s_eSDAPin;

static UINT16 s_u16CycleTime;
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
)
{
	s_eSCKPort = eSCKPort;
	s_eSDAPort = eSDAPort;
	s_eSCKPin = eSCKPin;
	s_eSDAPin = eSDAPin;
	
	DrvGPIO_SetIOModeEx(s_GpioPort[s_eSCKPort],(DRVGPIO_IOMODE_PIN0_OUT<<(s_eSCKPin<<1)), DRVGPIO_PIN_0<<s_eSCKPin);
	DrvGPIO_SetIOModeEx(s_GpioPort[s_eSDAPort],(DRVGPIO_IOMODE_PIN0_OUT<<(s_eSDAPin<<1)), DRVGPIO_PIN_0<<s_eSDAPin);
	
	DrvGPIO_SetOutputBit(s_GpioPort[s_eSCKPort],DRVGPIO_PIN_0<<s_eSCKPin);
	DrvGPIO_SetOutputBit(s_GpioPort[s_eSDAPort],DRVGPIO_PIN_0<<s_eSDAPin);
	
	s_u16CycleTime = eI2cClk;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function: DrvI2C_Close                                                                                  */
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
void DrvI2C_Close(void)
{
	DrvGPIO_SetIOModeEx(s_GpioPort[s_eSCKPort],(DRVGPIO_IOMODE_PIN0_IN<<(s_eSCKPin<<1)), DRVGPIO_PIN_0<<s_eSCKPin);
	DrvGPIO_SetIOModeEx(s_GpioPort[s_eSDAPort],(DRVGPIO_IOMODE_PIN0_IN<<(s_eSDAPin<<1)), DRVGPIO_PIN_0<<s_eSDAPin);
}

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
void DrvI2C_SendStart(void)
{
	DrvGPIO_SetOutputBit(s_GpioPort[s_eSCKPort],DRVGPIO_PIN_0<<s_eSCKPin);
	DrvGPIO_SetOutputBit(s_GpioPort[s_eSDAPort],DRVGPIO_PIN_0<<s_eSDAPin);
	
	DrvTimer_WaitMicrosecondTmr2(s_u16CycleTime);
	
	DrvGPIO_ClearOutputBit(s_GpioPort[s_eSDAPort],DRVGPIO_PIN_0<<s_eSDAPin);
	DrvTimer_WaitMicrosecondTmr2(s_u16CycleTime);
	DrvGPIO_ClearOutputBit(s_GpioPort[s_eSCKPort],DRVGPIO_PIN_0<<s_eSCKPin);
}

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
void DrvI2C_SendStop(void)
{
	DrvGPIO_ClearOutputBit(s_GpioPort[s_eSDAPort],DRVGPIO_PIN_0<<s_eSDAPin);
	DrvGPIO_ClearOutputBit(s_GpioPort[s_eSCKPort],DRVGPIO_PIN_0<<s_eSCKPin);
	
	DrvTimer_WaitMicrosecondTmr2(s_u16CycleTime);
	
	DrvGPIO_SetOutputBit(s_GpioPort[s_eSCKPort],DRVGPIO_PIN_0<<s_eSCKPin);
	DrvTimer_WaitMicrosecondTmr2(s_u16CycleTime);
	DrvGPIO_SetOutputBit(s_GpioPort[s_eSDAPort],DRVGPIO_PIN_0<<s_eSDAPin);
}

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
)
{
	UINT8 u8DataCount;
	UINT8 u8ChangeTime=s_u16CycleTime/4;
	UINT16 u16AckState;

	// Send Start Condition to Device	
	if(bStart)
	    DrvI2C_SendStart();
	
	// Send Data to Device (MSB->LSB)
	for(u8DataCount=0;u8DataCount<8;u8DataCount++)
	{	
		DrvGPIO_ClearOutputBit(s_GpioPort[s_eSCKPort],DRVGPIO_PIN_0<<s_eSCKPin);		
		DrvTimer_WaitMicrosecondTmr2(s_u16CycleTime-u8ChangeTime);
			
		if ( u8Data&0x80 )
			DrvGPIO_SetOutputBit(s_GpioPort[s_eSDAPort],DRVGPIO_PIN_0<<s_eSDAPin);
		else
			DrvGPIO_ClearOutputBit(s_GpioPort[s_eSDAPort],DRVGPIO_PIN_0<<s_eSDAPin);		
		DrvTimer_WaitMicrosecondTmr2(u8ChangeTime);
		
		DrvGPIO_SetOutputBit(s_GpioPort[s_eSCKPort],DRVGPIO_PIN_0<<s_eSCKPin);
		DrvTimer_WaitMicrosecondTmr2(s_u16CycleTime);

		u8Data<<=1;
	}
	
	// Get Ack from Device
	DrvGPIO_ClearOutputBit(s_GpioPort[s_eSCKPort],DRVGPIO_PIN_0<<s_eSCKPin);		
	DrvTimer_WaitMicrosecondTmr2(s_u16CycleTime-u8ChangeTime);
	
	DrvGPIO_SetIOModeEx(s_GpioPort[s_eSDAPort],(DRVGPIO_IOMODE_PIN0_IN<<(s_eSDAPin<<1)), DRVGPIO_PIN_0<<s_eSDAPin);
	DrvTimer_WaitMicrosecondTmr2(u8ChangeTime);
	
	DrvGPIO_SetOutputBit(s_GpioPort[s_eSCKPort],DRVGPIO_PIN_0<<s_eSCKPin);
	DrvTimer_WaitMicrosecondTmr2(s_u16CycleTime-u8ChangeTime);	
	u16AckState = DrvGPIO_GetPinValue(s_GpioPort[s_eSDAPort],DRVGPIO_PIN_0<<s_eSDAPin);
	DrvTimer_WaitMicrosecondTmr2(u8ChangeTime);
	DrvGPIO_ClearOutputBit(s_GpioPort[s_eSCKPort],DRVGPIO_PIN_0<<s_eSCKPin);
	
	DrvGPIO_SetOutputBit(s_GpioPort[s_eSDAPort],DRVGPIO_PIN_0<<s_eSDAPin);
	DrvGPIO_SetIOModeEx(s_GpioPort[s_eSDAPort],(DRVGPIO_IOMODE_PIN0_OUT<<(s_eSDAPin<<1)), DRVGPIO_PIN_0<<s_eSDAPin);
	
	// Send Stop Condition to Device
	if(bStop)
        DrvI2C_SendStop();
    
    if(bCheckAck)
        if(u16AckState==0)
    		return TRUE;
    	else
    		return FALSE;
	else
    	if(u16AckState==1)
    		return TRUE;
    	else
    		return FALSE;   
}

/*---------------------------------------------------------------------------------------------------------*/
/* Function: DrvI2C_ReadByte                                                                               */
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
)
{
	UINT8 u8ReadData=0,u8DataCount,u8ChangeTime=s_u16CycleTime/4;
	
	// Send Start Condition to Device	
	if(bStart)
	    DrvI2C_SendStart();
	
	DrvGPIO_SetIOModeEx(s_GpioPort[s_eSDAPort],(DRVGPIO_IOMODE_PIN0_IN<<(s_eSDAPin<<1)), DRVGPIO_PIN_0<<s_eSDAPin);
	
	// Send Data to Device (MSB->LSB)
	for(u8DataCount=0;u8DataCount<8;u8DataCount++)
	{	
		u8ReadData = u8ReadData<<1;
		DrvGPIO_ClearOutputBit(s_GpioPort[s_eSCKPort],DRVGPIO_PIN_0<<s_eSCKPin);		
		DrvTimer_WaitMicrosecondTmr2(s_u16CycleTime);
		
		DrvGPIO_SetOutputBit(s_GpioPort[s_eSCKPort],DRVGPIO_PIN_0<<s_eSCKPin);
		DrvTimer_WaitMicrosecondTmr2(s_u16CycleTime-u8ChangeTime);
		
		if (DrvGPIO_GetPinValue(s_GpioPort[s_eSDAPort],DRVGPIO_PIN_0<<s_eSDAPin))
			u8ReadData = u8ReadData|0x01;
			
		DrvTimer_WaitMicrosecondTmr2(u8ChangeTime);
	}
	
	DrvGPIO_SetIOModeEx(s_GpioPort[s_eSDAPort],(DRVGPIO_IOMODE_PIN0_OUT<<(s_eSDAPin<<1)), DRVGPIO_PIN_0<<s_eSDAPin);
	
	// Send Ack from Device
	DrvGPIO_ClearOutputBit(s_GpioPort[s_eSCKPort],DRVGPIO_PIN_0<<s_eSCKPin);		
	DrvTimer_WaitMicrosecondTmr2(s_u16CycleTime-u8ChangeTime);
		
	if(bSendAck)
	{		
		DrvGPIO_ClearOutputBit(s_GpioPort[s_eSDAPort],DRVGPIO_PIN_0<<s_eSDAPin);		
		DrvTimer_WaitMicrosecondTmr2(u8ChangeTime);		
	}
	else
	{
		DrvGPIO_SetOutputBit(s_GpioPort[s_eSDAPort],DRVGPIO_PIN_0<<s_eSDAPin);		
		DrvTimer_WaitMicrosecondTmr2(u8ChangeTime);
	}
	
	DrvGPIO_SetOutputBit(s_GpioPort[s_eSCKPort],DRVGPIO_PIN_0<<s_eSCKPin);
	DrvTimer_WaitMicrosecondTmr2(s_u16CycleTime);
	
	DrvGPIO_ClearOutputBit(s_GpioPort[s_eSCKPort],DRVGPIO_PIN_0<<s_eSCKPin);		
	DrvTimer_WaitMicrosecondTmr2(s_u16CycleTime-u8ChangeTime);
	
	// Send Stop Condition to Device
	if(bStop)
        DrvI2C_SendStop();

	*pu8ReadData = u8ReadData;

    return TRUE;
}

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
DrvI2C_GetVersion(void)
{
	return DRVI2S_VERSION_NUM;
}
