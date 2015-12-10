/*
 * drv8301.c
 *
 * Created: 1/21/2015 9:30:27 PM
 *  Author: Marko
 */
 
#include "Platform.h"

#include <stdint.h>
#include "Driver/DrvSPI.h"
#include "Spi.h"
#include "app_drv8301.h"

void configure_spi_master(void)
{
	spiMaster_Init();
}

uint16_t sendData(uint16_t Data)
{
	uint16_t Result;
	uint16_t txData, rxData;
	 
	txData  = ( Data >> 8) & 0x00FF;
	txData += (Data & 0x00FF) << 8;
	
	spiMaster_Write(Data);
	
	Result  = ( rxData >> 8) & 0x00FF;
	Result += (rxData & 0x00FF) << 8;
	 
	return Result;
}

void configure_DRV8301(void)
{
	int i;
	uint16_t CtrlReg = 0x00;	
	
	CtrlReg  = (0x00 << DRV8301_OC_MODE) || (0x01 << DRV8301_PWM_MODE) || (0x00 << DRV8301_GATE_RESET) || (0x02 << DRV8301_GATE_CURRENT);
	CtrlReg |= (DRV8301_CONTROL_REG_1 << DRV8301_DATA_BITS);
	CtrlReg |= (DRV8301_WRITE_CMD << 15);
	
	CtrlReg   = 0x17CA; // TODO use control flags.
	
	// It doesn't always work, so waiting a moment then doing it 1000 times makes it pretty damn foolproof. And we have plenty of time while the linux boots up.
	for(i=0;i<100000;i++);
	for(i=0;i<1000;i++)
	{
		while(DrvSPI_GetBusy(SPI_MASTER_HANDLER));
		CtrlReg   = 0x17CA;
		sendData(CtrlReg);
	}
}

uint16_t getStatus_DRV8301(void)
{
	uint16_t Cmd = 0x00;
	uint16_t Res;
	
	Cmd = 0x8000;	
	Res = sendData(Cmd);
	
	Cmd = 0x8000;	
	Res = sendData(Cmd);
	
	return Res;
}

uint16_t getCtrlReg_DRV8301(uint8_t Reg)
{
	uint16_t Cmd = 0x00;
	uint16_t Res;
	
	if (Reg == 1)
	{
		Cmd = 0x9000;
	} else if (Reg == 2)
	{
		Cmd = 0x9800;
	} else {
		return 0;
	}
		
	Res = sendData(Cmd);		
	Res = sendData(Cmd);
	
	return Res;
}

void init_DRV8301(void)
{
	configure_spi_master();
	configure_DRV8301();
}
