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
	
	spiMaster_Xchange(Data, &Result);
	 
	return Result;
}

void configure_DRV8301(void)
{
	int i;
	uint16_t CtrlReg = 0x00;	
	
	CtrlReg  = (DRV8301_GATE_PEAK_CURRENT_250mA << DRV8301_GATE_CURRENT_POS);
	CtrlReg |= (DRV8301_GATE_RESET              << DRV8301_GATE_RESET_POS);
	CtrlReg |= (DRV8301_PWM_MODE_3_INPUTS       << DRV8301_PWM_MODE_POS);	
	CtrlReg |= (DRV8301_OC_MODE_REPORT_ONLY     << DRV8301_OC_MODE_POS);
	CtrlReg |= (DRV8301_OC_ADJ_SET_VDS_60mV     << DRV8301_OC_ADJ_SET_POS);
	
	CtrlReg |= (DRV8301_CONTROL_REG_1           << DRV8301_ADDRESS_BIT_POS);
	
	CtrlReg |= (DRV8301_WRITE_CMD               << DRV8301_RW_BIT_POS);
	
	for(i=0;i<1000;i++) // It doesn't always work, so doing it 1000 times makes it pretty damn foolproof. And we have plenty of time while the linux boots up.
	{
		while(DrvSPI_GetBusy(SPI_MASTER_HANDLER));
		sendData(CtrlReg);
	}
}

uint16_t getReg_DRV8301(uint8_t Reg)
{
	uint16_t Cmd = 0x00;
	uint16_t Res;
	
	Cmd  = (Reg << DRV8301_ADDRESS_BIT_POS);
	Cmd |= (DRV8301_READ_CMD << DRV8301_RW_BIT_POS);
	
	while(DrvSPI_GetBusy(SPI_MASTER_HANDLER));	
	Res = sendData(Cmd);		
	while(DrvSPI_GetBusy(SPI_MASTER_HANDLER));
	Res = sendData(Cmd);
	
	return Res;
}

void init_DRV8301(void)
{
	configure_spi_master();
	configure_DRV8301();
}
