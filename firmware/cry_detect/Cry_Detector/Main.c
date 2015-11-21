#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <math.h>
#include "Driver/DrvGPIO.h"
#include "Driver/DrvSYS.h"
#include "Driver/DrvCLK.h"
#include "Driver/DrvTimer.h"
#include "Commu/SoftUART.h"
#include "SysClkConfig.h"
#include "DirDetect.h"
#include "Spi.h"
#include "../Shared/Nuvoton/XPrintf.h"
#include "DirDetect.h"



// Handle the GPIO interrupt.
void GPAB_IRQHandler(void)
{
	// Is this SPI CS?
	if (DrvGPIO_GetIntFlag(&SPI_GPIO, SPI_CS_PIN))
	{
		// Handle send/receive of packets from the robot body.
		read_and_write_SPI();
		
		// Clear the SPI CS interrupt.
		DrvGPIO_ClearIntFlag(&SPI_GPIO, SPI_CS_PIN);
	}
}

// Init ADC clock
void adcClockInit()
{
	UINT32	u32HCLK = 0,u32ADCClk = 0;
// 	INT32 adc_clk_freq, sample_freq;
	
	DrvCLK_SetClkSrcAdc(eDRVCLK_ADCSRC_48M);
// 	adc_clk_freq = ADC_CLOCK_FREQUENCY;
//	sample_freq = SAMPLE_FREQUENCY;
	
	DrvCLK_SetClkDividerAdc(48000000/ADC_CLOCK_FREQUENCY - 1);  // == sampling_frequency*25
	
	u32HCLK = DrvCLK_GetHclk();
	u32ADCClk = DrvCLK_GetClkAdc();

//	printf("adc_clk_f should be %d, is %d . sample_f should be %d, is %d. hclk is %d\n", adc_clk_freq, u32ADCClk, sample_freq, u32ADCClk/CYCLES_PER_CONVERSION/NUM_CHANNELS, u32HCLK);
	
	//The ADC engine clock must meet the constraint: ADCLK <=  HCKL/2.
	if (u32ADCClk>(u32HCLK/2)) {
//		puts("ADCClk is greater than half the frequency of the HCLK. That's bad.\n");
//		puts("Check adcClockInit() in Main.c\n");
	}
}

// Initialize system clock.
void clkInit(void)
{
	// Unlock protected registers.
	DrvSYS_UnlockKeyReg();

	// Configure the system clock source.  See SysClkConfig.h.
	_SYSCLKCONFIG();

	// Enable LDO33.
	DrvCLK_EnableLDO30_P();
	
	// init ADC clock
	adcClockInit();

	// Lock protected registers.
	DrvSYS_LockKeyReg();
}

// Initialize interrupt priorities.
void priorityInit(void)
{
	// Set the SPI slave interrupt priority high.
	NVIC_SetPriority(SPI1_IRQn, 0);

	// Set the GPIO interrupt priority high. (low now)
	NVIC_SetPriority(GPAB_IRQn, 0);

	// Set the ADC interrupt lower than SPI and GPIO.
	NVIC_SetPriority(ADC_IRQn, 1);
}

// Initialize system GPIO.
void gpioInit(void)
{
	// Configure GPIO A special functions.
	DrvSYS_EnableMultifunctionGpioa(
		DRVSYS_GPIOA_MF1_SPI0_1ST_CHIP_SEL_OUT |	// Master SPI select output serial flash
		DRVSYS_GPIOA_MF2_SPI0_CLOCK_OUT |					// Master SPI clock output
		DRVSYS_GPIOA_MF3_SPI0_DATA_IN |						// Master SPI data input
		DRVSYS_GPIOA_MF4_SPI0_DATA_OUT |					// Master SPI data output
		
		DRVSYS_GPIOA_MF8_ADC_CHANNEL0_IN|  	// ADC input
		DRVSYS_GPIOA_MF9_ADC_CHANNEL1_IN|  	// ADC input
		DRVSYS_GPIOA_MF10_ADC_CHANNEL2_IN|  // ADC input
		DRVSYS_GPIOA_MF11_ADC_CHANNEL3_IN|  // ADC input
		DRVSYS_GPIOA_MF12_ADC_CHANNEL4_IN|  // ADC input
		DRVSYS_GPIOA_MF13_ADC_CHANNEL5_IN  	// ADC input
		);
	
		// Configure GPIO B special functions.
	DrvSYS_EnableMultifunctionGpiob(
		DRVSYS_GPIOB_MF1_SPI1_1ST_CHIP_SEL_OUT |	// Slave SPI select input
		DRVSYS_GPIOB_MF2_SPI1_CLOCK_OUT |			// Slave SPI clock input
		DRVSYS_GPIOB_MF3_SPI1_DATA_IN |				// Slave SPI data output
		DRVSYS_GPIOB_MF4_SPI1_DATA_OUT 				// Slave SPI data input
	);
		
		
	// Configure GPIO port A pins.
	DrvGPIO_SetIOMode(&GPIOA, 
		DRVGPIO_IOMODE_PIN0_QUASI|
		DRVGPIO_IOMODE_PIN1_QUASI|
		DRVGPIO_IOMODE_PIN2_QUASI|
		DRVGPIO_IOMODE_PIN3_QUASI|
		DRVGPIO_IOMODE_PIN4_QUASI|
		DRVGPIO_IOMODE_PIN5_QUASI|
		DRVGPIO_IOMODE_PIN6_QUASI|
		DRVGPIO_IOMODE_PIN7_QUASI|
		DRVGPIO_IOMODE_PIN11_QUASI
		
	);

	// Configure GPIO port B pins.
	DrvGPIO_SetIOMode(&GPIOB,
		DRVGPIO_IOMODE_PIN6_QUASI|
		DRVGPIO_IOMODE_PIN7_QUASI|
//		DRVGPIO_IOMODE_PIN8_IN| // TODO: change for PWM
//		DRVGPIO_IOMODE_PIN9_IN|
		DRVGPIO_IOMODE_PIN8_QUASI| // TODO: change for PWM
		DRVGPIO_IOMODE_PIN9_QUASI|
		DRVGPIO_IOMODE_PIN10_QUASI|
		DRVGPIO_IOMODE_PIN11_QUASI
	);
	
	// Enable GPIO interrupt routine.
	NVIC_EnableIRQ(GPAB_IRQn);

	// Release the slave select line (inactive high) to the SPI slaves.
	// Even if we aren't using a peripheral we should release the slave
	// select line to make sure the device doesn't put data on the bus.
//	DrvGPIO_SetOutputBit(&SPI_SFLASH_GPIO, SPI_SFLASH_SS_PIN);
}

// Initializet the soft UART.
void uartInit(void)
{
	S_SOFTUART_INTF uartIntf;
	S_SOFTUART_UART_CONF uartConf;	

	// Configure the soft uart configuration.
	uartConf.u32BaudRate = SOFTUART_BAUD_230400; //SOFTUART_BAUD_9600;
	uartConf.u8DataBits = SOFTUART_DATABITS_8;
	uartConf.u8Parity = SOFTUART_PARITY_NONE;
	uartConf.u8StopBits = SOFTUART_STOPBITS_1;

	// The soft uart only supports SPI1 for N572F072.
	uartIntf.psTimer = (TMR_T *) TMR0_BASE;
	uartIntf.sSPIIntf.psSPIIntf = (SPI_T *) SPI1_BASE;

	// Open the soft uart driver.
	SoftUART_Open(&uartConf, &uartIntf, 0);
}

// Main thread.
int main (void)
{
	// Initialize clocks.
 	clkInit();

	// Initialize interrupt priorities.
	priorityInit();

	// Initialize GPIO.
	gpioInit();
	
	// Initialize ADC and DirDetect event handler
	dirDetectInit();
	
	// Initialize SPI.
//	spiSlave_Init();

	for (;;)
	{	
		Do_Loop();
	}
}

