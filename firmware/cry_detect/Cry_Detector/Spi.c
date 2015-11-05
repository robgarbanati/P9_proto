#include <string.h>
#include <stdio.h>
#include "Driver/DrvGPIO.h"
#include "Driver/DrvSPI.h"
#include "Driver/DrvSYS.h"
#include "Spi.h"

//
// Global Variables
//
extern volatile UINT16 cry_volume;
extern volatile uint16_t audio_filter;
extern volatile int filt_amp_ave_B;
//
// Local Defines
//

// SPI slave mode for communication with robot body.
#define SPI_SLAVE_HANDLER					DRVSPI_SPI1_HANDLER
#define SPI_SLAVE_DIVIDER     			(_DRVSPI_DIVIDER(DrvCLK_GetHclk(), 1))

#define SPI_SLAVE_OPEN_FLAGS 			(DRVSPI_ENDIAN_BIG | \
										 DRVSPI_MSB_FIRST | \
										 DRVSPI_TX_1DATA | \
										 DRVSPI_TX_POSEDGE | \
										 DRVSPI_RX_NEGEDGE | \
										 _DRVSPI_DATA_BITS(16))																	 

//
// Local Functions
//


//
// open spi slave driver to talk to Sway Host
//
void spiSlave_Init(void)
{
	// Open the SPI driver.
	DrvSPI_Open(SPI_SLAVE_HANDLER, SPI_SLAVE_OPEN_FLAGS, SPI_SLAVE_DIVIDER);

	// Configure for slave mode.
	DrvSPI_SPI1_SetSlaveMode(TRUE);

	// Level trigger for slave mode.
	DrvSPI_SPI1_LevelTriggerInSlave(TRUE);

	// Set the zero status byte to shift out.
	DrvSPI_SingleWriteData0(SPI_SLAVE_HANDLER, (UINT32) 0x00);

	// Initiate the SPI transaction.
	DrvSPI_SetGo(SPI_SLAVE_HANDLER);

	// Enable interupt on SPI CS falling.
	DrvGPIO_SetFallingInt(&SPI_GPIO, SPI_CS_PIN, TRUE);
}

// 
// write a value of 16 bits to Sway Hoste
//
void spiSlave_Write(UINT32 value)
{
	// Set the data to shift out.
	DrvSPI_SingleWriteData0(SPI_SLAVE_HANDLER, (UINT32) value);

	// Initiate the next SPI transaction.
	DrvSPI_SetGo(SPI_SLAVE_HANDLER);
}

//
// Handle the send/receive of the SPI packets at interrupt time.
// On the order of 10 to 100 microseconds
//
void read_and_write_SPI(void)
{
//	UINT8 index = 0;
	
	// Send/receive bytes until the SPI CS pin is raised.
	while (!DrvGPIO_GetInputPinValue(&SPI_GPIO, SPI_CS_PIN))
	{
		// Wait while the SPI ports are busy and the SPI CS line is low.
		while (DrvSPI_GetBusy(SPI_SLAVE_HANDLER) && !DrvGPIO_GetInputPinValue(&SPI_GPIO, SPI_CS_PIN));  // && DrvSPI_GetBusy(SPI_MASTER_HANDLER)

		// Process the next byte if the SPI CS line is still low.
		if (!DrvGPIO_GetInputPinValue(&SPI_GPIO, SPI_CS_PIN))
		{
			// Read the value shifted in.
 			audio_filter = DrvSPI_SingleReadData0(SPI_SLAVE_HANDLER);

//			// Increment the index, but prevent overflow.
//			if (index < SPI_BUF_LENGTH) ++index;
		}
	}
	// Initialize the first zero status byte to shift out on the next packet.
	spiSlave_Write(cry_volume);
}
