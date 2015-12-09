#ifndef __SPI_H
#define __SPI_H

#include "Platform.h"

//
// Global Defines and Declarations
//

#define SPI_BODY_PACKET_SIGNAL	0x01


#define SPI_BUF_LENGTH 8

// Buffer data structure for head.
typedef __packed struct
{
	UINT8 status;
	UINT8 buffer[SPI_BUF_LENGTH];
} spiHeadData;

#define SPI_GPIO		GPIOB
#define SPI_CS_PIN		DRVGPIO_PIN_1
#define AUDIO_FILTER_MASK	0x30
//
// Global Functions
//


//
// open spi slave driver to talk to Sway Host
//
void spiSlave_Init(void);

void spiSlave_Write(UINT32 value);

// Interrupt handler for SPI packets from the body.
void read_and_write_SPI(void);

#endif // __SPI_H


