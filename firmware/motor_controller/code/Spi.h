#ifndef __SPI_H
#define __SPI_H

#include "Platform.h"

//
// Global Defines and Declarations
//

#define SPI_BUF_LENGTH 8

#define SPI_SLAVE_GPIO			GPIOB
#define SPI_SLAVE_CS_PIN		DRVGPIO_PIN_1
#define SPI_SLAVE_SCK_PIN		DRVGPIO_PIN_2
#define SPI_SLAVE_DO_PIN		DRVGPIO_PIN_3
#define SPI_SLAVE_DI_PIN		DRVGPIO_PIN_4

#define LEFT_SAFETY_CLIP_FLAG_POSITION	6
#define RIGHT_SAFETY_CLIP_FLAG_POSITION	5

//
// Global Functions
//
UINT32 get_desired_speed(void);
UINT8 get_sway_state(void);
	
void spiSlave_Init(void);
void spiMaster_Init(void);

void spiSlave_Close(void);
void spiMaster_Close(void);

void spiMaster_Write(UINT32 value);

// return 8 bits read from spi.
static UINT8 spiMaster_Read8(void);

// write a value of 8 bits to an address
void spiMaster_Write8(UINT8 value);

// Interrupt handler for SPI packets from the N3290.
void read_and_write_SPI(void);

// Exchange single word with slave device
void spiMaster_Xchange(UINT16 TxData, UINT16 RxData);

#endif // __SPI_H


