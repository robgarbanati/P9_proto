#ifndef __SPI_H
#define __SPI_H

#include "Platform.h"

//
// Global Defines and Declarations
//


// SPI master mode for communication with Cry Detect Board.
#define SPI_MASTER_HANDLER		DRVSPI_SPI0_HANDLER
//#define SPI_MASTER_DEVICE  		eDRVSPI_SLAVE_1
#define SPI_MASTER_DEVICE  		eDRVSPI_SLAVE_2
#define SPI_MASTER_DIVIDER    (_DRVSPI_DIVIDER(DrvCLK_GetHclk(), 100000))
#define SPI_MASTER_OPEN_FLAGS (	DRVSPI_ENDIAN_BIG | \
								DRVSPI_IDEL_CLK_LOW | \
								DRVSPI_MSB_FIRST | \
								DRVSPI_TX_1DATA | \
								DRVSPI_TX_POSEDGE | \
								DRVSPI_RX_NEGEDGE | \
								_DRVSPI_SLEEP(2) | \
								_DRVSPI_DATA_BITS(16))

// SPI slave mode for communication with N3290 State Machine.
#define SPI_SLAVE_HANDLER					DRVSPI_SPI1_HANDLER
#define SPI_SLAVE_DIVIDER     			(_DRVSPI_DIVIDER(DrvCLK_GetHclk(), 8000000))

#define SPI_SLAVE_OPEN_FLAGS 			(DRVSPI_ENDIAN_BIG | \
										 DRVSPI_MSB_FIRST | \
										 DRVSPI_TX_1DATA | \
										 DRVSPI_TX_NEGEDGE | \
										 DRVSPI_RX_POSEDGE | \
										 _DRVSPI_DATA_BITS(16))


#define SPI_BUF_LENGTH 8

#define SPI_SLAVE_GPIO			GPIOB
#define SPI_SLAVE_CS_PIN		DRVGPIO_PIN_1
#define SPI_SLAVE_SCK_PIN		DRVGPIO_PIN_2
#define SPI_SLAVE_DO_PIN		DRVGPIO_PIN_3
#define SPI_SLAVE_DI_PIN		DRVGPIO_PIN_4

#define LEFT_SAFETY_CLIP_FLAG_POSITION	5
#define RIGHT_SAFETY_CLIP_FLAG_POSITION	4

#define ONLINE_STATE	13
#define BASELINE 		11
#define STEPUP1 		2
#define STEPUP2 		3
#define STEPUP3 		5
#define STEPUP4			6

//
// Global Functions
//
UINT32 get_desired_speed(void);
UINT8 get_sway_state(void);
void set_sway_state(UINT8 newState);
float get_motor_PWM(void);
	
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

void spiSlave_Write(UINT32 value);

// Exchange single word with slave device
void spiMaster_Xchange(UINT16 TxData, UINT16 *RxData);


float get_frequency_from_state(void);
float get_amplitude_from_state(void);
void set_led_color_from_state(void);

#endif // __SPI_H
