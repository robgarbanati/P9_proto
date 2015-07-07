#include <string.h>
#include <stdio.h>
#include "Driver/DrvGPIO.h"
#include "Driver/DrvSPI.h"
#include "Driver/DrvSYS.h"
#include "Spi.h"
#include "Motor.h"
#include "PWM.h"
#include "Main.h"

//
// Global Variables
//
#define SPI_MAX_VALUE	0xFF
#define MOTOR_SPEED_DIVIDER 4  // We have a resolution of 0.25 hz
volatile UINT8 activity_button_pressed_flag = 0;

//
// Local Defines
//

// SPI master mode for communication with Cry Detect Board.
#define SPI_MASTER_HANDLER		DRVSPI_SPI0_HANDLER
#define SPI_MASTER_DEVICE  		eDRVSPI_SLAVE_1
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
										 
static UINT16 sway_state;
static UINT32 desired_speed;

//
// Global Functions
//

UINT8 get_safety_clip_flags(void)
{
	return (is_left_safety_clip_in() << LEFT_SAFETY_CLIP_FLAG_POSITION) | (is_right_safety_clip_in() << RIGHT_SAFETY_CLIP_FLAG_POSITION);
}

UINT32 get_desired_speed(void)
{
	return desired_speed;
}

UINT8 get_sway_state(void)
{
	return (UINT8) sway_state;
}

//
// Local Functions
//

void spiSlave_Init(void)
{
	// Open the SPI driver.
	DrvSPI_Open(SPI_SLAVE_HANDLER, SPI_SLAVE_OPEN_FLAGS, SPI_SLAVE_DIVIDER);
	
//	DrvSPI_SlaveSelect(SPI_SLAVE_HANDLER, TRUE, DRVSPI_IDEL_CLK_LOW);

	// Configure for slave mode.
	DrvSPI_SPI1_SetSlaveMode(TRUE);

	// Level trigger for slave mode.
	DrvSPI_SPI1_LevelTriggerInSlave(TRUE);

	// Set the zero status byte to shift out.
	DrvSPI_SingleWriteData0(SPI_SLAVE_HANDLER, (UINT32) 0x00);

	// Initiate the SPI transaction.
	DrvSPI_SetGo(SPI_SLAVE_HANDLER);

	// Enable interupt on SPI CS falling.
	DrvGPIO_SetFallingInt(&SPI_SLAVE_GPIO, SPI_SLAVE_CS_PIN, TRUE);
}

void spiSlave_Write(UINT32 value)
{
	// Set the data to shift out.
	DrvSPI_SingleWriteData0(SPI_SLAVE_HANDLER, (UINT32) value);

	// Initiate the next SPI transaction.
	DrvSPI_SetGo(SPI_SLAVE_HANDLER);
}

// Master talks to Cry Detect Board (CDB)
void spiMaster_Init(void) {
	// Open the SPI driver.
	DrvSPI_Open(SPI_MASTER_HANDLER, SPI_MASTER_OPEN_FLAGS, SPI_MASTER_DIVIDER);

	// Select the slave.
	DrvSPI_SlaveSelect(SPI_MASTER_HANDLER, TRUE, DRVSPI_IDEL_CLK_LOW);
	DrvSPI_SelectSlave(SPI_MASTER_HANDLER, SPI_MASTER_DEVICE);

	// Read/write data in 16 bit chunks.
	DrvSPI_SetDataConfig(SPI_MASTER_HANDLER, 1, 16);
}

void spiMaster_Write(UINT32 value)
{
	// Set the data to shift out of the SPI port.
	DrvSPI_SingleWriteData0(SPI_MASTER_HANDLER, (UINT32) value);
	
	// Initiate the SPI transaction.
	DrvSPI_SetGo(SPI_MASTER_HANDLER);
}

void spiMaster_Xchange(UINT16 TxData, UINT16 RxData)
{
	// Set the data to shift out of the SPI port.
	DrvSPI_SingleWriteData0(SPI_MASTER_HANDLER, (UINT32) TxData);
	
	// Initiate the SPI transaction.
	DrvSPI_SetGo(SPI_MASTER_HANDLER);
	
	// Wait while the SPI port is busy
	while (DrvSPI_GetBusy(SPI_MASTER_HANDLER));
	
	// Read the value shifted in
	DrvSPI_SingleReadData0(SPI_MASTER_HANDLER);
}

// Handle the send/receive of the SPI packets at interrupt time.
// On the order of 10 to 100 microseconds
void read_and_write_SPI(void)
{
	UINT8 index = 0;
	UINT16 spiMaster_Data[SPI_BUF_LENGTH];
	UINT16 spiSlave_Data[SPI_BUF_LENGTH];
	
	// Assume we receive a zero length packet.
	spiSlave_Data[0] = 0;
	spiMaster_Data[0] = 0;

	// Send/receive bytes until the SPI CS pin is raised.
	while (!DrvGPIO_GetInputPinValue(&SPI_SLAVE_GPIO, SPI_SLAVE_CS_PIN))
	{
		// Wait while the SPI ports are busy and the SPI CS line is low.
		while (DrvSPI_GetBusy(SPI_SLAVE_HANDLER) && !DrvGPIO_GetInputPinValue(&SPI_SLAVE_GPIO, SPI_SLAVE_CS_PIN));  // && DrvSPI_GetBusy(SPI_MASTER_HANDLER)

		// Process the next byte if the SPI CS line is still low.
		if (!DrvGPIO_GetInputPinValue(&SPI_SLAVE_GPIO, SPI_SLAVE_CS_PIN))
		{
			// Read the values shifted in.
			spiSlave_Data[index] = DrvSPI_SingleReadData0(SPI_SLAVE_HANDLER);
			spiMaster_Data[index] = DrvSPI_SingleReadData0(SPI_MASTER_HANDLER);
			
			// Interpret messages
			desired_speed = (spiSlave_Data[index] & 0x00FF);
			sway_state = (spiSlave_Data[index] >> 8) & 0x0F;
			
			// Daisy-chain: Pass slave values (recvd from Linux master) to master (to Cry Detect Board) and vice versa.
			spiSlave_Write(spiMaster_Data[index] | activity_button_pressed_flag);
			spiMaster_Write(spiSlave_Data[index]);
			
			// Increment the index, but prevent overflow.
			if (index < SPI_BUF_LENGTH) ++index;
		}
	}

	// Initialize the first zero status byte to shift out on the next packet.
	spiSlave_Write(spiMaster_Data[index-1] | activity_button_pressed_flag | get_safety_clip_flags());
	spiMaster_Write(spiSlave_Data[index-1]);
	
	activity_button_pressed_flag = 0;
}

