#include <string.h>
#include "global.h"
#include "app_util.h"
#include "nrf_soc.h"
#include "nrf_gpio.h"
#include "nrf_gpiote.h"
#include "vuart.h"
#include "btle_queue.h"
#include "spi_master.h"

//
// Global Variables
//

// SPI thread ID.
osThreadId spiThreadId;

// SPI master bus mutex.
osMutexId spiMasterMutex;
osMutexDef(spiMasterMutex);

// SPI queue mutex.
static osMutexId spiQueueMutex;
osMutexDef(spiQueueMutex);

//
// Local Defines
//

// This value yields about a 5 millisecond wait time for the master
// to wait for the slave to assert the master select line.
#define SPI_MASTER_SELECT_WAIT				2500

//
// Send/Receive Queues and Flags
//

// Must be a power of two.
#define SPI_SEND_QUEUE_SIZE 8
#define SPI_SEND_QUEUE_MASK (SPI_SEND_QUEUE_SIZE - 1)

// Send spi queue state variables.
static UINT8 spiSendQueueTail = 0;
static UINT8 spiSendQueueHead = 0;
static PACKETData spiSendQueue[SPI_SEND_QUEUE_SIZE];

//
// Local Functions
//

// Function for handling GPIOTE interrupt. Triggered when master select
// goes from high-to-low transition.
void GPIOTE_IRQHandler(void)
{
	// Is this a high-to-low event transition on the master select?
	if ((NRF_GPIOTE->EVENTS_IN[0] == 1) && (NRF_GPIOTE->INTENSET & GPIOTE_INTENSET_IN0_Msk))
	{
		// Clear the interrupt.
		NRF_GPIOTE->EVENTS_IN[0] = 0;

		// Notify the SPI thread to process the packet.
		osSignalSet(spiThreadId, 0x01);
	}
}

// Returns true if the slave is requesting service from the master.
static bool spi_nvt1_ready(void)
{
	// Return true if the master select is asserted (active low).
	return nrf_gpio_pin_read(SPI0_NVT1_MS_PIN) ? false : true;	
}

// Transfer a multi-byte packet to and from the NVT1 slave.
static bool spi_nvt1_write_read(const PACKETData *write_data, PACKETData *read_data)
{
	uint8_t xfer_count;
	uint8_t read_index = 0;
	uint8_t write_index = 0;
	uint32_t wait_count;

	// Reset the read data length.
	read_data->length = 0;

	// Wait for exclusive access to the SPI master bus.
	osMutexWait(spiMasterMutex, osWaitForever);

	// Assert the slave select line (active low) to the slave.
  nrf_gpio_pin_clear(SPI0_NVT1_SS_PIN);

	// Wait for assertion of master select to indicate slave is ready.
	for (wait_count = 0; (wait_count < SPI_MASTER_SELECT_WAIT) && nrf_gpio_pin_read(SPI0_NVT1_MS_PIN); ++wait_count) { __NOP(); __NOP(); }

	// Was the master select asserted before the wait expired?
	if (wait_count < SPI_MASTER_SELECT_WAIT)
	{
		// Configure the SPI0 port for communication with NVT1.
		spi_configure(SPI0, SPI_MODE0, false, Freq_1Mbps);

		// Send length and discard the first byte from the slave.
		spi_write_read_slave_8(SPI0, write_data->length);

		// Send checksum, receive length from the slave.
		read_data->length = spi_write_read_slave_8(SPI0, write_data->checksum);

		// Send first byte, receive checksum from the slave.
		read_data->checksum = spi_write_read_slave_8(SPI0, write_data->buffer[write_index++]);

		// Sanity check the head buffer count.  While under development
		// the head may not be attached and we need to clean up bogus counts.
		if (read_data->length > PACKET_BUF_LENGTH) read_data->length = 0;

		// Determine the maximum number of bytes to transfer.
		if (write_data->length == 0)
		{
			// Use the read data to set the transfer count.
			xfer_count = read_data->length;
		}
		else if (read_data->length > (write_data->length - 1))
		{
			// Use the read data to set the transfer count.
			xfer_count = read_data->length;
		}
		else
		{
			// Use the write data to set the transfer count.
			xfer_count = write_data->length - 1;
		}

		// Sanity check the maximum number of bytes to send.
		if (xfer_count > PACKET_BUF_LENGTH) xfer_count = PACKET_BUF_LENGTH;

		// Transfer the rest of buffer.
		while (read_index < xfer_count)
		{
			// Send the indexed byte from the buffer.
			read_data->buffer[read_index++] =  spi_write_read_slave_8(SPI0, write_data->buffer[write_index++]);

			// Make sure the write index doesn't overflow.  This can happen because
			// we may have to send an extra byte to receive a full packet from the slave.
			if (write_index >= PACKET_BUF_LENGTH) write_index = PACKET_BUF_LENGTH - 1;
		}
	}

	// Release the slave select line (inactive high) to the slave.
	nrf_gpio_pin_set(SPI0_NVT1_SS_PIN);

	// Release exclusive access to the SPI master bus.
	osMutexRelease(spiMasterMutex);

	// Return true if we were passed back data to be processed.
	return read_data->length && packet_validate_checksum(read_data);
}

//
// Global Functions
//

void spi_init(void)
{
	// Create the SPI mutex.
	spiQueueMutex = osMutexCreate(osMutex(spiQueueMutex));
	spiMasterMutex = osMutexCreate(osMutex(spiMasterMutex));
	
	// We don't actually configure the SPI peripheral below, but rather the
	// GPIO pin configuration related to the SPI devices.  The SPI peripheral
	// will be configured before each device is communicated with.
	
	// Configure SPI0 GPIO pins.
	nrf_gpio_cfg_output(SPI0_MOSI_PIN);
	nrf_gpio_cfg_input(SPI0_MISO_PIN, NRF_GPIO_PIN_NOPULL);
	nrf_gpio_cfg_output(SPI0_CLK_PIN);

	// Configure NVT1 slave and master select GPIO pins.
	nrf_gpio_cfg_output(SPI0_NVT1_SS_PIN);
	nrf_gpio_cfg_input(SPI0_NVT1_MS_PIN, NRF_GPIO_PIN_NOPULL);
	
	// Configure serial flash slave select GPIO pin.
	nrf_gpio_cfg_output(SPI0_FLASH_SS_PIN);

	// Disable slave select (inactive high).
	nrf_gpio_pin_set(SPI0_NVT1_SS_PIN);
	nrf_gpio_pin_set(SPI0_FLASH_SS_PIN);
}

void spi_configure(SPIModuleNumber module, uint8_t spi_mode, uint8_t lsb_first, uint8_t spi_freq)
{
	uint32_t config_mode;
  NRF_SPI_Type *spi_base;

	// Set the SPI base to use.
	spi_base = (module == SPI0) ? NRF_SPI0 : NRF_SPI1;

	// Configure the GPIO pins associated with the SPI port.
	if (module == SPI0)
	{
		// Configure SPI pins
		spi_base->PSELSCK  = SPI0_CLK_PIN;
		spi_base->PSELMOSI = SPI0_MOSI_PIN;
		spi_base->PSELMISO = SPI0_MISO_PIN;
	}
	else
	{
		// Configure SPI pins
		spi_base->PSELSCK  = SPI1_CLK_PIN;
		spi_base->PSELMOSI = SPI1_MOSI_PIN;
		spi_base->PSELMISO = SPI1_MISO_PIN;
	}

	// Configure SPI frequency.
	spi_base->FREQUENCY = (0x02000000UL << (uint32_t) spi_freq);

	// Clock polarity 0. Clock starts with level 0. Sample data at 
	// rising edge of clock and shift serial data at falling edge.
	switch (spi_mode)
	{
		case SPI_MODE0:
				config_mode = (SPI_CONFIG_CPHA_Leading << SPI_CONFIG_CPHA_Pos) | (SPI_CONFIG_CPOL_ActiveHigh << SPI_CONFIG_CPOL_Pos);
				break;
		case SPI_MODE1:
				config_mode = (SPI_CONFIG_CPHA_Trailing << SPI_CONFIG_CPHA_Pos) | (SPI_CONFIG_CPOL_ActiveHigh << SPI_CONFIG_CPOL_Pos);
				break;
		case SPI_MODE2:
				config_mode = (SPI_CONFIG_CPHA_Leading << SPI_CONFIG_CPHA_Pos) | (SPI_CONFIG_CPOL_ActiveLow << SPI_CONFIG_CPOL_Pos);
				break;
		case SPI_MODE3:
				config_mode = (SPI_CONFIG_CPHA_Trailing << SPI_CONFIG_CPHA_Pos) | (SPI_CONFIG_CPOL_ActiveLow << SPI_CONFIG_CPOL_Pos);
				break;
		default:
				config_mode = 0;
				break;
		/*lint -restore */
	}

	if (lsb_first)
	{
		config_mode |= (SPI_CONFIG_ORDER_LsbFirst << SPI_CONFIG_ORDER_Pos);
	}
	else
	{
		config_mode |= (SPI_CONFIG_ORDER_MsbFirst << SPI_CONFIG_ORDER_Pos);
	}

	// Configure the SPI port.
	spi_base->CONFIG = config_mode;

	// Enable SPI communication.
	spi_base->EVENTS_READY = 0U;
	spi_base->ENABLE = (SPI_ENABLE_ENABLE_Enabled << SPI_ENABLE_ENABLE_Pos);
}


uint8_t spi_write_read_slave_8(SPIModuleNumber module, uint8_t value)
{
	// Set the SPI base to use.
	NRF_SPI_Type *spi_base = (module == SPI0) ? NRF_SPI0 : NRF_SPI1;

	// Clear the event.
	spi_base->EVENTS_READY = 0U;

	// Write the outgoing data into the SPI transmit register.
  spi_base->TXD = (uint32_t) value;

	// Wait for the transaction to complete.
  while (spi_base->EVENTS_READY == 0U);

	// Return the incoming data from the SPI receive register.
	return spi_base->RXD;
}

uint16_t spi_write_read_slave_16(SPIModuleNumber module, uint16_t out_value)
{
	uint16_t in_value;
	
	// Set the SPI base to use.
	NRF_SPI_Type *spi_base = (module == SPI0) ? NRF_SPI0 : NRF_SPI1;

	// LSB transfer?
	if (spi_base->CONFIG & (SPI_CONFIG_ORDER_LsbFirst << SPI_CONFIG_ORDER_Pos))
	{
		// Yes.
		NRF_SPI0->EVENTS_READY = 0U;
		spi_base->TXD = (uint32_t) ((uint8_t *) &out_value)[0];
		spi_base->TXD = (uint32_t) ((uint8_t *) &out_value)[1];
		while (spi_base->EVENTS_READY == 0U);
		spi_base->EVENTS_READY = 0U;
		((uint8_t *) &in_value)[0] = (uint8_t) spi_base->RXD;
		while (spi_base->EVENTS_READY == 0U);
		((uint8_t *) &in_value)[1] = (uint8_t) spi_base->RXD;
	}
	else
	{
		// No.
		NRF_SPI0->EVENTS_READY = 0U;
		spi_base->TXD = (uint32_t) ((uint8_t *) &out_value)[1];
		spi_base->TXD = (uint32_t) ((uint8_t *) &out_value)[0];
		while (spi_base->EVENTS_READY == 0U);
		spi_base->EVENTS_READY = 0U;
		((uint8_t *) &in_value)[1] = (uint8_t) spi_base->RXD;
		while (spi_base->EVENTS_READY == 0U);
		((uint8_t *) &in_value)[0] = (uint8_t) spi_base->RXD;
	}

	return in_value;
}

uint32_t spi_write_read_slave_24(SPIModuleNumber module, uint32_t out_value)
{
	uint32_t in_value;
	
	// Set the SPI base to use.
	NRF_SPI_Type *spi_base = (module == SPI0) ? NRF_SPI0 : NRF_SPI1;

	// LSB transfer?
	if (spi_base->CONFIG & (SPI_CONFIG_ORDER_LsbFirst << SPI_CONFIG_ORDER_Pos))
	{
		// Yes.
		NRF_SPI0->EVENTS_READY = 0U;
		spi_base->TXD = (uint32_t) ((uint8_t *) &out_value)[0];
		spi_base->TXD = (uint32_t) ((uint8_t *) &out_value)[1];
		while (spi_base->EVENTS_READY == 0U);
		spi_base->EVENTS_READY = 0U;
		((uint8_t *) &in_value)[0] = (uint8_t) spi_base->RXD;
		spi_base->TXD = (uint32_t) ((uint8_t *) &out_value)[2];
		while (spi_base->EVENTS_READY == 0U);
		spi_base->EVENTS_READY = 0U;
		((uint8_t *) &in_value)[1] = (uint8_t) spi_base->RXD;
		while (spi_base->EVENTS_READY == 0U);
		((uint8_t *) &in_value)[2] = (uint8_t) spi_base->RXD;
	}
	else
	{
		// No.
		NRF_SPI0->EVENTS_READY = 0U;
		spi_base->TXD = (uint32_t) ((uint8_t *) &out_value)[2];
		spi_base->TXD = (uint32_t) ((uint8_t *) &out_value)[1];
		while (spi_base->EVENTS_READY == 0U);
		spi_base->EVENTS_READY = 0U;
		((uint8_t *) &in_value)[2] = (uint8_t) spi_base->RXD;
		spi_base->TXD = (uint32_t) ((uint8_t *) &out_value)[0];
		while (spi_base->EVENTS_READY == 0U);
		spi_base->EVENTS_READY = 0U;
		((uint8_t *) &in_value)[1] = (uint8_t) spi_base->RXD;
		while (spi_base->EVENTS_READY == 0U);
		((uint8_t *) &in_value)[0] = (uint8_t) spi_base->RXD;
	}

	return in_value;
}

uint32_t spi_write_read_slave_32(SPIModuleNumber module, uint32_t out_value)
{
	uint32_t in_value;
	
	// Set the SPI base to use.
	NRF_SPI_Type *spi_base = (module == SPI0) ? NRF_SPI0 : NRF_SPI1;

	// LSB transfer?
	if (spi_base->CONFIG & (SPI_CONFIG_ORDER_LsbFirst << SPI_CONFIG_ORDER_Pos))
	{
		// Yes.
		NRF_SPI0->EVENTS_READY = 0U;
		spi_base->TXD = (uint32_t) ((uint8_t *) &out_value)[0];
		spi_base->TXD = (uint32_t) ((uint8_t *) &out_value)[1];
		while (spi_base->EVENTS_READY == 0U);
		spi_base->EVENTS_READY = 0U;
		((uint8_t *) &in_value)[0] = (uint8_t) spi_base->RXD;
		spi_base->TXD = (uint32_t) ((uint8_t *) &out_value)[2];
		while (spi_base->EVENTS_READY == 0U);
		spi_base->EVENTS_READY = 0U;
		((uint8_t *) &in_value)[1] = (uint8_t) spi_base->RXD;
		spi_base->TXD = (uint32_t) ((uint8_t *) &out_value)[3];
		while (spi_base->EVENTS_READY == 0U);
		spi_base->EVENTS_READY = 0U;
		((uint8_t *) &in_value)[2] = (uint8_t) spi_base->RXD;
		while (spi_base->EVENTS_READY == 0U);
		((uint8_t *) &in_value)[3] = (uint8_t) spi_base->RXD;
	}
	else
	{
		// No.
		NRF_SPI0->EVENTS_READY = 0U;
		spi_base->TXD = (uint32_t) ((uint8_t *) &out_value)[3];
		spi_base->TXD = (uint32_t) ((uint8_t *) &out_value)[2];
		while (spi_base->EVENTS_READY == 0U);
		spi_base->EVENTS_READY = 0U;
		((uint8_t *) &in_value)[3] = (uint8_t) spi_base->RXD;
		spi_base->TXD = (uint32_t) ((uint8_t *) &out_value)[1];
		while (spi_base->EVENTS_READY == 0U);
		spi_base->EVENTS_READY = 0U;
		((uint8_t *) &in_value)[2] = (uint8_t) spi_base->RXD;
		spi_base->TXD = (uint32_t) ((uint8_t *) &out_value)[0];
		while (spi_base->EVENTS_READY == 0U);
		spi_base->EVENTS_READY = 0U;
		((uint8_t *) &in_value)[1] = (uint8_t) spi_base->RXD;
		while (spi_base->EVENTS_READY == 0U);
		((uint8_t *) &in_value)[0] = (uint8_t) spi_base->RXD;
	}

	return in_value;
}


// Returns true if there are packets ready to send.
BOOL spi_send_queue_ready(void)
{
	return spiSendQueueTail != spiSendQueueHead;
}


// Put a packet on the SPI send queue.
// Returns TRUE if the packet put succeeded.
BOOL spi_put_send_queue(const PACKETData *spi)
{
	BOOL status = FALSE;

	// Sanity check the packet length.
	if ((spi->length >= 1) && (spi->length <= PACKET_BUF_LENGTH))
	{
		// Wait for exclusive access to the send queue.
		osMutexWait(spiQueueMutex, osWaitForever);

		// If there is no room in the queue the caller will have to try again.
		if (((spiSendQueueHead + 1) & SPI_SEND_QUEUE_MASK) != spiSendQueueTail)
		{
			// Update the head to the spot in the queue.
			spiSendQueueHead = (spiSendQueueHead + 1) & SPI_SEND_QUEUE_MASK;

			// Copy data packet into the send queue.
			memcpy(&spiSendQueue[spiSendQueueHead], spi, sizeof(PACKETData));

			// We were able to queue the data.
			status = TRUE;
		}

		// Release exclusive access to the send queue.
		osMutexRelease(spiQueueMutex);
	}

	// Notify the SPI thread to process the packet.
	if (status) osSignalSet(spiThreadId, 0x01);

	return status;
}

// Get the next packet from the send queue.
// Returns TRUE if the packet get succeeded.
BOOL spi_get_send_queue(PACKETData *packet)
{
	BOOL status = FALSE;

	// Wait for exclusive access to the send queue.
	osMutexWait(spiQueueMutex, osWaitForever);

	// Is there a packet on the queue?
	if (spiSendQueueTail != spiSendQueueHead)
	{
		// Increment the receive queue tail.
		spiSendQueueTail = (spiSendQueueTail + 1) & SPI_SEND_QUEUE_MASK;

		// Copy data packet from the receive queue.
		memcpy(packet, &spiSendQueue[spiSendQueueTail], sizeof(PACKETData));

		// We were able to get a packet from the queue.
		status = TRUE;
	}

	// Release exclusive access to the send queue.
	osMutexRelease(spiQueueMutex);

	return status;
}

// SPI processing thread.
void spi_thread(void const *argument)
{
	// Configure GPIOTE channel 0 to generate event when master select goes from high to low.
  NRF_GPIOTE->CONFIG[0] = (GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos) |
                          (SPI0_NVT1_MS_PIN << GPIOTE_CONFIG_PSEL_Pos) |
                          (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos);

	// Enable interrupt for NRF_GPIOTE->EVENTS_IN[0] event
  NRF_GPIOTE->INTENSET  = GPIOTE_INTENSET_IN0_Set << GPIOTE_INTENSET_IN0_Pos;

	// Set the interrupt priority.
	sd_nvic_SetPriority(GPIOTE_IRQn, APP_IRQ_PRIORITY_LOW);

	// Enable GPIOTE interrupt in Nested Vector Interrupt Controller.
	sd_nvic_EnableIRQ(GPIOTE_IRQn);

	// We loop forever processing SPI events.
	for (;;)
	{
		// Signal ourselves if there is something to do.
		if (spi_send_queue_ready() || spi_nvt1_ready()) osSignalSet(spiThreadId, 0x01);
	
		// Wait for a signal indicating SPI activity.
		osEvent evt = osSignalWait(0, osWaitForever);

		// Make sure a signal event was returned.
		if (evt.status != osEventSignal) continue;

		// Determine which event was signaled.
		if ((evt.value.signals & 0x01) && (spi_send_queue_ready() || spi_nvt1_ready()))
		{
			static PACKETData recv_data;
			static PACKETData send_data;

			// Pull the next packet to send from the SPI queue.  We send a null packet 
			// if there are no pending packets on the send queue.
			if (!spi_get_send_queue(&send_data)) memset(&send_data, 0, sizeof(PACKETData));

			// Receive a packet from the SPI slave.
			if (spi_nvt1_write_read(&send_data, &recv_data))
			{
				// Route this packet either to the BTLE send queue or to the
				// vuart serial stream receive queue.  Ignore packets that
				// are coming up from the SPI bus meant for other MCUs on the
				// SPI bus to prevent packet loops.
				if (packet_get_dest(&recv_data) == PACKET_LOC_BTLE)
				{
					// Direct this packet to BTLE send queue.  As current implemented,
					// overflow packets will discard older packets which may not be
					// the best strategy for handling buffer overflows.
					btle_put_send_queue(&recv_data);
				}
				else if (packet_get_dest(&recv_data) == PACKET_LOC_MCU0)
				{
					// XXX Check the packet type.

					// Direct this packet to the local receive queue.
					vuart_put_recv_queue(&recv_data);
				}
			}
		}
	}
}
