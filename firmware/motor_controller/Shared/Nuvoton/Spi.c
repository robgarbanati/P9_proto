#include <string.h>
#include "Global.h"
#include "Driver/DrvGPIO.h"
#include "Driver/DrvSPI.h"
#include "Driver/DrvSYS.h"
#include "Packet.h"
#include "Spi.h"
#include "Command.h"
#include "Vuart.h"

//
// Global Variables
//

// SPI thread ID.
osThreadId spiThreadId;

// SPI queue mutex.
osMutexId spiQueueMutex;
osMutexDef(spiQueueMutex);

// SPI master bus mutex.
osMutexId spiMasterMutex;
osMutexDef(spiMasterMutex);

//
// Local Defines
//

// SPI slave mode for communication with the upward SPI bus.
#define SPI_BUSUP_HANDLER					DRVSPI_SPI1_HANDLER
#define SPI_BUSUP_DIVIDER     		(_DRVSPI_DIVIDER(DrvCLK_GetHclk(), 8000000))
#define SPI_BUSUP_OPEN_FLAGS 			(SPI_CNTRL_SLAVE | \
																	 DRVSPI_ENDIAN_BIG | \
																	 DRVSPI_MSB_FIRST | \
																	 DRVSPI_TX_1DATA | \
																	 DRVSPI_TX_NEGEDGE | \
																	 DRVSPI_RX_POSEDGE | \
																	 _DRVSPI_DATA_BITS(8))

#ifdef __NVT1_APP__
// SPI master mode for communication with the downward SPI bus.
#define SPI_BUSDN_HANDLER					DRVSPI_SPI0_HANDLER
#define SPI_BUSDN_DEVICE  				eDRVSPI_SLAVE_NONE
#define SPI_BUSDN_DIVIDER     		(_DRVSPI_DIVIDER(DrvCLK_GetHclk(), 1000000))
#define SPI_BUSDN_OPEN_FLAGS  		(DRVSPI_ENDIAN_BIG | \
																	 DRVSPI_IDEL_CLK_LOW | \
																	 DRVSPI_MSB_FIRST | \
																	 DRVSPI_TX_1DATA | \
																	 DRVSPI_TX_NEGEDGE | \
																	 DRVSPI_RX_POSEDGE | \
																	 _DRVSPI_SLEEP(2) | \
																	 _DRVSPI_DATA_BITS(8))
#endif

//
// Send/Receive Queues and Flags
//

// Queue size must be a power of two.
#define SPI_SEND_QUEUE_SIZE 4
#define SPI_SEND_QUEUE_MASK (SPI_SEND_QUEUE_SIZE - 1)

// Upstream send packet queue state variables.
static UINT16 spiUpstreamSendQueueTail = 0;
static UINT16 spiUpstreamSendQueueHead = 0;
static PACKETData spiUpstreamSendQueue[SPI_SEND_QUEUE_SIZE];

#ifdef __NVT1_APP__
// Downstream send packet queue state variables.
static UINT16 spiDownstreamSendQueueTail = 0;
static UINT16 spiDownstreamSendQueueHead = 0;
static PACKETData spiDownstreamSendQueue[SPI_SEND_QUEUE_SIZE];

// Inline helper functions.
static __inline BOOL spiDownstreamSendQueueIsEmpty(void)
{
	return spiDownstreamSendQueueHead == spiDownstreamSendQueueTail;
}
#endif

// SPI slave state buffer variables.
static volatile BOOL spiSlaveXferBusy;
static UINT8 spiSlaveXferState;
static UINT8 spiSlaveXferRecvCount;
static UINT8 spiSlaveXferSendCount;
static PACKETData spiSlaveSendPacket;
static PACKETData spiSlaveRecvPacket;

// SPI slave states for receiving data.
#define SPI_SLAVE_STATE_LEN				0
#define SPI_SLAVE_STATE_SUM				1
#define SPI_SLAVE_STATE_DATA			2
#define SPI_SLAVE_STATE_DONE			3

//
// Local Functions
//

#ifdef __NVT1_APP__
static UINT8 spiWriteReadBusDownByte(UINT8 value)
{
	// Write the outgoing data into the SPI transmit register.
	DrvSPI_SingleWriteData0(SPI_BUSDN_HANDLER, (UINT32) value);

	// Initiate the SPI transaction.
	DrvSPI_SetGo(SPI_BUSDN_HANDLER);

	// Wait for the transaction to complete.
	while (DrvSPI_GetBusy(SPI_BUSDN_HANDLER));

	// Return the incoming data from the SPI receive register.
	return (UINT8) DrvSPI_SingleReadData0(SPI_BUSDN_HANDLER);
}
#endif

// Add the next packet packet on the upstream send queue.
// Returns TRUE if the packet was added.
static BOOL spiPutUpstreamSendQueue(const PACKETData *packet)
{
	BOOL status = FALSE;

	// Sanity check the length.
	if ((packet->length >= 1) || (packet->length <= PACKET_BUF_LENGTH))
	{
		// Wait for exclusive access to the send queue.
		osMutexWait(spiQueueMutex, osWaitForever);

		// If there is no room in the queue the caller will have to try again.
		if (((spiUpstreamSendQueueHead + 1) & SPI_SEND_QUEUE_MASK) != spiUpstreamSendQueueTail)
		{
			// Update the head to the spot in the queue.
			spiUpstreamSendQueueHead = (spiUpstreamSendQueueHead + 1) & SPI_SEND_QUEUE_MASK;

			// Copy data packet into the send queue.
			memcpy(&spiUpstreamSendQueue[spiUpstreamSendQueueHead], packet, sizeof(PACKETData));

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

// Get the next packet packet on the upstream send queue.
// Returns TRUE if a packet was received and the length set.
static BOOL spiGetUpstreamSendQueue(PACKETData *packet)
{
	BOOL status = FALSE;

	// Wait for exclusive access to the send queue.
	osMutexWait(spiQueueMutex, osWaitForever);

	// Look for a non-zero length packet in the queue.
	while (!status && (spiUpstreamSendQueueTail != spiUpstreamSendQueueHead))
	{
		// Increment the receive queue tail.
		spiUpstreamSendQueueTail = (spiUpstreamSendQueueTail + 1) & SPI_SEND_QUEUE_MASK;

		// Ignore packets with zero length.
		if (spiUpstreamSendQueue[spiUpstreamSendQueueTail].length > 0)
		{
			// Copy data packet from the receive queue.
			memcpy(packet, &spiUpstreamSendQueue[spiUpstreamSendQueueTail], sizeof(PACKETData));

			// We were able to get a packet from the queue.
			status = TRUE;
		}
	}

	// Release exclusive access to the send queue.
	osMutexRelease(spiQueueMutex);

	// Signal that there is now space in the send queue.
	if (status) osSignalSet(mainThreadId, 0x01);

	return status;
}

#ifdef __NVT1_APP__
// Add the next packet packet on the downstream send queue.
// Returns TRUE if the packet was added.
static BOOL spiPutDownstreamSendQueue(const PACKETData *packet)
{
	BOOL status = FALSE;

	// Sanity check the length.
	if ((packet->length >= 1) && (packet->length <= PACKET_BUF_LENGTH))
	{
		// Wait for exclusive access to the send queue.
		osMutexWait(spiQueueMutex, osWaitForever);

		// If there is no room in the queue the caller will have to try again.
		if (((spiDownstreamSendQueueHead + 1) & SPI_SEND_QUEUE_MASK) != spiDownstreamSendQueueTail)
		{
			// Update the head to the spot in the queue.
			spiDownstreamSendQueueHead = (spiDownstreamSendQueueHead + 1) & SPI_SEND_QUEUE_MASK;

			// Copy data packet into the send queue.
			memcpy(&spiDownstreamSendQueue[spiDownstreamSendQueueHead], packet, sizeof(PACKETData));

			// We were able to queue the data.
			status = TRUE;
		}

		// Release exclusive access to the send queue.
		osMutexRelease(spiQueueMutex);
	}

	// Notify the SPI processing of the signal change.
	if (status) osSignalSet(spiThreadId, 0x02);

	return status;
}

// Get the next packet packet on the downstream send queue.
// Returns TRUE if a packet was received and the length set.
static BOOL spiGetDownstreamSendQueue(PACKETData *packet)
{
	BOOL status = FALSE;

	// Wait for exclusive access to the send queue.
	osMutexWait(spiQueueMutex, osWaitForever);

	// Look for a non-zero length packet in the queue.
	while (!status && (spiDownstreamSendQueueTail != spiDownstreamSendQueueHead))
	{
		// Increment the receive queue tail.
		spiDownstreamSendQueueTail = (spiDownstreamSendQueueTail + 1) & SPI_SEND_QUEUE_MASK;

		// Ignore packets with zero length.
		if (spiDownstreamSendQueue[spiDownstreamSendQueueTail].length > 0)
		{
			// Copy data packet from the receive queue.
			memcpy(packet, &spiDownstreamSendQueue[spiDownstreamSendQueueTail], sizeof(PACKETData));

			// We were able to get a packet from the queue.
			status = TRUE;
		}
	}

	// Release exclusive access to the send queue.
	osMutexRelease(spiQueueMutex);

	// Signal that there is now space in the send queue.
	if (status) osSignalSet(mainThreadId, 0x01);

	return status;
}
#endif

//
// Global Functions
//

void spiInit(void)
{
	// Create the SPI mutex.
	spiQueueMutex = osMutexCreate(osMutex(spiQueueMutex));
	spiMasterMutex = osMutexCreate(osMutex(spiMasterMutex));

	// Clear the slave receive and send lengths.
	spiSlaveRecvPacket.length = 0;
	spiSlaveSendPacket.length = 0;
	
	// Reset buffer state for a new packet.
	spiSlaveXferBusy = FALSE;
	spiSlaveXferState = SPI_SLAVE_STATE_LEN;
	spiSlaveXferRecvCount = 0;
	spiSlaveXferSendCount = 0;
	spiSlaveRecvPacket.length = 0;
	spiSlaveSendPacket.length = 0;

	// Enable interupt on upstream bus SS rising and falling.
	DrvGPIO_SetRisingInt(&SPI_BUSUP_GPIO, SPI_BUSUP_SS_PIN, TRUE);
	DrvGPIO_SetFallingInt(&SPI_BUSUP_GPIO, SPI_BUSUP_SS_PIN, TRUE);

#ifdef __NVT1_APP__
	// Enable interupt on downstream master select falling.
	DrvGPIO_SetFallingInt(&SPI_BUSDN_GPIO, SPI_BUSDN_MS_PIN, TRUE);
#endif
}

// Add the packet to the upstream or downstream send queue.
// Returns TRUE if the packet was added.
BOOL spiPutSendQueue(const PACKETData *packet)
{
	BOOL status = FALSE;

	// Determine which queue to add the packet to.
	if (packetGetDest(packet) < PACKET_LOC_THIS)
	{
		// Send the packet upstream.
		status = spiPutUpstreamSendQueue(packet);
	}
#ifdef __NVT1_APP__
	else if (packetGetDest(packet) > PACKET_LOC_THIS)
	{
		// Send the packet downstream.
		status = spiPutDownstreamSendQueue(packet);
	}
#endif
	else
	{
		// Silently drop all other packets.
		status = TRUE;
	}

	return status;
}

// SPI slave IRQ handler.  This interrupt handler is called after each
// byte is transferred from the master and once after the slave select
// is released.  Data transfers from the master are extremely time
// senstive and we cannot delay for more than four to five microseconds
// otherwise the packet will be corrupted.
void SPI1_IRQHandler(void)
{
	// Does the received number of bits indicate a transfer took place?
	// This interrupt handler is usually after the slave select is released 
	// with the trigger flag false.  Unfortunately, this doesn't happen all 
	// the time so it's unreliable to detect end of sending.  However, we 
	// can use the trigger flag set to be sure to reliably detect when an 
	// actual data byte was received by the SPI hardware.
	if (DrvSPI_SPI1_GetLevelTriggerFlag())
	{
		if (spiSlaveXferState == SPI_SLAVE_STATE_LEN)
		{
			// Get the length of the packet we are receiving.
			spiSlaveRecvPacket.length = DrvSPI_SingleReadData0(SPI_BUSUP_HANDLER);

			// Send the length of the packet being sent.
			DrvSPI_SingleWriteData0(SPI_BUSUP_HANDLER, spiSlaveSendPacket.length);

			// Sanity check the receive length to make sure we never overflow.
			if (spiSlaveRecvPacket.length > PACKET_BUF_LENGTH) spiSlaveRecvPacket.length = PACKET_BUF_LENGTH;
			
			// Move to the next state.
			spiSlaveXferState = SPI_SLAVE_STATE_SUM;
		}
		else if (spiSlaveXferState == SPI_SLAVE_STATE_SUM)
		{
			// Get the checksum of the packet we are receiving.
			spiSlaveRecvPacket.checksum = DrvSPI_SingleReadData0(SPI_BUSUP_HANDLER);

			// Send the checksum of the packet being sent.
			DrvSPI_SingleWriteData0(SPI_BUSUP_HANDLER, spiSlaveSendPacket.checksum);

			// Move to the next state.
			spiSlaveXferState = SPI_SLAVE_STATE_DATA;
		}
		else if (spiSlaveXferState == SPI_SLAVE_STATE_DATA)
		{
			// Get the next data byte just transferred in.
			if (spiSlaveXferRecvCount < spiSlaveRecvPacket.length)
				spiSlaveRecvPacket.buffer[spiSlaveXferRecvCount++] = DrvSPI_SingleReadData0(SPI_BUSUP_HANDLER);

			// Put the next data byte to be transferred out.
			if (spiSlaveXferSendCount < spiSlaveSendPacket.length)
				DrvSPI_SingleWriteData0(SPI_BUSUP_HANDLER, spiSlaveSendPacket.buffer[spiSlaveXferSendCount++]);
			else
				DrvSPI_SingleWriteData0(SPI_BUSUP_HANDLER, 0);

			// Are we done?
			if ((spiSlaveXferRecvCount >= spiSlaveRecvPacket.length) && (spiSlaveXferSendCount >= spiSlaveSendPacket.length))
			{
				spiSlaveXferState = SPI_SLAVE_STATE_DONE;
			}
		}
		else 
		{
			// Send zeros if the master is requesting additional data.
			DrvSPI_SingleWriteData0(SPI_BUSUP_HANDLER, 0);
		}
	}

	// Initiate the next SPI transaction.
	DrvSPI_SetGo(SPI_BUSUP_HANDLER);

	// Clear the SPI interrupt.
	DrvSPI_ClearIntFlag(SPI_BUSUP_HANDLER);
}


// Handles sending and receiving upstream packets.
void spiSlaveSelectHandler(void)
{
	// Use the slave select state to determine if a packet is starting or ending.
	if (!DrvGPIO_GetInputPinValue(&SPI_BUSUP_GPIO, SPI_BUSUP_SS_PIN))
	{
		// Mark the SPI interface as busy.
		spiSlaveXferBusy = TRUE;

		// Is the master select already asserted?  If the slave is initiating the transfer 
		// the master select would already be set and we don't need to do it again.
		if (DrvGPIO_GetInputPinValue(&SPI_BUSUP_GPIO, SPI_BUSUP_MS_PIN))
		{
			// No.  Initialize SPI transfers as a slave.
			DrvSPI_Open(SPI_BUSUP_HANDLER, SPI_BUSUP_OPEN_FLAGS, SPI_BUSUP_DIVIDER);

			// Slave select active low.
			DrvSPI_SlaveSelect(SPI_BUSUP_HANDLER, FALSE, FALSE);
	
			// Level trigger for slave mode.
			DrvSPI_SPI1_LevelTriggerInSlave(TRUE);

			// The initial data byte from a slave transaction is always zero.
			DrvSPI_SingleWriteData0(SPI_BUSUP_HANDLER, 0);

			// Initiate the SPI transaction.
			DrvSPI_SetGo(SPI_BUSUP_HANDLER);

			// Assert the master select line.
			DrvGPIO_ClearOutputBit(&SPI_BUSUP_GPIO, SPI_BUSUP_MS_PIN);

			// Enable SPI interrupts.
			DrvSPI_EnableInt(SPI_BUSUP_HANDLER);
		}
	}
	else
	{
		// Packet ending so release the master select line.
		DrvGPIO_SetOutputBit(&SPI_BUSUP_GPIO, SPI_BUSUP_MS_PIN);

		// Mark the SPI interface as no longer busy.
		spiSlaveXferBusy = FALSE;

		// Close the SPI driver for the upstream bus.
		DrvSPI_Close(SPI_BUSUP_HANDLER);

		// Validate length and checksum of the packet just received.
		// Silently ignore packets with invalid checksum values.
		if ((spiSlaveRecvPacket.length > 0) && packetValidateChecksum(&spiSlaveRecvPacket))
		{
			// The buffers have data that need to be processed so hold off 
			// additional processing by disabling the slave select interrupt.
			// These interrupts will be enabled below when the buffers are
			// processed and can receive additional data.
			DrvGPIO_SetRisingInt(&SPI_BUSUP_GPIO, SPI_BUSUP_SS_PIN, FALSE);
			DrvGPIO_SetFallingInt(&SPI_BUSUP_GPIO, SPI_BUSUP_SS_PIN, FALSE);
		}
		else
		{
			// Reset buffer state for a new packet.
			spiSlaveXferState = SPI_SLAVE_STATE_LEN;
			spiSlaveXferRecvCount = 0;
			spiSlaveXferSendCount = 0;
			spiSlaveRecvPacket.length = 0;
			spiSlaveSendPacket.length = 0;
		}

		// Notify the SPI thread that packet processing
		// to the SPI master should occur.
		osSignalSet(spiThreadId, 0x01);
	}
}

// Handles sending and receiving downstream packets.
void spiMasterSelectHandler(void)
{
	// Notify the SPI processing of the signal change.
	osSignalSet(spiThreadId, 0x02);
}

void spiProcessUpstreamPacket(void)
{
	// No processing while the upstream interface is busy.
	if (spiSlaveXferBusy) return;

	// Discard packets with invalid sizes.
	if ((spiSlaveRecvPacket.length >= 1) && (spiSlaveRecvPacket.length <= PACKET_BUF_LENGTH))
	{
		// Route the packet to the appropriate queue. We won't
		// attempt to route packets from upstream back upstream.
		if (packetGetDest(&spiSlaveRecvPacket) == PACKET_LOC_THIS)
		{
			// We currently support serial stream (VUART) and command packets.
			if (packetGetType(&spiSlaveRecvPacket) == PACKET_TYPE_SERIAL)
			{
				vuartPutRecvQueue(&spiSlaveRecvPacket);
			}
			else if (packetGetType(&spiSlaveRecvPacket) == PACKET_TYPE_SERIAL)
			{
				commandPutRecvQueue(&spiSlaveRecvPacket);
			}
		}
#ifdef __NVT1_APP__
		else if (packetGetDest(&spiSlaveRecvPacket) > PACKET_LOC_THIS)
		{
			// Put into the downstream send queue.
			spiPutDownstreamSendQueue(&spiSlaveRecvPacket);
		}
#endif
	}

	// Reset buffer state for a new packet.
	spiSlaveXferState = SPI_SLAVE_STATE_LEN;
	spiSlaveXferRecvCount = 0;
	spiSlaveXferSendCount = 0;
	spiSlaveRecvPacket.length = 0;

	// Should we put in the next packet to be processed?
	if (spiSlaveSendPacket.length == 0)
	{
		// Pull the next packet to send from the send queue.
		if (!spiGetUpstreamSendQueue(&spiSlaveSendPacket)) spiSlaveSendPacket.length = 0;
	}

	// Initiate a transfer if a packet is to be sent or slave select.
	// XXX There is a possiblity of a race condition where we don't
	// XXX initiate the transfer and don't see the select interrupt.
	// XXX We'll need to handle this at some point.
	if ((spiSlaveSendPacket.length != 0) || !DrvGPIO_GetInputPinValue(&SPI_BUSUP_GPIO, SPI_BUSUP_SS_PIN))
	{
		// Set the slave as busy.
		spiSlaveXferBusy = TRUE;

		// Initialize SPI transfers as a slave.
		DrvSPI_Open(SPI_BUSUP_HANDLER, SPI_BUSUP_OPEN_FLAGS, SPI_BUSUP_DIVIDER);

		// Slave select active low.
		DrvSPI_SlaveSelect(SPI_BUSUP_HANDLER, FALSE, FALSE);

		// Level trigger for slave mode.
		DrvSPI_SPI1_LevelTriggerInSlave(TRUE);

		// The initial data byte from a slave transaction is always zero.
		DrvSPI_SingleWriteData0(SPI_BUSUP_HANDLER, 0);

		// Initiate the SPI transaction.
		DrvSPI_SetGo(SPI_BUSUP_HANDLER);

		// Assert the master select line.
		DrvGPIO_ClearOutputBit(&SPI_BUSUP_GPIO, SPI_BUSUP_MS_PIN);

		// Enable SPI interrupts.
		DrvSPI_EnableInt(SPI_BUSUP_HANDLER);
	}

	// Interrupt on slave change of state.
	DrvGPIO_SetRisingInt(&SPI_BUSUP_GPIO, SPI_BUSUP_SS_PIN, TRUE);
	DrvGPIO_SetFallingInt(&SPI_BUSUP_GPIO, SPI_BUSUP_SS_PIN, TRUE);
}

#ifdef __NVT1_APP__
void spiProcessDownstreamPacket(void)
{
	UINT8 xferCount;
	UINT8 readIndex = 0;
	UINT8 writeIndex = 0;
	PACKETData sendPacket;
  PACKETData recvPacket;

	// Nothing to do if no downstream master select or no packet to send.
	if (DrvGPIO_GetInputPinValue(&SPI_BUSDN_GPIO, SPI_BUSDN_MS_PIN) && spiDownstreamSendQueueIsEmpty()) return;

	// Wait for exclusive access to the SPI master bus.
	osMutexWait(spiMasterMutex, osWaitForever);

	// Open the SPI driver.
	DrvSPI_Open(SPI_BUSDN_HANDLER, SPI_BUSDN_OPEN_FLAGS, SPI_BUSDN_DIVIDER);

	// Select the slave -- which is none in this case.  We manually assert chip select below.
	DrvSPI_SlaveSelect(SPI_BUSDN_HANDLER, FALSE, DRVSPI_IDEL_CLK_LOW);
	DrvSPI_SelectSlave(SPI_BUSDN_HANDLER, SPI_BUSDN_DEVICE);

	// We want an 8 bit transfer.
	DrvSPI_SetDataConfig(SPI_BUSDN_HANDLER, 1, 8);

	// Pull the next packet to send from the send queue.
	if (!spiGetDownstreamSendQueue(&sendPacket)) sendPacket.length = 0;

	// Assume we receive a zero length packet.
	recvPacket.length = 0;

	// Assert the slave select line to BUSDN.
	DrvGPIO_ClearOutputBit(&SPI_BUSDN_GPIO, SPI_BUSDN_SS_PIN);

	// Wait for assertion of master select to indicate slave is ready.
	while (DrvGPIO_GetInputPinValue(&SPI_BUSDN_GPIO, SPI_BUSDN_MS_PIN)) { __NOP(); __NOP(); }

	// Send length and discard the first byte received.
  spiWriteReadBusDownByte(sendPacket.length);	

	// Send checksum and receive length from the slave.
	recvPacket.length = spiWriteReadBusDownByte(sendPacket.checksum);
	
	// Send first byte, receive checksum from the slave.
	recvPacket.checksum = spiWriteReadBusDownByte(sendPacket.buffer[writeIndex++]);

	// Sanity check the receive buffer count. Discard oversize packets.
	if (recvPacket.length > PACKET_BUF_LENGTH) recvPacket.length = 0;

	// Determine the transfer count accounting for the fact we already sent one byte.
	xferCount = (recvPacket.length >= sendPacket.length) ? recvPacket.length : sendPacket.length - 1;
	
	// Transfer the rest of buffer.
	while (readIndex < xferCount)
	{
		// Send the indexed byte from the buffer.
		recvPacket.buffer[readIndex++] = spiWriteReadBusDownByte(sendPacket.buffer[writeIndex++]);

		// Make sure the write index doesn't overflow. This can happen because
		// we may have to send an extra byte to receive a full packet from the slave.
		if (writeIndex >= PACKET_BUF_LENGTH) writeIndex = PACKET_BUF_LENGTH - 1;
  }

	// Release the slave select line to BUSDN.
  DrvGPIO_SetOutputBit(&SPI_BUSDN_GPIO, SPI_BUSDN_SS_PIN);

	// Close the SPI driver.
	DrvSPI_Close(SPI_BUSDN_HANDLER);

	// Validate the length and checksum of the received packet.
	if ((recvPacket.length > 0) && packetValidateChecksum(&recvPacket))
	{
		// Route the packet to the appropriate queue. We won't
		// attempt to route packets from downstream back downstream.
		if (packetGetDest(&recvPacket) == PACKET_LOC_THIS)
		{
			// We currently support serial stream (VUART) and command packets.
			if (packetGetType(&spiSlaveRecvPacket) == PACKET_TYPE_SERIAL)
			{
				vuartPutRecvQueue(&spiSlaveRecvPacket);
			}
			else if (packetGetType(&spiSlaveRecvPacket) == PACKET_TYPE_SERIAL)
			{
				commandPutRecvQueue(&spiSlaveRecvPacket);
			}
		}
		else if (packetGetDest(&recvPacket) < PACKET_LOC_THIS)
		{
			// Put into the upstream send queue.
			spiPutUpstreamSendQueue(&recvPacket);
		}
	}

	// Release exclusive access to the SPI master bus.
	osMutexRelease(spiMasterMutex);
}
#endif

// SPI processing thread.
void spiThread(void const *argument)
{
	// We loop forever processing SPI events.
	for (;;)
	{
		// Wait for a signal indicating SPI activity.
		osEvent evt = osSignalWait(0, osWaitForever);

		// Determine which event was signaled.
		if (evt.status == osEventSignal)
		{
			// Upstream packet processing.
			if (evt.value.signals & 0x01) spiProcessUpstreamPacket();

#ifdef __NVT1_APP__
			// Downstream packet processing.
			if (evt.value.signals & 0x02) spiProcessDownstreamPacket();
#endif
		}
	}
}
