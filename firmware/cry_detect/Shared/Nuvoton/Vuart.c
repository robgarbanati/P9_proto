#include <string.h>
#include "Global.h"
#include "Spi.h"
#include "Vuart.h"
#include "Packet.h"
#include "Debug.h"

// Virtual UART packets are variable length and consist of the 
// first byte being a bit packed packet destination, a bit
// packed packet source and a bit packed packet type and then
// up to 19 actual data bytes.  Accounting for the first 
// byte is why the index and length are reset to one rather
// than zero.

// VUART buffer mutex
osMutexId vuartMutex;
osMutexDef(vuartMutex);

// VUART queue mutex.
osMutexId vuartQueueMutex;
osMutexDef(vuartQueueMutex);

//
// Send/Receive Queues
//

// Queue size must be a power of two.
#define VUART_RECV_QUEUE_SIZE 8
#define VUART_RECV_QUEUE_MASK (VUART_RECV_QUEUE_SIZE - 1)

// Receive packet queue state variables.
static UINT8 vuartRecvQueueTail = 0;
static UINT8 vuartRecvQueueHead = 0;
static PACKETData vuartRecvQueue[VUART_RECV_QUEUE_SIZE];

//
// Send/Receive Buffers
//

// Receive buffer.  Must match SPI buffer length.
UINT8 vuartRecvIndex = 0;
UINT8 vuartRecvLength = 0;
PACKETData vuartRecvPacket;

// Send buffer. Must match SPI buffer length.
UINT8 vuartSendIndex = 1;
PACKETData vuartSendPacket;

// Timer for auto-flushing buffers.
osTimerId vuartFlushTimerId;

static void vuartTimerCallback(void const *arg)
{
	// Wait for exclusive access to the packet buffers.
	osMutexWait(vuartMutex, osWaitForever);

	// Set the packet destination and packet type.
	packetSetDest(&vuartSendPacket, PACKET_LOC_BTLE);
	packetSetSource(&vuartSendPacket, PACKET_LOC_THIS);
	packetSetType(&vuartSendPacket, PACKET_TYPE_SERIAL);
	vuartSendPacket.length = vuartSendIndex;
	packetSetChecksum(&vuartSendPacket);
	
	// Attempt to send the buffer.
	if (spiPutSendQueue(&vuartSendPacket))
	{
		// Reset the buffer index.
		vuartSendIndex = 1;
	}

	// Release exclusive access to the packet buffers.
	osMutexRelease(vuartMutex);
}

// Declare the virtual timer.
osTimerDef(vuartTimer, vuartTimerCallback);

//
// Global Functions
//

void vuartInit(void)
{
	// Create the VUART mutexes.
	vuartMutex = osMutexCreate(osMutex(vuartMutex));
	vuartQueueMutex = osMutexCreate(osMutex(vuartQueueMutex));

	// Initialize a periodic timer to flush the virtual UART.
	vuartFlushTimerId = osTimerCreate(osTimer(vuartTimer), osTimerOnce, NULL);
}


// Non-blocking call to get a character from the receive queue.  
// Returns TRUE if the character was read or FALSE if the queue is empty.
BOOL vuartGetChar(UINT8 *ch)
{
	BOOL status = FALSE;

	// Wait for exclusive access to the packet buffers.
	osMutexWait(vuartMutex, osWaitForever);

	// Do we need data from the packet queue?
	if (vuartRecvIndex >= vuartRecvLength)
	{
		// Reset the index and length.
		vuartRecvIndex = vuartRecvLength = 0;

		// Attempt to pull a packet from the receive queue.
		if (vuartGetRecvQueue(&vuartRecvPacket))
		{
			// Determine how this packet should be handled.
			if (vuartRecvPacket.length < 1)
			{
				// Ignore short packets.
			}
			else if (packetGetType(&vuartRecvPacket) != PACKET_TYPE_SERIAL)
			{
				// Ignore non-serial packets in the bootloader.
			}
			else
			{
				// This is serial data we can use.
				vuartRecvIndex = 1;
				vuartRecvLength = vuartRecvPacket.length;
			}
		}
	}

	// Do we have data to be read?
	if (vuartRecvIndex < vuartRecvLength)
	{
		// Pull the next character from the buffer.
		*ch = vuartRecvPacket.buffer[vuartRecvIndex++];

		// We received a character.
		status = TRUE;
	}

	// Release exclusive access to the packet buffers.
	osMutexRelease(vuartMutex);

	return status;
}

// Blocking call with timeout to get a character from the receive queue.  
// Returns TRUE if the character was read or FALSE if the timeout expired.
BOOL vuartGetCharTimeout(UINT8 *ch, UINT32 timeout)
{
	BOOL status = FALSE;
	UINT32 tick = osKernelSysTick();

	do
	{
		// Attempt to get a character.
		status = vuartGetChar(ch);

		// Break if we succeeded.
		if (status) break;

		// Wait a bit if we didn't succeed.
		osSignalWait(0x01, 10);
	} while ((osKernelSysTick() - tick) < osKernelSysTickMicroSec(timeout * 1000));

	return status;
}

// Non-blocking call to add a character to the send queue.  Returns 
// TRUE if the character was added or FALSE if the queue is full.  
// The caller should either retry sending or discard the character.
BOOL vuartPutChar(UINT8 ch)
{
	BOOL status = FALSE;

	// Wait for exclusive access to the packet buffers.
	osMutexWait(vuartMutex, osWaitForever);

	// Is there room to put data into the queue?
	if (vuartSendIndex < PACKET_BUF_LENGTH)
	{
		// Start the 20 millisecond flush timer.
		if (vuartSendIndex == 1) osTimerStart(vuartFlushTimerId, 20);

		// Place the character into the buffer.
		vuartSendPacket.buffer[vuartSendIndex++] = ch;

		// We added the character.
		status = TRUE;
	}

#if 0
	// XXX Currently there is no flow control for packets destined for the
	// XXX BTLE connection and the Nordic queues can be overflowed if we send 
	// XXX too much data here.  For now, we avoid the issue by only sending
	// XXX out packets every 20 milliseconds which is roughly how quickly
	// XXX packets can be sent over the BTLE connection.  A longer term 
	// XXX solution will need to be found for packet flow control.
	if (vuartSendIndex == PACKET_BUF_LENGTH)
	{
		// Set the packet destination and packet type.
		packetSetDest(&vuartSendPacket, PACKET_LOC_BTLE);
		packetSetSource(&vuartSendPacket, PACKET_LOC_THIS);
		packetSetType(&vuartSendPacket, PACKET_TYPE_SERIAL);
		vuartSendPacket.length = vuartSendIndex;
		packetSetChecksum(&vuartSendPacket);
		
		// Attempt to send the buffer.
		if (spiPutSendQueue(&vuartSendPacket))
		{
			// Reset the buffer index.
			vuartSendIndex = 1;

			// Stop the flush timer.
			osTimerStop(vuartFlushTimerId);
		}
	}
#endif

	// Release exclusive access to the packet buffers.
	osMutexRelease(vuartMutex);

	return status;
}


// Add the next packet packet on the receive queue.
// Returns TRUE if the packet was added.
BOOL vuartPutRecvQueue(const PACKETData *packet)
{
	BOOL status = FALSE;

	// Sanity check the length.
	if ((packet->length >= 1) || (packet->length <= PACKET_BUF_LENGTH))
	{
		// Wait for exclusive access to the receive queue.
		osMutexWait(vuartQueueMutex, osWaitForever);

		// Increment the receive queue head.
		vuartRecvQueueHead = (vuartRecvQueueHead + 1) & VUART_RECV_QUEUE_MASK;

		// Prevent collision of the receive queue head by discarding old packets.
		if (vuartRecvQueueHead == vuartRecvQueueTail) vuartRecvQueueTail = (vuartRecvQueueTail + 1) & VUART_RECV_QUEUE_MASK;
		
		// Copy data packet into the receive queue.
		memcpy(&vuartRecvQueue[vuartRecvQueueHead], packet, sizeof(PACKETData));

		// Release exclusive access to the receive queue.
		osMutexRelease(vuartQueueMutex);

		// We were able to queue the data.
		status = TRUE;
	}

	// Signal that a receive queue contains at least one packet.
	if (status) osSignalSet(mainThreadId, 0x01);

	return status;
}

// Get the next packet packet on the receive queue.
// Returns TRUE if a packet was received and the length set.
BOOL vuartGetRecvQueue(PACKETData *packet)
{
	BOOL status = FALSE;

	// Wait for exclusive access to the receive queue.
	osMutexWait(vuartQueueMutex, osWaitForever);

	// Look for a non-zero length packet in the queue.
	while (!status && (vuartRecvQueueTail != vuartRecvQueueHead))
	{
		// Increment the receive queue tail.
		vuartRecvQueueTail = (vuartRecvQueueTail + 1) & VUART_RECV_QUEUE_MASK;

		// Ignore packets with zero length.
		if (vuartRecvQueue[vuartRecvQueueTail].length > 0)
		{
			// Copy data packet from the receive queue.
			memcpy(packet, &vuartRecvQueue[vuartRecvQueueTail], sizeof(PACKETData));

			// We were able to get a packet from the queue.
			status = TRUE;
		}
	}

	// Release exclusive access to the receive queue.
	osMutexRelease(vuartQueueMutex);

	return status;
}

