#include <string.h>
#include "global.h"
#include "vuart.h"
#include "packet.h"
#include "xprintf.h"
#include "ble_spi.h"

//
// Send/Receive Queues
//

// Receive buffer.
static UINT8 vuartRecvIndex = 0;
static UINT8 vuartRecvLength = 0;
static PACKETData vuartRecvPacket;

// Send buffer.
static UINT8 vuartSendIndex = 1;
static PACKETData vuartSendPacket;

// Queue size must be a power of two.
#define VUART_SEND_QUEUE_SIZE 4
#define VUART_RECV_QUEUE_SIZE 4
#define VUART_SEND_QUEUE_MASK (VUART_SEND_QUEUE_SIZE - 1)
#define VUART_RECV_QUEUE_MASK (VUART_RECV_QUEUE_SIZE - 1)

// Send packet queue state variables.
static UINT8 vuartSendQueueTail = 0;
static UINT8 vuartSendQueueHead = 0;
static PACKETData vuartSendQueue[VUART_SEND_QUEUE_SIZE];

// Receive packet queue state variables.
static UINT8 vuartRecvQueueTail = 0;
static UINT8 vuartRecvQueueHead = 0;
static PACKETData vuartRecvQueue[VUART_RECV_QUEUE_SIZE];

// Thread IDs associated with sending/receiving thread.
static osThreadId vuartSendThreadId = NULL;
static osThreadId vuartRecvThreadId = NULL;

// VUART buffer mutex
static osMutexId vuartBufferMutex;
osMutexDef(vuartBufferMutex);

// VUART queue mutex.
static osMutexId vuartQueueMutex;
osMutexDef(vuartQueueMutex);

// Timer for auto-flushing buffers.
static void vuartTimerCallback(void const *arg);
static osTimerId vuartFlushTimerId;
osTimerDef(vuartTimer, vuartTimerCallback);

// Put a packet on the VUART send queue.
static BOOL vuart_put_send_queue(const PACKETData *packet)
{
	BOOL status = FALSE;

	// Sanity check the length.  Notice that we expect VUART
	// packets to have a size of at least 2 bytes.
	if ((packet->length >= 2) && (packet->length <= PACKET_BUF_LENGTH))
	{
		// Wait for exclusive access to the queue.
		osMutexWait(vuartQueueMutex, osWaitForever);

		// If there is no room in the queue the caller will have to try again.
		if (((vuartSendQueueHead + 1) & VUART_SEND_QUEUE_MASK) != vuartSendQueueTail)
		{
			// Increment the receive queue head.
			vuartSendQueueHead = (vuartSendQueueHead + 1) & VUART_SEND_QUEUE_MASK;

			// Copy data packet into the send queue.
			memcpy(&vuartSendQueue[vuartSendQueueHead], packet, sizeof(PACKETData));

			// We were able to queue the data.
			status = TRUE;
		}

		// Release exclusive access to the queue.
		osMutexRelease(vuartQueueMutex);
	}

	// Signal the ble thread that a packet is waiting to be sent.
	if (status) osSignalSet(bleThreadId, 0x01);

	return status;
}


// Get the next packet from the VUART receive queue.
static BOOL vuart_get_recv_queue(PACKETData *packet)
{
	BOOL status = FALSE;

	// Wait for exclusive access to the queue.
	osMutexWait(vuartQueueMutex, osWaitForever);

	// Is there a packet on the queue?
	if (vuartRecvQueueTail != vuartRecvQueueHead)
	{
		// Increment the receive queue tail.
		vuartRecvQueueTail = (vuartRecvQueueTail + 1) & VUART_RECV_QUEUE_MASK;

		// Copy data packet from the receive queue.
		memcpy(packet, &vuartRecvQueue[vuartRecvQueueTail], sizeof(PACKETData));

		// We were able to get a packet from the queue.
		status = TRUE;
	}

	// Release exclusive access to the queue.
	osMutexRelease(vuartQueueMutex);

	return status;
}


static void vuartTimerCallback(void const *arg)
{
	// Wait for exclusive access to the packet buffers.
	osMutexWait(vuartBufferMutex, osWaitForever);

	// Set the packet destination and packet type.
	packet_set_dest(&vuartSendPacket, PACKET_LOC_BTLE);
	packet_set_source(&vuartSendPacket, PACKET_LOC_MCU0);
	packet_set_type(&vuartSendPacket, PACKET_TYPE_SERIAL);
	vuartSendPacket.length = vuartSendIndex;
	packet_set_checksum(&vuartSendPacket);
	
	// Attempt to send the buffer.
	if (vuart_put_send_queue(&vuartSendPacket))
	{
		// Reset the buffer index.
		vuartSendIndex = 1;
	}

	// Release exclusive access to the packet buffers.
	osMutexRelease(vuartBufferMutex);
}


// Callback interface for formatted input.
static unsigned char vuart_xprintf_input(void)
{
	UINT8 ch;

	// Get a character.
	vuart_get_char(&ch);

	return ch;
}


// Callback interface for formatted output.
static void vuart_xprintf_output(char ch)
{
	// Put the character.
	vuart_put_char((UINT8) ch);
}


//
// Global Functions
//

void vuart_init(void)
{
	// Create the virtual UART mutexes.
	vuartBufferMutex = osMutexCreate(osMutex(vuartBufferMutex));
	vuartQueueMutex = osMutexCreate(osMutex(vuartQueueMutex));

	// Initialize a periodic timer to flush the virtual UART.
	vuartFlushTimerId = osTimerCreate(osTimer(vuartTimer), osTimerOnce, NULL);

	// Connected formatted input/output to the virtual UART.
	xdev_in(vuart_xprintf_input);
	xdev_out(vuart_xprintf_output);
}


// Get a character from the receive queue.  Returns TRUE if the 
// character was read or FALSE if the queue is empty and the
// timeout expired.
BOOL vuart_get_char_with_timeout(UINT8 *ch, UINT32 timeout)
{
	BOOL status = FALSE;

	// Attempt to get a character.
	while (!status && (timeout > 0))
	{
		// Set the receiving thread.
		vuartRecvThreadId = osThreadGetId();

		// Clear the thread receive signal event.
		osSignalClear(vuartRecvThreadId, 0x01);

		// Wait for exclusive access to the packet buffers.
		osMutexWait(vuartBufferMutex, osWaitForever);

		// Do we need data from the packet queue?
		if (vuartRecvIndex >= vuartRecvLength)
		{
			// Reset the index and length.
			vuartRecvIndex = vuartRecvLength = 0;

			// Attempt to pull a packet from the receive queue.
			if (vuart_get_recv_queue(&vuartRecvPacket))
			{
				// Determine how this packet should be handled.
				if (vuartRecvPacket.length <  1)
				{
					// Ignore short packets.
				}
				else if (packet_get_type(&vuartRecvPacket) != PACKET_TYPE_SERIAL)
				{
					// Ignore non-serial packets.
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
		osMutexRelease(vuartBufferMutex);

		// Should we wait for data?
		if (!status && (timeout > 0))
		{
			// Determine how long to wait for.
			UINT32 wait = (timeout > 100) ? 100 : timeout;
			
			// Wait up to 100 milliseconds for a matching signal.
			osSignalWait(0x01, wait); 

			// Update the timeout.
			if (timeout != osWaitForever) timeout -= wait;
		}
	}

	return status;
}


// Add a character to the send queue.  Returns TRUE if the character
// was added or FALSE if the queue is full and the timeout expired.
BOOL vuart_put_char_with_timeout(UINT8 ch, UINT32 timeout)
{
	BOOL status = FALSE;

	// Attempt to get a character.
	while (!status && (timeout > 0))
	{
		// Set the sending thread.
		vuartSendThreadId = osThreadGetId();

		// Clear the thread send signal event.
		osSignalClear(vuartSendThreadId, 0x02);

		// Wait for exclusive access to the packet buffers.
		osMutexWait(vuartBufferMutex, osWaitForever);

		// Is there room to put data into the queue?
		if (vuartSendIndex < PACKET_BUF_LENGTH)
		{
			// Start the 20 millisecond flush timer.
			if (vuartSendIndex == 1)
			{
				osTimerStart(vuartFlushTimerId, 20);
			}

			// Place the character into the buffer.
			vuartSendPacket.buffer[vuartSendIndex++] = ch;

			// We added the character.
			status = TRUE;
		}

		// Is this buffer ready to be sent?
		if (vuartSendIndex == PACKET_BUF_LENGTH)
		{
			// Set the packet destination and packet type.
			packet_set_dest(&vuartSendPacket, PACKET_LOC_BTLE);
			packet_set_source(&vuartSendPacket, PACKET_LOC_MCU0);
			packet_set_type(&vuartSendPacket, PACKET_TYPE_SERIAL);
			vuartSendPacket.length = vuartSendIndex;
			packet_set_checksum(&vuartSendPacket);

			// Attempt to send the buffer.
			if (vuart_put_send_queue(&vuartSendPacket))
			{
				// Reset the buffer index.
				vuartSendIndex = 1;

				// Stop the flush timer.
				osTimerStop(vuartFlushTimerId);
			}
		}

		// Release exclusive access to the packet buffers.
		osMutexRelease(vuartBufferMutex);

		// Should we wait for the queue to clear?
		if (!status && (timeout > 0))
		{
			// Determine how long to wait for.
			UINT32 wait = (timeout > 100) ? 100 : timeout;
			
			// Wait up to 100 milliseconds for a matching signal.
			osSignalWait(0x02, wait); 

			// Update the timeout.
			if (timeout != osWaitForever) timeout -= wait;
		}
	}

	return status;
}


// Get the next packet from the VUART send queue.
BOOL vuart_get_send_queue(PACKETData *packet)
{
	BOOL status = FALSE;

	// Wait for exclusive access to the queue.
	osMutexWait(vuartQueueMutex, osWaitForever);

	// Is there a packet on the queue?
	if (vuartSendQueueTail != vuartSendQueueHead)
	{
		// Increment the receive queue tail.
		vuartSendQueueTail = (vuartSendQueueTail + 1) & VUART_SEND_QUEUE_MASK;

		// Copy data packet from the receive queue.
		memcpy(packet, &vuartSendQueue[vuartSendQueueTail], sizeof(PACKETData));

		// We were able to get a packet from the queue.
		status = TRUE;
	}

	// Release exclusive access to the queue.
	osMutexRelease(vuartQueueMutex);

	// Signal the sending thread that there is space to send a packet.
	if (status && vuartSendThreadId) osSignalSet(vuartSendThreadId, 0x02);
	
	return status;
}


// Put a packet on the VUART receive queue.
BOOL vuart_put_recv_queue(const PACKETData *packet)
{
	BOOL status = FALSE;

	// Sanity check the length.  Notice that we expect VUART
	// packets to have a size of at least 2 bytes.
	if ((packet->length >= 2) && (packet->length <= PACKET_BUF_LENGTH))
	{
		// Wait for exclusive access to the queue.
		osMutexWait(vuartQueueMutex, osWaitForever);

		// Increment the receive queue head.
		vuartRecvQueueHead = (vuartRecvQueueHead + 1) & VUART_RECV_QUEUE_MASK;

		// Prevent collision of the receive queue head by discarding old packets.
		if (vuartRecvQueueHead == vuartRecvQueueTail) vuartRecvQueueTail = (vuartRecvQueueTail + 1) & VUART_RECV_QUEUE_MASK;

		// Copy data packet into the send queue.
		memcpy(&vuartRecvQueue[vuartRecvQueueHead], packet, sizeof(PACKETData));

		// Release exclusive access to the queue.
		osMutexRelease(vuartQueueMutex);

		// We were able to queue the data.
		status = TRUE;
	}

	// Signal the waiting thread that a packet is waiting to be received.
	if (status && vuartRecvThreadId) osSignalSet(vuartRecvThreadId, 0x01);

	return status;
}

