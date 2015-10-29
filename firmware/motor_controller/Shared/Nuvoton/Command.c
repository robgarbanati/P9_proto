#include <stdio.h>
#include <string.h>
#include "Global.h"
#include "Command.h"

//
// Global Variables
//

// Command thread ID.
osThreadId commandThreadId;

// Command queue mutex.
osMutexId commandQueueMutex;
osMutexDef(commandQueueMutex);

//
// Local Variables and Defines
//

// Queue size must be a power of two.
#define COMMAND_RECV_QUEUE_SIZE 4
#define COMMAND_RECV_QUEUE_MASK (COMMAND_RECV_QUEUE_SIZE - 1)

// Receive packet queue state variables.
static UINT8 commandRecvQueueTail = 0;
static UINT8 commandRecvQueueHead = 0;
static PACKETData commandRecvQueue[COMMAND_RECV_QUEUE_SIZE];

// Current command being processed.
static PACKETData commandBuffer;

//
// Local Functions
//

//
// Global Functions
//

void commandInit(void)
{
	// Create the command queue mutex.
	commandQueueMutex = osMutexCreate(osMutex(commandQueueMutex));
}

// Add the next packet packet on the receive queue.
// Returns TRUE if the packet was added.
BOOL commandPutRecvQueue(const PACKETData *packet)
{
	BOOL status = FALSE;

	// Sanity check the length.
	if ((packet->length >= 1) || (packet->length <= PACKET_BUF_LENGTH))
	{
		// Wait for exclusive access to the receive queue.
		osMutexWait(commandQueueMutex, osWaitForever);

		// Increment the receive queue head.
		commandRecvQueueHead = (commandRecvQueueHead + 1) & COMMAND_RECV_QUEUE_MASK;

		// Prevent collision of the receive queue head by discarding old packets.
		if (commandRecvQueueHead == commandRecvQueueTail) commandRecvQueueTail = (commandRecvQueueTail + 1) & COMMAND_RECV_QUEUE_MASK;
		
		// Copy data packet into the receive queue.
		memcpy(&commandRecvQueue[commandRecvQueueHead], packet, sizeof(PACKETData));

		// We were able to queue the data.
		status = TRUE;

		// Release exclusive access to the receive queue.
		osMutexRelease(commandQueueMutex);
	}

	// Signal that a receive queue contains at least one packet.
	if (status) osSignalSet(commandThreadId, 0x01);

	return status;
}

// Get the next packet packet on the receive queue.
// Returns TRUE if a packet was received and the length set.
BOOL commandGetRecvQueue(PACKETData *packet)
{
	BOOL status = FALSE;

	// Wait for exclusive access to the receive queue.
	osMutexWait(commandQueueMutex, osWaitForever);

	// Look for a non-zero length packet in the queue.
	while (!status && (commandRecvQueueTail != commandRecvQueueHead))
	{
		// Increment the receive queue tail.
		commandRecvQueueTail = (commandRecvQueueTail + 1) & COMMAND_RECV_QUEUE_MASK;

		// Ignore packets with zero length.
		if (commandRecvQueue[commandRecvQueueTail].length > 0)
		{
			// Copy data packet from the receive queue.
			memcpy(packet, &commandRecvQueue[commandRecvQueueTail], sizeof(PACKETData));

			// We were able to get a packet from the queue.
			status = TRUE;
		}
	}

	// Release exclusive access to the receive queue.
	osMutexRelease(commandQueueMutex);

	return status;
}

// Command processing thread.
void commandThread(void const *argument)
{
	// We loop forever processing command events.
	for (;;)
	{
		// Wait for a signal indicating sound activity.
		osEvent evt = osSignalWait(0, osWaitForever);

		// Skip if this isn't an event signal.
		if (evt.status != osEventSignal) continue;

		// Do we have a command to process?
		if (evt.value.signals & 0x01)
		{
			// Get the command packet from the receive queue.
			if (commandGetRecvQueue(&commandBuffer))
			{
				// XXX Parse and process the command.
			}
		}
	}
}

