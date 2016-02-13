#include <string.h>
#include "global.h"
#include "spi_queue.h"

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
// Global Functions
//

// Returns true if there are spis to send.
BOOL spi_send_queue_ready(void)
{
	return spiSendQueueTail != spiSendQueueHead;
}

// Put a spi on the send queue.
BOOL spi_put_send_queue(const PACKETData *spi)
{
	BOOL status = FALSE;

	// Sanity check the length.
	if ((spi->length >= 2) && (spi->length <= PACKET_BUF_LENGTH))
	{
		// Increment the receive queue head.
		spiSendQueueHead = (spiSendQueueHead + 1) & SPI_SEND_QUEUE_MASK;

		// Prevent collision of the receive queue head by discarding old spis.
		if (spiSendQueueHead == spiSendQueueTail) spiSendQueueTail = (spiSendQueueTail + 1) & SPI_SEND_QUEUE_MASK;

		// Copy data spi into the send queue.
		memcpy(&spiSendQueue[spiSendQueueHead], spi, sizeof(PACKETData));

		// We were able to queue the data.
		status = TRUE;
	}

	return status;
}

// Get the next spi from the send queue.
BOOL spi_get_send_queue(PACKETData *spi)
{
	BOOL status = FALSE;

	// Look for a non-zero length spi in the queue.
	while (!status && (spiSendQueueTail != spiSendQueueHead))
	{
		// Increment the receive queue tail.
		spiSendQueueTail = (spiSendQueueTail + 1) & SPI_SEND_QUEUE_MASK;

		// Ignore spis with zero length.
		if (spiSendQueue[spiSendQueueTail].length > 0)
		{
			// Copy data spi from the receive queue.
			memcpy(spi, &spiSendQueue[spiSendQueueTail], sizeof(PACKETData));

			// We were able to get a spi from the queue.
			status = TRUE;
		}
	}

	return status;
}

