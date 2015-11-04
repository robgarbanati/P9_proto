#ifndef __SPI_QUEUE_H
#define __SPI_QUEUE_H

#include "global.h"
#include "packet.h"

//
// Global Defines and Declarations
//

//
// Global Functions
//

BOOL spi_send_queue_ready(void);
BOOL spi_put_send_queue(const PACKETData *packet);
BOOL spi_get_send_queue(PACKETData *packet);

#endif // __SPI_QUEUE_H


