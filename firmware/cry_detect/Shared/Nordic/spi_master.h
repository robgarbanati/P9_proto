#ifndef SPI_MASTER_H
#define SPI_MASTER_H

#include "global.h"
#include "packet.h."
#include "gpio.h"

//
// Global Defines and Declarations
//

// SPI thread ID.
extern osThreadId spiThreadId;

// SPI master bus mutex.
extern osMutexId spiMasterMutex;

// Software controlled SPI Master driver.
//
// Supported features:
// - Operate two SPI masters independently or in parallel.
// - Transmit and Receive given size of data through SPI.
// - configure each SPI module separately

// SPI master operating frequency
typedef enum
{
	Freq_125Kbps = 0,		// Drive SClk with frequency 125Kbps
	Freq_250Kbps,				// Drive SClk with frequency 250Kbps
	Freq_500Kbps,				// Drive SClk with frequency 500Kbps
	Freq_1Mbps,					// Drive SClk with frequency 1Mbps
	Freq_2Mbps,					// Drive SClk with frequency 2Mbps
	Freq_4Mbps,					// Drive SClk with frequency 4Mbps
	Freq_8Mbps					// Drive SClk with frequency 8Mbps
} SPIFrequency;

// SPI master module number
typedef enum
{
	SPI0 = 0,							// SPI module 0
	SPI1									// SPI module 1
} SPIModuleNumber;

// SPI mode
typedef enum
{
	//------------------------Clock polarity 0, Clock starts with level 0-------------------------------------------
	SPI_MODE0 = 0,				// Sample data at rising edge of clock and shift serial data at falling edge
	SPI_MODE1,						// Sample data at falling edge of clock and shift serial data at rising edge
	//------------------------Clock polarity 1, Clock starts with level 1-------------------------------------------
	SPI_MODE2,						// Sample data at falling edge of clock and shift serial data at rising edge
	SPI_MODE3							// Sample data at rising edge of clock and shift serial data at falling edge
} SPIMode;

// SPI functions.
void spi_init(void);
void spi_configure(SPIModuleNumber module, uint8_t spi_mode, uint8_t lsb_first, uint8_t spi_freq);

// Low level functions.
uint8_t spi_write_read_slave_8(SPIModuleNumber module, uint8_t value);
uint16_t spi_write_read_slave_16(SPIModuleNumber module, uint16_t out_value);
uint32_t spi_write_read_slave_24(SPIModuleNumber module, uint32_t out_value);
uint32_t spi_write_read_slave_32(SPIModuleNumber module, uint32_t out_value);

// SPI queue functions.
BOOL spi_send_queue_ready(void);
BOOL spi_put_send_queue(const PACKETData *packet);
BOOL spi_get_send_queue(PACKETData *packet);

// The SPI thread.
void spi_thread(void const *argument);

#endif // SPI_MASTER_H
