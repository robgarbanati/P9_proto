#ifndef SPI_FLASH_H
#define SPI_FLASH_H

#include "global.h"
#include "spi_master.h"

//
// Global Defines and Declarations
//

// SPI Flash Status
#define SPIFLASH_SPR						0x80	// Status Register Protect
#define SPIFLASH_R							0x40	// Reserved Bit
#define SPIFLASH_TB							0x20	// Top / Bottom Block Protect
#define SPIFLASH_BP2						0x10	// Block Protect Bit 2
#define SPIFLASH_BP1						0x08	// Block Protect Bit 1
#define SPIFLASH_BP0						0x04	// Block Protect Bit 0
#define SPIFLASH_WEL						0x02	// Write Enable Latch
#define SPIFLASH_WIP						0x01	// Write In Progress
#define SPIFLASH_BP							(SPIFLASH_TB|SPIFLASH_BP2|SPIFLASH_BP1|SPIFLASH_BP0)

// SPI Flash Command
#define SPIFLASH_ZERO						0x00
#define SPIFLASH_DUMMY					0xFF
#define SPIFLASH_WRITE_ENABLE		0x06
#define SPIFLASH_WRITE_DISABLE	0x04
#define SPIFLASH_READ_STATUS		0x05
#define SPIFLASH_WRITE_STATUS		0x01
#define SPIFLASH_READ_DATA			0x03 // Using fast read to replace normal read
#define SPIFLASH_FAST_READ			0x0B
#define SPIFLASH_FAST_RD_DUAL		0x3B
#define SPIFLASH_PAGE_PROGRAM		0x02
#define SPIFLASH_64K_ERASE			0xD8
#define SPIFLASH_4K_ERASE				0x20
#define SPIFLASH_32K_ERASE			0x52
#define SPIFLASH_CHIP_ERASE			0xC7
#define SPIFLASH_POWER_DOWN			0xB9
#define SPIFLASH_RELEASE_PD_ID	0xAB
#define SPIFLASH_DEVICE_ID			0x90
#define SPIFLASH_JEDEC_ID				0x9F

#define SPIFLASH_FLAG_ERASE_64K	0x01
#define SPIFLASH_FLAG_ERASE_4K	0x02
#define SPIFLASH_FLAG_ERASE_32K	0x04
#define SPIFLASH_FLAG_DUAL_SPI	0x08
#define SPIFLASH_FLAG_WINBOND		0x10
#define SPIFLASH_FLAG_MXIC			0x20
#define SPIFLASH_FLAG_ATMEL			0x40

// Device ID for support flash type list
#define SPIFLASH_W25P		0x10				// W25P series
#define SPIFLASH_W25X		0x40				// W25X series
#define SPIFLASH_W25Q		0x50				// W25Q series

// W25P series
#define SPIFLASH_W25P10		0x10			// 1M-bits
#define SPIFLASH_W25P20		0x11			// 2M-bits
#define SPIFLASH_W25P40		0x12			// 4M-bits
#define SPIFLASH_W25P80		0x13			// 8M-bits
#define SPIFLASH_W25P16		0x14			// 16M-bits
#define SPIFLASH_W25P32		0x15			// 32M-bits

// W25X series
#define SPIFLASH_W25X10		0x40			//  1M-bits
#define SPIFLASH_W25X20		0x41			//  2M-bits
#define SPIFLASH_W25X40		0x42			//  4M-bits
#define SPIFLASH_W25X80		0x43			//  8M-bits
#define SPIFLASH_W25X16		0x44			// 16M-bits
#define SPIFLASH_W25X32		0x45			// 32M-bits
#define SPIFLASH_W25X64		0x46			// 64M-bits

// W25Q series
#define SPIFLASH_W25Q10		0x50			//  1M-bits
#define SPIFLASH_W25Q20		0x51			//  2M-bits
#define SPIFLASH_W25Q40		0x52			//  4M-bits
#define SPIFLASH_W25Q80		0x53			//  8M-bits
#define SPIFLASH_W25Q16		0x54			// 16M-bits
#define SPIFLASH_W25Q32		0x55			// 32M-bits
#define SPIFLASH_W25Q64		0x56			// 64M-bits
#define SPIFLASH_W25Q128	0x57			// 128M-bits

typedef struct
{
	SPIModuleNumber module;
	uint32_t				flash_size;				// SPI memory size
	uint8_t					flags;
} SPIFlash;

//
// Global Functions
//

// SPI flash functions.
void spi_flash_open(SPIModuleNumber module, uint8_t spi_mode, uint8_t lsb_first, uint8_t spi_freq, SPIFlash *handler);
void spi_flash_close(SPIFlash *handler);
uint8_t spi_flash_read_status(SPIFlash *handler);
void spi_flash_wait_not_writing(SPIFlash *handler);
void spi_flash_write_enable(SPIFlash *handler, bool enable);
void spi_flash_get_chip_info(SPIFlash *handler);
void spi_flash_erase_chip(SPIFlash *handler);
void spi_flash_erase(SPIFlash *handler, uint32_t cmd_addr, uint32_t add_incr, uint16_t erase_count);
void spi_flash_read(SPIFlash *handler, uint32_t byte_addr, uint8_t *data, uint32_t length);
void spi_flash_write(SPIFlash *handler, uint32_t byte_addr, const uint8_t *data, uint32_t length);

// Inline helper functions.
static inline void spi_flash_erase_4K(SPIFlash *handler, uint16_t index_of_4K, uint16_t erase_count)
{
	spi_flash_erase(handler, ((uint32_t) SPIFLASH_4K_ERASE << 24) | (index_of_4K << 12), (1 << 12), erase_count);
}
static inline void spi_flash_erase_64K(SPIFlash *handler, uint8_t index_of_64K, uint16_t erase_count)
{
	spi_flash_erase(handler, ((uint32_t) SPIFLASH_64K_ERASE << 24) | (index_of_64K << 16), (1 << 16), erase_count);
}

#endif // SPI_FLASH_H
