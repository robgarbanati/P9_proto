#ifndef __SPI_H
#define __SPI_H

#include "Global.h"
#include "Packet.h"


//
// Global Defines and Declarations
//

#define SPI_BUSUP_GPIO					GPIOB
#define SPI_BUSUP_MS_PIN				DRVGPIO_PIN_0
#define SPI_BUSUP_SS_PIN				DRVGPIO_PIN_1
#define SPI_BUSUP_SCK_PIN				DRVGPIO_PIN_2
#define SPI_BUSUP_DO_PIN				DRVGPIO_PIN_3
#define SPI_BUSUP_DI_PIN				DRVGPIO_PIN_4

#ifdef __NVT1_APP__
#define SPI_BUSDN_GPIO					GPIOA
#define SPI_BUSDN_MS_PIN				DRVGPIO_PIN_5
#define SPI_BUSDN_SS_PIN				DRVGPIO_PIN_0
#define SPI_BUSDN_SCK_PIN				DRVGPIO_PIN_2
#define SPI_BUSDN_DI_PIN				DRVGPIO_PIN_3
#define SPI_BUSDN_DO_PIN				DRVGPIO_PIN_4

#define SPI_SFLASH_GPIO					GPIOA
#define SPI_SFLASH_SS_PIN				DRVGPIO_PIN_1
#define SPI_SFLASH_SCK_PIN			DRVGPIO_PIN_2
#define SPI_SFLASH_DI_PIN				DRVGPIO_PIN_3
#define SPI_SFLASH_DO_PIN				DRVGPIO_PIN_4
#endif

// SPI thread ID.
extern osThreadId spiThreadId;

// SPI master bus mutex.
extern osMutexId spiMasterMutex;

//
// Global Functions
//

// Init operations.
void spiInit(void);
void spiReset(void);

// Packet queue operations.
BOOL spiPutSendQueue(const PACKETData *packet);

// Data transfer handlers for upstream and downstream SPI buses.
void spiSlaveSelectHandler(void);
#ifdef __NVT1_APP__
void spiMasterSelectHandler(void);
#endif

// Spi processing thread.
void spiThread(void const *argument);

#endif // __SPI_H


