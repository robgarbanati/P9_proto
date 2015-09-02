#ifndef __SPIFS_H
#define __SPIFS_H

#include "Global.h"

//
// Global Defines and Declarations
//

#define SPI_SFLASH_GPIO					GPIOA
#define SPI_SFLASH_SS_PIN				DRVGPIO_PIN_1
#define SPI_SFLASH_SCK_PIN			DRVGPIO_PIN_2
#define SPI_SFLASH_DI_PIN				DRVGPIO_PIN_3
#define SPI_SFLASH_DO_PIN				DRVGPIO_PIN_4

#define SPIFS_FILENAME_LEN      16

// SPI file structure.
typedef struct _SPIFS_FILE
{
  BOOL write;
  INT16 fileId;
  INT16 blockId;
  INT16 blockSize;
	INT16 blockIndex;
  INT16 thisId;
	INT16 nextId;
	UINT32 position;
} SPIFS_FILE;

// SPI file structure.
typedef struct _SPIFS_DIR
{
  INT16 fileId;
	UINT32 fileSize;
	char fileName[SPIFS_FILENAME_LEN];
} SPIFS_DIR;

//
// Global Functions
//

BOOL spiFSMount(void);
void spiFSUnmount(void);
BOOL spiFSFormat(void);
BOOL spiFSUnlink(const char *filename);
BOOL spiFSOpen(const char *filename, BOOL write, SPIFS_FILE *file);
BOOL spiFSClose(SPIFS_FILE *file);
BOOL spiFSRead(SPIFS_FILE *file, UINT8 *buffer, INT16 *count);
BOOL spiFSWrite(SPIFS_FILE *file, const UINT8 *buffer, INT16 *count);
BOOL spiFSSeek(SPIFS_FILE *file, UINT32 position);

BOOL spiFSOpenDir(SPIFS_DIR *dir);
BOOL spiFSCloseDir(SPIFS_DIR *dir);
BOOL spiFSReadDir(SPIFS_DIR *dir);

#endif // __SPIFS_H


