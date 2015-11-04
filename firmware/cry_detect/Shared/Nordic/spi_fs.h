#ifndef __SPIFS_H
#define __SPIFS_H

#include "global.h"

//
// Global Defines and Declarations
//

#define SPIFS_FILENAME_LEN      16

// SPI file structure.
typedef struct _SPIFSFile
{
  bool write;
  int16_t fileId;
  int16_t blockId;
  int16_t blockSize;
	int16_t blockIndex;
  int16_t thisId;
	int16_t nextId;
	uint32_t position;
} SPIFSFile;

// SPI file structure.
typedef struct _SPIFSDir
{
  int16_t fileId;
	uint32_t fileSize;
	char fileName[SPIFS_FILENAME_LEN];
} SPIFSDir;

//
// Global Functions
//

bool spifs_mount(void);
void spifs_unmount(void);
bool spifs_format(void);
bool spifs_unlink(const char *filename);

bool spifs_open(const char *filename, bool write, SPIFSFile *file);
bool spifs_close(SPIFSFile *file);
uint32_t spifs_read(SPIFSFile *file, uint8_t *buffer, uint32_t count);
uint32_t spifs_write(SPIFSFile *file, const uint8_t *buffer, uint32_t count);
bool spifs_seek(SPIFSFile *file, uint32_t position);

bool spifs_open_dir(SPIFSDir *dir);
bool spifs_close_dir(SPIFSDir *dir);
bool spifs_read_dir(SPIFSDir *dir);

#endif // __SPIFS_H


