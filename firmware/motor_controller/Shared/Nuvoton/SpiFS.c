#include <string.h>
#include <ctype.h>
#include "Global.h"
#include "Driver/DrvSPI.h"
#include "Storage/SPIFlash.h"
#include "XPrintf.h"
#include "Spi.h"
#include "SpiFS.h"

#define SPI_FLASH_HANDLER				DRVSPI_SPI0_HANDLER
#define SPI_FLASH_DEVICE  			eDRVSPI_SLAVE_1
#define SPI_FLASH_DIVIDER     	(_DRVSPI_DIVIDER(DrvCLK_GetHclk(), 12000000))

#define SPIFS_SECTOR_FIRST			1			// First sector (others are reserved).
#define SPIFS_SECTOR_COUNT			512		// Count of sectors on spi flash.
#define SPIFS_SECTOR_SIZE				4096	// Size of sector in bytes.

#define SPIFS_HEADER_SIZE				8			// Size of the header portion of a file block.
#define SPIFS_BLOCK_SIZE				4088	// Size of the data portion of the file block.

#define SPIFS_FILEID_OFFSET			0			// Offset of file id in a file block.
#define SPIFS_BLOCKID_OFFSET		2			// Offset of block id in a file block.
#define SPIFS_BLOCKSIZE_OFFSET	4			// Offset of block size in a file block.
#define SPIFS_NEXTID_OFFSET			6			// Offset of next block id in a file block.
#define SPIFS_FILENAME_OFFSET		8			// Offset of filename a file block.

// Returns TRUE if sectorid is valid.
#define SPIFS_SECTORID_VALID(sid) \
		((sid >= SPIFS_SECTOR_FIRST) && (sid < SPIFS_SECTOR_COUNT))

// Returns TRUE if fileid is valid.
#define SPIFS_FILEID_VALID(fid) \
		((fid >= SPIFS_SECTOR_FIRST) && (fid < SPIFS_SECTOR_COUNT))

// Returns TRUE if block size is valid.
#define SPIFS_BLOCKSIZE_VALID(bs) \
		((bs > 0) && (bs <= SPIFS_BLOCK_SIZE))

// Converts a sector id to a byte offset in flash.
#define SPIFS_SECTORID_TO_BYTEOFFSET(id) \
		(((UINT32) id) << 12)

// SPI block header structure.
typedef __packed struct _SPIFS_FILE_BLOCK
{
  INT16 fileId;
  INT16 blockId;
  INT16 blockSize;
  INT16 nextId;
} SPIFS_FILE_BLOCK;

// Cleans the filename of problem characters.
static int CleanFilename(const char *src, char *dst)
{
	char *p;
	char *e;

	// Point to the two end of the buffer.
	p = dst;
	e = dst + (SPIFS_FILENAME_LEN - 1);

	// Skip leading space.
	while (*src && isspace(*src)) src++;

	// Copy the source into the end ignoring special characters.
	while (*src && (p < e))
	{
		// Filter out special ASCII characters: '\/:*?"<>|
		switch (*src)
		{
			case '\'':
			case '\\':
			case '/':
			case ':':
			case '*':
			case '?':
			case '"':
			case '<':
			case '>':
			case '|':
				// Increment the source.
				src++;
				break;
			default:
				*p++ = *src++;
				break;
		}
	}

	// Back up p.
	p--;

	// Trim trailing space, but don't go too far!
	while ((p > dst) && isspace(*p)) p--;

	// p currently points at the last non-space, skip and terminate.
	*++p = '\0';

	// Return the resulting string length.
	return strlen(dst);
}

// Returns the file id associated with the filename or -1;
static INT16 spiFSFindFilename(S_SPIFLASH_HANDLER *spiFlashHandler, char *filename)
{
	INT16 sectorId;
  UINT32 sectorOffset;
	SPIFS_FILE_BLOCK fileBlock;
  char sectorFilename[SPIFS_FILENAME_LEN];

  // Loop over each sector to check for the file.
  for (sectorId = SPIFS_SECTOR_FIRST; sectorId < SPIFS_SECTOR_COUNT; ++sectorId)
  {
    // Get the byte offset of the sector.
    sectorOffset = SPIFS_SECTORID_TO_BYTEOFFSET(sectorId);

    // Read the fileblock at the start of the sector.
		SPIFlash_Read(spiFlashHandler, sectorOffset, (PUINT8) &fileBlock, sizeof(fileBlock));

    // Is this the first block in a file?
    if (SPIFS_FILEID_VALID(fileBlock.fileId) && (fileBlock.blockId == 0))
    {
			// Set the sector offset to the file name.
			sectorOffset += sizeof(fileBlock);

      // Read the filename.
			SPIFlash_Read(spiFlashHandler, sectorOffset, (PUINT8) sectorFilename, SPIFS_FILENAME_LEN);

      // If the filename matches, return the sector id as the file id.
      if (!strncmp(filename, sectorFilename, SPIFS_FILENAME_LEN)) return sectorId;
    }
  }

  return -1;
}

// Returns a random free sector to use as a file block.
static INT16 spiFSFreeBlock(S_SPIFLASH_HANDLER *spiFlashHandler)
{
	BOOL found;
	INT16 randomId;
	INT16 sectorId;
  UINT32 sectorOffset;
	SPIFS_FILE_BLOCK fileBlock;

	// Get a random sector id.
	randomId = (INT16) (osKernelSysTick() % SPIFS_SECTOR_COUNT);	
	if (randomId < SPIFS_SECTOR_FIRST) randomId = SPIFS_SECTOR_FIRST;

	// We start searching from the randomId.
	sectorId = randomId;

	// We initially haven't yet found a free sector.
	found = FALSE;
	
	// Loop over each sector looking for a free sector.
	do
	{
		// Get the byte offset of the sector.
    sectorOffset = SPIFS_SECTORID_TO_BYTEOFFSET(sectorId);

    // Read the fileblock at the start of the sector.
		SPIFlash_Read(spiFlashHandler, sectorOffset, (PUINT8) &fileBlock, sizeof(fileBlock));

		// Is this a free sector?  This is indicated with an invalid fileid.
		if (!SPIFS_FILEID_VALID(fileBlock.blockId))
		{
			// Make sure the block appears erased.  This will hopefully 
			// help reduce instances of a corrupt file system.
			if ((fileBlock.fileId != -1) || (fileBlock.blockId != -1) || 
					(fileBlock.blockSize != -1) || (fileBlock.nextId != -1))
			{
				// Erase the sector.
				SPIFlash_Erase4K(spiFlashHandler, (UINT16) sectorId, 1);
			}

			// We found a free sector.
			found = TRUE;
		}
		else
		{
			// Increment the sector id.
			++sectorId;

			// Wrap the sector as needed to the from of the sectors.
			if (sectorId >= SPIFS_SECTOR_COUNT) sectorId = SPIFS_SECTOR_FIRST;
		}
	} while (!found && (sectorId != randomId));

	// If not found, return -1 for the sector id of the block.
	if (!found) sectorId = -1;

	return sectorId;
}

// Erase each block associated with the file id.
static void spiFSEraseBlocks(S_SPIFLASH_HANDLER *spiFlashHandler, INT16 fileId)
{
	INT16 sectorId;
  UINT32 sectorOffset;
	SPIFS_FILE_BLOCK fileBlock;

	// The first sector id is the same as the file id.
	sectorId = fileId;
	
  // Loop over each sector in the file and erase the sector.
	while (SPIFS_SECTORID_VALID(sectorId))
	{
    // Get the byte offset of the sector.
    sectorOffset = SPIFS_SECTORID_TO_BYTEOFFSET(sectorId);

    // Read the fileblock at the start of the sector.
		SPIFlash_Read(spiFlashHandler, sectorOffset, (PUINT8) &fileBlock, sizeof(fileBlock));

		// Erase the sector.
		SPIFlash_Erase4K(spiFlashHandler, (UINT16) sectorId, 1);

		// Set the sector id to the next sector id.
		sectorId = fileBlock.nextId;
  }
}

// Determine the file size associated with the file id.
static UINT32 spiFSFileSize(S_SPIFLASH_HANDLER *spiFlashHandler, INT16 fileId)
{
	INT16 sectorId;
	UINT32 fileSize;
  UINT32 sectorOffset;
	SPIFS_FILE_BLOCK fileBlock;

	// Initialize the filesize.
	fileSize = 0;
	
	// The first sector id is the same as the file id.
	sectorId = fileId;
	
  // Loop over each sector in the file and add up the block sizes.
	while (SPIFS_SECTORID_VALID(sectorId))
	{
    // Get the byte offset of the sector.
    sectorOffset = SPIFS_SECTORID_TO_BYTEOFFSET(sectorId);

    // Read the fileblock at the start of the sector.
		SPIFlash_Read(spiFlashHandler, sectorOffset, (PUINT8) &fileBlock, sizeof(fileBlock));

		// Add the block size to the file size.
		fileSize += (UINT32) fileBlock.blockSize;
		
		// Set the sector id to the next sector id.
		sectorId = fileBlock.nextId;
  }

	// Subtract the filename.
	if (fileSize > SPIFS_FILENAME_LEN) fileSize -= SPIFS_FILENAME_LEN;
	
	return fileSize;
}

// Mounts the SPI flash file system.
BOOL spiFSMount(void)
{
	return TRUE;
}

// Unmounts the SPI flash file system.
void spiFSUnmount(void)
{
	// XXX Make sure all files are closed.
}

BOOL spiFSFormat(void)
{
	S_SPIFLASH_HANDLER spiFlashHandler;

	// Wait for exclusive access to the SPI master bus.
	osMutexWait(spiMasterMutex, osWaitForever);

	// Open the SPI flash device.
	SPIFlash_Open(SPI_FLASH_HANDLER, SPI_FLASH_DEVICE, SPI_FLASH_DIVIDER, &spiFlashHandler);

  // Erase the chip.
	SPIFlash_EraseChip(&spiFlashHandler);

	// Close the SPI flash device.
	SPIFlash_Close(&spiFlashHandler);

	// Release exclusive access to the SPI master bus.
	osMutexRelease(spiMasterMutex);

	return TRUE;
}

BOOL spiFSUnlink(const char *filename)
{
  INT16 fileId;
	char cleanname[SPIFS_FILENAME_LEN];
	S_SPIFLASH_HANDLER spiFlashHandler;

	// Clean the filename of problem characters.
	if (!CleanFilename(filename, cleanname)) return FALSE;

	// Wait for exclusive access to the SPI master bus.
	osMutexWait(spiMasterMutex, osWaitForever);

	// Open the SPI flash device.
	SPIFlash_Open(SPI_FLASH_HANDLER, SPI_FLASH_DEVICE, SPI_FLASH_DIVIDER, &spiFlashHandler);

  // Find a file id associated with the filename.
  fileId = spiFSFindFilename(&spiFlashHandler, cleanname);

	// Erase each fileblock associated with the file.
	if (SPIFS_FILEID_VALID(fileId)) spiFSEraseBlocks(&spiFlashHandler, fileId);

	// Close the SPI flash device.
	SPIFlash_Close(&spiFlashHandler);

	// Release exclusive access to the SPI master bus.
	osMutexRelease(spiMasterMutex);

	return TRUE;
}

// Open the named file for reading or writing.
BOOL spiFSOpen(const char *filename, BOOL write, SPIFS_FILE *file)
{
	BOOL retval;
  INT16 fileId;
  UINT32 sectorOffset;
	SPIFS_FILE_BLOCK fileBlock;
	S_SPIFLASH_HANDLER spiFlashHandler;
	char cleanname[SPIFS_FILENAME_LEN];

	// Assume we fail.
	retval = FALSE;
	
  // Initialize the file structure.
  memset(file, 0, sizeof(SPIFS_FILE));

	// Clean the filename of problem characters.
	if (!CleanFilename(filename, cleanname)) return FALSE;

	// Wait for exclusive access to the SPI master bus.
	osMutexWait(spiMasterMutex, osWaitForever);

	// Open the SPI flash device.
	SPIFlash_Open(SPI_FLASH_HANDLER, SPI_FLASH_DEVICE, SPI_FLASH_DIVIDER, &spiFlashHandler);

  // Get the file id of any file with the same filename.
  fileId = spiFSFindFilename(&spiFlashHandler, cleanname);

  // Are we opening the file for writing?
  if (write)
  {
		// Erase each fileblock associated with the existing file.
		if (SPIFS_FILEID_VALID(fileId)) spiFSEraseBlocks(&spiFlashHandler, fileId);

		// Find a free sector.
		fileId = spiFSFreeBlock(&spiFlashHandler);

		// Did we find a free sector?
		if (SPIFS_FILEID_VALID(fileId))
		{
			// Prepare the sector for writing.
			file->write = TRUE;
			file->fileId = fileId;
			file->blockId = 0;
			file->blockSize = SPIFS_FILENAME_LEN;
			file->blockIndex = SPIFS_FILENAME_LEN;
			file->thisId = fileId;
			file->nextId = -1;
			file->position = SPIFS_FILENAME_LEN;

			// Get the byte offset of the sector at the start of the file.
			// The sector id is equal to the file id.
			sectorOffset = SPIFS_SECTORID_TO_BYTEOFFSET(fileId);

			// Write the file id.
			SPIFlash_Write(&spiFlashHandler, sectorOffset + SPIFS_FILEID_OFFSET, (PUINT8) &file->fileId, sizeof(INT16));

			// Write the block id.
			SPIFlash_Write(&spiFlashHandler, sectorOffset + SPIFS_BLOCKID_OFFSET, (PUINT8) &file->blockId, sizeof(INT16));

			// Write the file name.
			SPIFlash_Write(&spiFlashHandler, sectorOffset + SPIFS_FILENAME_OFFSET, (PUINT8) cleanname, SPIFS_FILENAME_LEN);

			// We won't write the other header information until we begin 
			// writing the next file block or the file is closed.
			
			// We succeeded opening the file for writing.
			retval = TRUE;
		}
  }
  else
  {
		// If we are reading a file we must have a valid file id.
		if (SPIFS_FILEID_VALID(fileId))
		{
			// Get the byte offset of the sector at the start of the file.
			// The sector id is equal to the file id.
			sectorOffset = SPIFS_SECTORID_TO_BYTEOFFSET(fileId);

			// Read in the first block header.
			SPIFlash_Read(&spiFlashHandler, sectorOffset, (PUINT8) &fileBlock, sizeof(fileBlock));

			// Initialize the file structure.
			file->write = FALSE;
			file->fileId = fileBlock.fileId;
			file->blockId = fileBlock.blockId;
			file->blockSize = fileBlock.blockSize;
			file->blockIndex = SPIFS_FILENAME_LEN;
			file->thisId = fileBlock.fileId;
			file->nextId = fileBlock.nextId;
			file->position = SPIFS_FILENAME_LEN;

			// Sanity check the block size.
			if (!SPIFS_BLOCKSIZE_VALID(file->blockSize)) file->blockSize = SPIFS_BLOCK_SIZE;

			// We succeeded opening the file for reading.
			retval = TRUE;
		}
  }

	// Close the SPI flash device.
	SPIFlash_Close(&spiFlashHandler);

	// Release exclusive access to the SPI master bus.
	osMutexRelease(spiMasterMutex);

	return retval;
}


BOOL spiFSClose(SPIFS_FILE *file)
{
  UINT32 sectorOffset;
	S_SPIFLASH_HANDLER spiFlashHandler;

	// Return if file isn't open.
	if (!SPIFS_FILEID_VALID(file->fileId)) return FALSE;

  // Is this file open for writing?
  if (file->write)
  {
    // Finalize the current sector.

		// Wait for exclusive access to the SPI master bus.
		osMutexWait(spiMasterMutex, osWaitForever);

		// Open the SPI flash device.
		SPIFlash_Open(SPI_FLASH_HANDLER, SPI_FLASH_DEVICE, SPI_FLASH_DIVIDER, &spiFlashHandler);

		// Get the byte offset of the current sector.
		sectorOffset = SPIFS_SECTORID_TO_BYTEOFFSET(file->thisId);

		// Write the block size.
		SPIFlash_Write(&spiFlashHandler, sectorOffset + SPIFS_BLOCKSIZE_OFFSET, (PUINT8) &file->blockSize, sizeof(INT16));

		// This is the final block in the file so leave the next id at -1.

		// Close the SPI flash device.
		SPIFlash_Close(&spiFlashHandler);

		// Release exclusive access to the SPI master bus.
		osMutexRelease(spiMasterMutex);
  }

  // Clear the file structure.
  memset(file, 0, sizeof(SPIFS_FILE));

	return TRUE;
}


BOOL spiFSRead(SPIFS_FILE *file, UINT8 *buffer, INT16 *count)
{
	INT16 left;
	INT16 read;
  UINT32 byteCount;
  UINT32 byteOffset;
	SPIFS_FILE_BLOCK fileBlock;
	S_SPIFLASH_HANDLER spiFlashHandler;

	// Keep track of how much data we read and still have to read.
	read = 0;
	left = *count;
	*count = 0;

	// Return if file isn't open.
	if (!SPIFS_FILEID_VALID(file->fileId)) return FALSE;

	// Return error if the file is not open for reading.
	if (file->write) return FALSE;

	// Wait for exclusive access to the SPI master bus.
	osMutexWait(spiMasterMutex, osWaitForever);

	// Open the SPI flash device.
	SPIFlash_Open(SPI_FLASH_HANDLER, SPI_FLASH_DEVICE, SPI_FLASH_DIVIDER, &spiFlashHandler);

	// Keep going while there is data left to read.
	while (left > 0)
	{
		// Do we have data still to read in this block?
		if (file->blockIndex < file->blockSize)
		{
			// Get the byte offset within the current sector to read from.
			byteOffset = SPIFS_SECTORID_TO_BYTEOFFSET(file->thisId) + SPIFS_HEADER_SIZE + file->blockIndex;

			// Determine the byte count to read.
			byteCount = (left > (file->blockSize - file->blockIndex)) ? file->blockSize - file->blockIndex : left;

			// Read in the data from the block.
			SPIFlash_Read(&spiFlashHandler, byteOffset, (PUINT8) buffer, byteCount);

			// Adjust the buffers, indices and counts.
			left -= byteCount;
			read += byteCount;
			buffer += byteCount;
			file->blockIndex += byteCount;
			file->position += byteCount;
		}
		else
		{
			// Make sure there is another buffer to read.
			if (SPIFS_FILEID_VALID(file->nextId))
			{
				// Get the byte offset of the next sector.
				byteOffset = SPIFS_SECTORID_TO_BYTEOFFSET(file->nextId);

				// Read in the block header.
				SPIFlash_Read(&spiFlashHandler, byteOffset, (PUINT8) &fileBlock, sizeof(fileBlock));

				// Update the file structure.
				file->blockId = fileBlock.blockId;
				file->blockSize = fileBlock.blockSize;
				file->blockIndex = 0;
				file->thisId = file->nextId;
				file->nextId = fileBlock.nextId;

				// Sanity check the block size.
				if (!SPIFS_BLOCKSIZE_VALID(file->blockSize)) file->blockSize = SPIFS_BLOCK_SIZE;
			}
			else
			{
				// No more data to read in the file.
				left = 0;
			}
		}
	}
		
	// Close the SPI flash device.
	SPIFlash_Close(&spiFlashHandler);

	// Release exclusive access to the SPI master bus.
	osMutexRelease(spiMasterMutex);

	// Return how much data we read.
	*count = read;

	// Return true if we read at least some data.
	return (read > 0) ? TRUE : FALSE;
}


BOOL spiFSWrite(SPIFS_FILE *file, const UINT8 *buffer, INT16 *count)
{
	INT16 left;
	INT16 wrote;
	INT16 nextId;
  UINT32 byteCount;
  UINT32 byteOffset;
	S_SPIFLASH_HANDLER spiFlashHandler;

	// Keep track of how much data we write and still have to read.
	wrote = 0;
	left = *count;
	*count = 0;

	// Return if file isn't open.
	if (!SPIFS_FILEID_VALID(file->fileId)) return FALSE;

	// Return error if the file is not open for writing.
	if (!file->write) return FALSE;
	
	// Wait for exclusive access to the SPI master bus.
	osMutexWait(spiMasterMutex, osWaitForever);

	// Open the SPI flash device.
	SPIFlash_Open(SPI_FLASH_HANDLER, SPI_FLASH_DEVICE, SPI_FLASH_DIVIDER, &spiFlashHandler);

	// Keep going while there is data left to write.
	while (left > 0)
	{
		// Is there still room to write in this block?
		if (file->blockIndex < SPIFS_BLOCK_SIZE)
		{
			// Get the byte offset within the current sector to write to.
			byteOffset = SPIFS_SECTORID_TO_BYTEOFFSET(file->thisId) + SPIFS_HEADER_SIZE + file->blockIndex;

			// Determine the byte count to write.
			byteCount = (left > (SPIFS_BLOCK_SIZE - file->blockIndex)) ? SPIFS_BLOCK_SIZE - file->blockIndex : left;

			// Write out the data to the block.
			SPIFlash_Write(&spiFlashHandler, byteOffset, (PUINT8) buffer, byteCount);

			// Adjust the buffers, indices and counts.
			left -= byteCount;
			wrote += byteCount;
			buffer += byteCount;
			file->blockIndex += byteCount;
			file->blockSize = file->blockIndex;
			file->position += byteCount;
		}
		else
		{
			// Find a free sector.
			nextId = spiFSFreeBlock(&spiFlashHandler);

			// Make sure there is another buffer to write.
			if (SPIFS_FILEID_VALID(nextId))
			{
				// Close out the current block.

				// Get the byte offset of the current sector.
				byteOffset = SPIFS_SECTORID_TO_BYTEOFFSET(file->thisId);

				// Write the block size.
				SPIFlash_Write(&spiFlashHandler, byteOffset + SPIFS_BLOCKSIZE_OFFSET, (PUINT8) &file->blockSize, sizeof(INT16));

				// Write the next sector id.
				SPIFlash_Write(&spiFlashHandler, byteOffset + SPIFS_NEXTID_OFFSET, (PUINT8) &nextId, sizeof(INT16));

				// Prepare this next sector for writing.
				file->blockId = file->blockId + 1;
				file->blockSize = 0;
				file->blockIndex = 0;
				file->thisId = nextId;
				file->nextId = -1;

				// Get the byte offset of the sector at the start of the file.
				// The sector id is equal to the file id.
				byteOffset = SPIFS_SECTORID_TO_BYTEOFFSET(nextId);

				// Write the file id.
				SPIFlash_Write(&spiFlashHandler, byteOffset + SPIFS_FILEID_OFFSET, (PUINT8) &file->fileId, sizeof(INT16));

				// Write the block id.
				SPIFlash_Write(&spiFlashHandler, byteOffset + SPIFS_BLOCKID_OFFSET, (PUINT8) &file->blockId, sizeof(INT16));

				// We won't write the other header information until we begin 
				// writing the next file block or the file is closed.
			}
			else
			{
				// Unable to write more data to the file system.
				left = 0;
			}
		}
	}
		
	// Close the SPI flash device.
	SPIFlash_Close(&spiFlashHandler);

	// Release exclusive access to the SPI master bus.
	osMutexRelease(spiMasterMutex);

	// Return how much data we wrote.
	*count = wrote;

	// Return true if we wrote at least some data.
	return (wrote > 0) ? TRUE : FALSE;
}

BOOL spiFSSeek(SPIFS_FILE *file, UINT32 position)
{
	BOOL done;
	INT16 thisId;
	SPIFS_FILE_BLOCK fileBlock;
	S_SPIFLASH_HANDLER spiFlashHandler;

	// Return if file isn't open.
	if (!SPIFS_FILEID_VALID(file->fileId)) return FALSE;

	// We can only seek while reading a file.
	if (file->write) return FALSE;

	// Add the filename to the position.
	position += SPIFS_FILENAME_LEN;

	// If we are already at the position, then don't bother.
	if (file->position == position) return TRUE;

	// Wait for exclusive access to the SPI master bus.
	osMutexWait(spiMasterMutex, osWaitForever);

	// Open the SPI flash device.
	SPIFlash_Open(SPI_FLASH_HANDLER, SPI_FLASH_DEVICE, SPI_FLASH_DIVIDER, &spiFlashHandler);

	// Get the first sector in the file from the file id.
	thisId = file->fileId;

	// We are not yet done.
	done = FALSE;

  // Loop over each sector in the file to find the right position.
	while (!done && SPIFS_SECTORID_VALID(thisId))
	{
    // Read the fileblock for this sector.
		SPIFlash_Read(&spiFlashHandler, SPIFS_SECTORID_TO_BYTEOFFSET(thisId), (PUINT8) &fileBlock, sizeof(fileBlock));

		// Update the file structure.
		file->blockId = fileBlock.blockId;
		file->blockSize = fileBlock.blockSize;
		file->blockIndex = fileBlock.blockSize;
		file->thisId = thisId;
		file->nextId = fileBlock.nextId;
		
		// Is the position within this block?
		if (position <= (file->position + (UINT32) fileBlock.blockSize))
		{
			// Update the block index based on the position.
			file->blockIndex = position - file->position;

			// Set the file position.
			file->position = position;

			// We are done.
			done = TRUE;
		}
		else
		{
			// Add the block size to the file position.
			file->position += (UINT32) fileBlock.blockSize;

			// Set this id to the next id.
			thisId = fileBlock.nextId;
		}
  }

	// Close the SPI flash device.
	SPIFlash_Close(&spiFlashHandler);

	// Release exclusive access to the SPI master bus.
	osMutexRelease(spiMasterMutex);

	return TRUE;
}

BOOL spiFSOpenDir(SPIFS_DIR *dir)
{
  // Initialize the file directory structure.
  memset(dir, 0, sizeof(SPIFS_DIR));

	return TRUE;
}

BOOL spiFSCloseDir(SPIFS_DIR *dir)
{
  // Reset the file directory structure.
  memset(dir, 0, sizeof(SPIFS_DIR));

	return TRUE;
}

BOOL spiFSReadDir(SPIFS_DIR *dir)
{
	BOOL retval;
  UINT32 sectorOffset;
	SPIFS_FILE_BLOCK fileBlock;
	S_SPIFLASH_HANDLER spiFlashHandler;

	// Increment the file id.
	dir->fileId += 1;
	
	// Wait for exclusive access to the SPI master bus.
	osMutexWait(spiMasterMutex, osWaitForever);

	// Open the SPI flash device.
	SPIFlash_Open(SPI_FLASH_HANDLER, SPI_FLASH_DEVICE, SPI_FLASH_DIVIDER, &spiFlashHandler);

	// We haven't found the next file yet.
	retval = FALSE;
	
  // Loop over each sector to check for the file.
  while (!retval && SPIFS_FILEID_VALID(dir->fileId))
  {
    // Get the byte offset of the sector.
    sectorOffset = SPIFS_SECTORID_TO_BYTEOFFSET(dir->fileId);

    // Read the fileblock at the start of the sector.
		SPIFlash_Read(&spiFlashHandler, sectorOffset, (PUINT8) &fileBlock, sizeof(fileBlock));

    // Is this the first block in a file?
    if (SPIFS_FILEID_VALID(fileBlock.fileId) && (fileBlock.blockId == 0))
    {
			// Yes. Set the sector offset to the file name.
			sectorOffset += sizeof(fileBlock);

      // Read the filename.
			SPIFlash_Read(&spiFlashHandler, sectorOffset, (PUINT8) dir->fileName, SPIFS_FILENAME_LEN);

			// Read the file size.
			dir->fileSize = spiFSFileSize(&spiFlashHandler, dir->fileId);

			// We found the next filename.
			retval = TRUE;
    }
		else
		{
			// No. Increment to the next file id.
			dir->fileId += 1;
		}
  }

	// Close the SPI flash device.
	SPIFlash_Close(&spiFlashHandler);

	// Release exclusive access to the SPI master bus.
	osMutexRelease(spiMasterMutex);

	return retval;
}

