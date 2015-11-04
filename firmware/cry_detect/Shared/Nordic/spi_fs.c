#include <string.h>
#include <ctype.h>
#include "global.h"
#include "nrf_rng.h"
#include "spi_flash.h"
#include "spi_fs.h"

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

// Returns true if sectorid is valid.
#define SPIFS_SECTORID_VALID(sid) \
		((sid >= SPIFS_SECTOR_FIRST) && (sid < SPIFS_SECTOR_COUNT))

// Returns true if fileid is valid.
#define SPIFS_FILEID_VALID(fid) \
		((fid >= SPIFS_SECTOR_FIRST) && (fid < SPIFS_SECTOR_COUNT))

// Returns true if block size is valid.
#define SPIFS_BLOCKSIZE_VALID(bs) \
		((bs > 0) && (bs <= SPIFS_BLOCK_SIZE))

// Converts a sector id to a byte offset in flash.
#define SPIFS_SECTORID_TO_BYTEOFFSET(id) \
		(((uint32_t) id) << 12)

// SPI block header structure.
typedef __packed struct _SPIFSFileBlock
{
  int16_t fileId;
  int16_t blockId;
  int16_t blockSize;
  int16_t nextId;
} SPIFSFileBlock;

// Cleans the filename of problem characters.
static int clean_filename(const char *src, char *dst)
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
static int16_t spifs_find_filename(SPIFlash *handler, char *filename)
{
	int16_t sectorId;
  uint32_t sectorOffset;
	SPIFSFileBlock fileBlock;
  char sectorFilename[SPIFS_FILENAME_LEN];

  // Loop over each sector to check for the file.
  for (sectorId = SPIFS_SECTOR_FIRST; sectorId < SPIFS_SECTOR_COUNT; ++sectorId)
  {
    // Get the byte offset of the sector.
    sectorOffset = SPIFS_SECTORID_TO_BYTEOFFSET(sectorId);

    // Read the fileblock at the start of the sector.
		spi_flash_read(handler, sectorOffset, (uint8_t *) &fileBlock, sizeof(fileBlock));

    // Is this the first block in a file?
    if (SPIFS_FILEID_VALID(fileBlock.fileId) && (fileBlock.blockId == 0))
    {
			// Set the sector offset to the file name.
			sectorOffset += sizeof(fileBlock);

      // Read the filename.
			spi_flash_read(handler, sectorOffset, (uint8_t *) sectorFilename, SPIFS_FILENAME_LEN);

      // If the filename matches, return the sector id as the file id.
      if (!strncmp(filename, sectorFilename, SPIFS_FILENAME_LEN)) return sectorId;
    }
  }

  return -1;
}

// Returns a random free sector to use as a file block.
static int16_t spifs_free_block(SPIFlash *handler)
{
	bool found;
	int16_t randomId;
	int16_t sectorId;
  uint32_t sectorOffset;
	SPIFSFileBlock fileBlock;
	extern uint32_t g_app_count;
	
	// Get a random sector id.
	randomId = nrf_rng_get_16() % SPIFS_SECTOR_COUNT;	
	if (randomId < SPIFS_SECTOR_FIRST) randomId = SPIFS_SECTOR_FIRST;

	// We start searching from the randomId.
	sectorId = randomId;

	// We initially haven't yet found a free sector.
	found = false;
	
	// Loop over each sector looking for a free sector.
	do
	{
		// Get the byte offset of the sector.
    sectorOffset = SPIFS_SECTORID_TO_BYTEOFFSET(sectorId);

    // Read the fileblock at the start of the sector.
		spi_flash_read(handler, sectorOffset, (uint8_t *) &fileBlock, sizeof(fileBlock));

		// Is this a free sector?  This is indicated with an invalid fileid.
		if (!SPIFS_FILEID_VALID(fileBlock.blockId))
		{
			// Make sure the block appears erased.  This will hopefully 
			// help reduce instances of a corrupt file system.
			if ((fileBlock.fileId != -1) || (fileBlock.blockId != -1) || 
					(fileBlock.blockSize != -1) || (fileBlock.nextId != -1))
			{
				// Erase the sector.
				spi_flash_erase_4K(handler, (UINT16) sectorId, 1);
			}

			// We found a free sector.
			found = true;
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
static void spifs_erase_blocks(SPIFlash *handler, int16_t fileId)
{
	int16_t sectorId;
  uint32_t sectorOffset;
	SPIFSFileBlock fileBlock;

	// The first sector id is the same as the file id.
	sectorId = fileId;
	
  // Loop over each sector in the file and erase the sector.
	while (SPIFS_SECTORID_VALID(sectorId))
	{
    // Get the byte offset of the sector.
    sectorOffset = SPIFS_SECTORID_TO_BYTEOFFSET(sectorId);

    // Read the fileblock at the start of the sector.
		spi_flash_read(handler, sectorOffset, (uint8_t *) &fileBlock, sizeof(fileBlock));

		// Erase the sector.
		spi_flash_erase_4K(handler, (UINT16) sectorId, 1);

		// Set the sector id to the next sector id.
		sectorId = fileBlock.nextId;
  }
}

// Determine the file size associated with the file id.
static uint32_t spifs_get_file_size(SPIFlash *handler, int16_t fileId)
{
	int16_t sectorId;
	uint32_t fileSize;
  uint32_t sectorOffset;
	SPIFSFileBlock fileBlock;

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
		spi_flash_read(handler, sectorOffset, (uint8_t *) &fileBlock, sizeof(fileBlock));

		// Add the block size to the file size.
		fileSize += (uint32_t) fileBlock.blockSize;
		
		// Set the sector id to the next sector id.
		sectorId = fileBlock.nextId;
  }

	// Subtract the filename.
	if (fileSize >= SPIFS_FILENAME_LEN) fileSize -= SPIFS_FILENAME_LEN;
	
	return fileSize;
}

// Mounts the SPI flash file system.
bool spifs_mount(void)
{
	return true;
}

// Unmounts the SPI flash file system.
void spifs_unmount(void)
{
	// XXX Make sure all files are closed.
}

bool spifs_format(void)
{
	SPIFlash handler;

	// Wait for exclusive access to the SPI master bus.
	osMutexWait(spiMasterMutex, osWaitForever);

	// Open the SPI flash device.
	spi_flash_open(SPI0, SPI_MODE0, false, Freq_1Mbps, &handler);

  // Erase the chip.
	spi_flash_erase_chip(&handler);

	// Close the SPI flash device.
	spi_flash_close(&handler);

	// Release exclusive access to the SPI master bus.
	osMutexRelease(spiMasterMutex);

	return true;
}

bool spifs_unlink(const char *filename)
{
  int16_t fileId;
	char cleanname[SPIFS_FILENAME_LEN];
	SPIFlash handler;

	// Clean the filename of problem characters.
	if (!clean_filename(filename, cleanname)) return false;

	// Wait for exclusive access to the SPI master bus.
	osMutexWait(spiMasterMutex, osWaitForever);

	// Open the SPI flash device.
	spi_flash_open(SPI0, SPI_MODE0, false, Freq_1Mbps, &handler);

  // Find a file id associated with the filename.
  fileId = spifs_find_filename(&handler, cleanname);

	// Erase each fileblock associated with the file.
	if (SPIFS_FILEID_VALID(fileId)) spifs_erase_blocks(&handler, fileId);

	// Close the SPI flash device.
	spi_flash_close(&handler);

	// Release exclusive access to the SPI master bus.
	osMutexRelease(spiMasterMutex);

	return true;
}

// Open the named file for reading or writing.
bool spifs_open(const char *filename, bool write, SPIFSFile *file)
{
	bool retval;
  int16_t fileId;
  uint32_t sectorOffset;
	SPIFSFileBlock fileBlock;
	SPIFlash handler;
	char cleanname[SPIFS_FILENAME_LEN];

	// Assume we fail.
	retval = false;
	
  // Initialize the file structure.
  memset(file, 0, sizeof(SPIFSFile));

	// Clean the filename of problem characters.
	if (!clean_filename(filename, cleanname)) return false;

	// Wait for exclusive access to the SPI master bus.
	osMutexWait(spiMasterMutex, osWaitForever);

	// Open the SPI flash device.
	spi_flash_open(SPI0, SPI_MODE0, false, Freq_1Mbps, &handler);

  // Get the file id of any file with the same filename.
  fileId = spifs_find_filename(&handler, cleanname);

  // Are we opening the file for writing?
  if (write)
  {
		// Erase each fileblock associated with the existing file.
		if (SPIFS_FILEID_VALID(fileId)) spifs_erase_blocks(&handler, fileId);

		// Find a free sector.
		fileId = spifs_free_block(&handler);

		// Did we find a free sector?
		if (SPIFS_FILEID_VALID(fileId))
		{
			// Prepare the sector for writing.
			file->write = true;
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
			spi_flash_write(&handler, sectorOffset + SPIFS_FILEID_OFFSET, (uint8_t *) &file->fileId, sizeof(int16_t));

			// Write the block id.
			spi_flash_write(&handler, sectorOffset + SPIFS_BLOCKID_OFFSET, (uint8_t *) &file->blockId, sizeof(int16_t));

			// Write the file name.
			spi_flash_write(&handler, sectorOffset + SPIFS_FILENAME_OFFSET, (uint8_t *) cleanname, SPIFS_FILENAME_LEN);

			// We won't write the other header information until we begin 
			// writing the next file block or the file is closed.
			
			// We succeeded opening the file for writing.
			retval = true;
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
			spi_flash_read(&handler, sectorOffset, (uint8_t *) &fileBlock, sizeof(fileBlock));

			// Initialize the file structure.
			file->write = false;
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
			retval = true;
		}
  }

	// Close the SPI flash device.
	spi_flash_close(&handler);

	// Release exclusive access to the SPI master bus.
	osMutexRelease(spiMasterMutex);

	return retval;
}


bool spifs_close(SPIFSFile *file)
{
  uint32_t sectorOffset;
	SPIFlash handler;

	// Return if file isn't open.
	if (!SPIFS_FILEID_VALID(file->fileId)) return false;

  // Is this file open for writing?
  if (file->write)
  {
    // Finalize the current sector.

		// Wait for exclusive access to the SPI master bus.
		osMutexWait(spiMasterMutex, osWaitForever);

		// Open the SPI flash device.
		spi_flash_open(SPI0, SPI_MODE0, false, Freq_1Mbps, &handler);

		// Get the byte offset of the current sector.
		sectorOffset = SPIFS_SECTORID_TO_BYTEOFFSET(file->thisId);

		// Write the block size.
		spi_flash_write(&handler, sectorOffset + SPIFS_BLOCKSIZE_OFFSET, (uint8_t *) &file->blockSize, sizeof(int16_t));

		// This is the final block in the file so leave the next id at -1.

		// Close the SPI flash device.
		spi_flash_close(&handler);

		// Release exclusive access to the SPI master bus.
		osMutexRelease(spiMasterMutex);
  }

  // Clear the file structure.
  memset(file, 0, sizeof(SPIFSFile));

	return true;
}


uint32_t spifs_read(SPIFSFile *file, uint8_t *buffer, uint32_t count)
{
	uint32_t read;
  uint32_t byteCount;
  uint32_t byteOffset;
	SPIFSFileBlock fileBlock;
	SPIFlash handler;

	// Keep track of how much data has been read.
	read = 0;

	// Return if file isn't open.
	if (!SPIFS_FILEID_VALID(file->fileId)) return false;

	// Return error if the file is not open for reading.
	if (file->write) return false;

	// Wait for exclusive access to the SPI master bus.
	osMutexWait(spiMasterMutex, osWaitForever);

	// Open the SPI flash device.
	spi_flash_open(SPI0, SPI_MODE0, false, Freq_1Mbps, &handler);

	// Keep going while there is data left to read.
	while (count > 0)
	{
		// Do we have data still to read in this block?
		if (file->blockIndex < file->blockSize)
		{
			// Get the byte offset within the current sector to read from.
			byteOffset = SPIFS_SECTORID_TO_BYTEOFFSET(file->thisId) + SPIFS_HEADER_SIZE + file->blockIndex;

			// Determine the byte count to read.
			byteCount = (count > (uint32_t) (file->blockSize - file->blockIndex)) ? (uint32_t) (file->blockSize - file->blockIndex) : count;

			// Read in the data from the block.
			spi_flash_read(&handler, byteOffset, (uint8_t *) buffer, byteCount);

			// Adjust the buffers, indices and counts.
			count -= byteCount;
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
				spi_flash_read(&handler, byteOffset, (uint8_t *) &fileBlock, sizeof(fileBlock));

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
				count = 0;
			}
		}
	}
		
	// Close the SPI flash device.
	spi_flash_close(&handler);

	// Release exclusive access to the SPI master bus.
	osMutexRelease(spiMasterMutex);

	// Return the number of bytes read.
	return read;
}


uint32_t spifs_write(SPIFSFile *file, const uint8_t *buffer, uint32_t count)
{
	int16_t nextId;
	uint32_t wrote;
  uint32_t byteCount;
  uint32_t byteOffset;
	SPIFlash handler;

	// Keep track of how much data has been written.
	wrote = 0;

	// Return if file isn't open.
	if (!SPIFS_FILEID_VALID(file->fileId)) return false;

	// Return error if the file is not open for writing.
	if (!file->write) return false;
	
	// Wait for exclusive access to the SPI master bus.
	osMutexWait(spiMasterMutex, osWaitForever);

	// Open the SPI flash device.
	spi_flash_open(SPI0, SPI_MODE0, false, Freq_1Mbps, &handler);

	// Keep going while there is data left to write.
	while (count > 0)
	{
		// Is there still room to write in this block?
		if (file->blockIndex < SPIFS_BLOCK_SIZE)
		{
			// Get the byte offset within the current sector to write to.
			byteOffset = SPIFS_SECTORID_TO_BYTEOFFSET(file->thisId) + SPIFS_HEADER_SIZE + file->blockIndex;

			// Determine the byte count to write.
			byteCount = (count > (uint32_t) (SPIFS_BLOCK_SIZE - file->blockIndex)) ? (uint32_t) (SPIFS_BLOCK_SIZE - file->blockIndex) : count;

			// Write out the data to the block.
			spi_flash_write(&handler, byteOffset, (uint8_t *) buffer, byteCount);

			// Adjust the buffers, indices and counts.
			count -= byteCount;
			wrote += byteCount;
			buffer += byteCount;
			file->blockIndex += byteCount;
			file->blockSize = file->blockIndex;
			file->position += byteCount;
		}
		else
		{
			// Find a free sector.
			nextId = spifs_free_block(&handler);

			// Make sure there is another buffer to write.
			if (SPIFS_FILEID_VALID(nextId))
			{
				// Close out the current block.

				// Get the byte offset of the current sector.
				byteOffset = SPIFS_SECTORID_TO_BYTEOFFSET(file->thisId);

				// Write the block size.
				spi_flash_write(&handler, byteOffset + SPIFS_BLOCKSIZE_OFFSET, (uint8_t *) &file->blockSize, sizeof(int16_t));

				// Write the next sector id.
				spi_flash_write(&handler, byteOffset + SPIFS_NEXTID_OFFSET, (uint8_t *) &nextId, sizeof(int16_t));

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
				spi_flash_write(&handler, byteOffset + SPIFS_FILEID_OFFSET, (uint8_t *) &file->fileId, sizeof(int16_t));

				// Write the block id.
				spi_flash_write(&handler, byteOffset + SPIFS_BLOCKID_OFFSET, (uint8_t *) &file->blockId, sizeof(int16_t));

				// We won't write the other header information until we begin 
				// writing the next file block or the file is closed.
			}
			else
			{
				// Unable to write more data to the file system.
				count = 0;
			}
		}
	}
		
	// Close the SPI flash device.
	spi_flash_close(&handler);

	// Release exclusive access to the SPI master bus.
	osMutexRelease(spiMasterMutex);

	// Return the number of bytes written.
	return wrote;
}

bool spifs_seek(SPIFSFile *file, uint32_t position)
{
	bool done;
	int16_t thisId;
	SPIFSFileBlock fileBlock;
	SPIFlash handler;

	// Return if file isn't open.
	if (!SPIFS_FILEID_VALID(file->fileId)) return false;

	// We can only seek while reading a file.
	if (file->write) return false;

	// Add the filename to the position.
	position += SPIFS_FILENAME_LEN;

	// If we are already at the position, then don't bother.
	if (file->position == position) return true;

	// Wait for exclusive access to the SPI master bus.
	osMutexWait(spiMasterMutex, osWaitForever);

	// Open the SPI flash device.
	spi_flash_open(SPI0, SPI_MODE0, false, Freq_1Mbps, &handler);

	// Get the first sector in the file from the file id.
	thisId = file->fileId;

	// We are not yet done.
	done = false;

  // Loop over each sector in the file to find the right position.
	while (!done && SPIFS_SECTORID_VALID(thisId))
	{
    // Read the fileblock for this sector.
		spi_flash_read(&handler, SPIFS_SECTORID_TO_BYTEOFFSET(thisId), (uint8_t *) &fileBlock, sizeof(fileBlock));

		// Update the file structure.
		file->blockId = fileBlock.blockId;
		file->blockSize = fileBlock.blockSize;
		file->blockIndex = fileBlock.blockSize;
		file->thisId = thisId;
		file->nextId = fileBlock.nextId;
		
		// Is the position within this block?
		if (position <= (file->position + (uint32_t) fileBlock.blockSize))
		{
			// Update the block index based on the position.
			file->blockIndex = position - file->position;

			// Set the file position.
			file->position = position;

			// We are done.
			done = true;
		}
		else
		{
			// Add the block size to the file position.
			file->position += (uint32_t) fileBlock.blockSize;

			// Set this id to the next id.
			thisId = fileBlock.nextId;
		}
  }

	// Close the SPI flash device.
	spi_flash_close(&handler);

	// Release exclusive access to the SPI master bus.
	osMutexRelease(spiMasterMutex);

	return true;
}

bool spifs_open_dir(SPIFSDir *dir)
{
  // Initialize the file directory structure.
  memset(dir, 0, sizeof(SPIFSDir));

	return true;
}

bool spifs_close_dir(SPIFSDir *dir)
{
  // Reset the file directory structure.
  memset(dir, 0, sizeof(SPIFSDir));

	return true;
}

bool spifs_read_dir(SPIFSDir *dir)
{
	bool retval;
  uint32_t sectorOffset;
	SPIFSFileBlock fileBlock;
	SPIFlash handler;

	// Increment the file id.
	dir->fileId += 1;
	
	// Wait for exclusive access to the SPI master bus.
	osMutexWait(spiMasterMutex, osWaitForever);

	// Open the SPI flash device.
	spi_flash_open(SPI0, SPI_MODE0, false, Freq_1Mbps, &handler);

	// We haven't found the next file yet.
	retval = false;
	
  // Loop over each sector to check for the file.
  while (!retval && SPIFS_FILEID_VALID(dir->fileId))
  {
    // Get the byte offset of the sector.
    sectorOffset = SPIFS_SECTORID_TO_BYTEOFFSET(dir->fileId);

    // Read the fileblock at the start of the sector.
		spi_flash_read(&handler, sectorOffset, (uint8_t *) &fileBlock, sizeof(fileBlock));

    // Is this the first block in a file?
    if (SPIFS_FILEID_VALID(fileBlock.fileId) && (fileBlock.blockId == 0))
    {
			// Yes. Set the sector offset to the file name.
			sectorOffset += sizeof(fileBlock);

      // Read the filename.
			spi_flash_read(&handler, sectorOffset, (uint8_t *) dir->fileName, SPIFS_FILENAME_LEN);

			// Read the file size.
			dir->fileSize = spifs_get_file_size(&handler, dir->fileId);

			// We found the next filename.
			retval = true;
    }
		else
		{
			// No. Increment to the next file id.
			dir->fileId += 1;
		}
  }

	// Close the SPI flash device.
	spi_flash_close(&handler);

	// Release exclusive access to the SPI master bus.
	osMutexRelease(spiMasterMutex);

	return retval;
}

