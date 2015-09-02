#include <stdio.h>
#include <string.h>
#include "Global.h"
#include "Driver/DrvAPU.h"
#include "Driver/DrvTimer.h"
#include "Audio/NuSoundEx.h"
#include "Audio/NuDACFilterEx.h"
#include "SpiFS.h"
#include "Sound.h"

//
// Global Variables
//

// Sound thread ID.
osThreadId soundThreadId;

//
// Local Variables and Defines
//

#define SOUND_SIGNAL_START		(0x01)
#define SOUND_SIGNAL_STOP			(0x02)
#define SOUND_SIGNAL_UPDATE		(0x04)

// Define the number of ADC PCM buffers.
#define PCM_BUFFER_COUNT 2

// Decoded PCM indices and buffers.
UINT8	pcmBufferDecodeIndex;
UINT8	pcmBufferPlaybackIndex;
UINT16 pcmBufferPlaybackPosition;	
INT16	pcmBufferData[PCM_BUFFER_COUNT][NUSOUNDEX_DECODE_SAMPLE_PER_FRAME];
BOOL pcmBufferReadyFlag[PCM_BUFFER_COUNT];

// Audio library work buffer (always keep when open lib.)
__align(4) UINT8 decodeWorkBuf[NUSOUNDEX_DECODE_WORK_BUF_SIZE];

// Audio library temp buffer (Per frame release.)
__align(4) UINT8 decodeTempBuf[NUSOUNDEX_DECODE_TEMP_BUF_SIZE];

//---------------------------------------------------------------------------
// Select APU upsampling scheme for speaker:
//  1-->No up sampling
//  2-->up x2 rate
//  4-->up x4 rate
//---------------------------------------------------------------------------
#define APU_UPSAMPLE		2

#if (defined(__N572__) && (APU_UPSAMPLE == 2))
__align(2) UINT8 filterUp2WorkBuf[NUDACFILTEREX_UP2_WORK_BUF_SIZE];
#endif
#if (defined(__N572__) && (APU_UPSAMPLE == 4))
__align(2) UINT8 filterUp4WorkBuf[NUDACFILTEREX_UP4_WORK_BUF_SIZE];			
#endif

#define SAMPLE_RATE_TABLE_COUNT 	5

// Table of prescale and divider values for each sample rate.
// Formula : sampleRateTable[][0] = 24,000,000 / (sampleRateTable[][1]+1) / sampleRateTable[][2]
// Extend array to contain support for additional sampling rates.
const UINT16 sampleRateTable[SAMPLE_RATE_TABLE_COUNT][3] = 
{
	{8000, 9, 150 * 2}, 
	{12000, 9, 100 * 2}, 
	{16000, 9, 75 * 2}, 
	{20000, 11, 50 * 2}, 
	{24000, 9, 50 * 2}
};

// Sound file name.
char soundFileName[SPIFS_FILENAME_LEN];

// Sound file structure.
SPIFS_FILE soundFile;

// Sound volume.
E_DRVAPU_VOLUME soundVolume = eDRVAPU_VOLUME_NEG_12DB;

//
// Local Functions
//

// Callback for the decoder to read compressed sound data.
static UINT32 soundDataCallback(void *buffer, UINT32 readPos, UINT32 count)
{
	INT16 length = (INT16) count;

	// Seek to the file position.
	if (spiFSSeek(&soundFile, readPos))
	{
		// Read the specified number of bytes.
		spiFSRead(&soundFile, (UINT8 *) buffer, &length);
	}

	return (UINT32) length;
}

// Callback for the decoder events embedded in the sound data.
static UINT32 soundEventCallback(UINT16 eventIndex, UINT16 eventSubIndex)
{
	// XXX Do nothing.
	return 0;
}

// Fill as many PCM buffers with sound data as possible.
static BOOL soundFillPCMBuffers(void)
{
	BOOL stopPlaying = FALSE;

	// We fill up as many buffers as we can before we hit the read buffer.
	while (!stopPlaying && (pcmBufferDecodeIndex != pcmBufferPlaybackIndex))
	{
		// Are we at the end of the current sound data?
		if (!NuSoundEx_DecodeIsEnd((UINT8 *) &decodeWorkBuf))
		{
			// Decode the next audio buffer.
			while (NuSoundEx_DecodeProcess(decodeWorkBuf, decodeTempBuf, pcmBufferData[pcmBufferDecodeIndex], soundDataCallback, soundEventCallback) == 0 );

			// Mark this buffer as now ready to be played.
			pcmBufferReadyFlag[pcmBufferDecodeIndex] = TRUE;

			// Adjust the playback buffer if needed.
			if (!pcmBufferReadyFlag[pcmBufferPlaybackIndex]) pcmBufferPlaybackIndex = pcmBufferDecodeIndex;
		}
		else
		{
			// Set the stop playing flag.
			stopPlaying = TRUE;
		}

		// Increment the buffer index within the range of buffers.
		pcmBufferDecodeIndex = (pcmBufferDecodeIndex + 1) % PCM_BUFFER_COUNT;
	}

	// Return FALSE if we should stop playing.
	return stopPlaying ? FALSE : TRUE;
}

//
// IRQ Functions
//

// Timer0 is APU sample rate controller.
void TMR0_IRQHandler()
{
   	DrvTimer_ClearIntFlagTmr0();
}

// Declared static to reduce stack usage.
static INT16 *pcmData;
static INT16 pcmSamples[4];

// APU IRQ handler for moving audio PCM pcmSamples to the APU registers.
void APU_IRQHandler()
{
	// Do we have a buffer we can play from?
	if (pcmBufferReadyFlag[pcmBufferPlaybackIndex])
	{
		// Point to the PCM data.
		pcmData = &pcmBufferData[pcmBufferPlaybackIndex][pcmBufferPlaybackPosition];
	 
#if (defined(__N572__) && (APU_UPSAMPLE == 2))
		// Apply the filter.
		NuDACFilterEx_Up2Process(filterUp2WorkBuf, *pcmData, &pcmSamples[0]);
		NuDACFilterEx_Up2Process(filterUp2WorkBuf, *(pcmData + 1), &pcmSamples[2]);
		pcmData = pcmSamples;

		// Increment our position within the APU read buffer.
		pcmBufferPlaybackPosition += 2;
#elif (defined(__N572__) && (APU_UPSAMPLE == 4))
		// Apply the filter.
		NuDACFilterEx_Up4Process(filterUp4WorkBuf, *pcmData, &pcmSamples[0]);
		pcmData = pcmSamples;

		// Increment our position within the APU read buffer.
		pcmBufferPlaybackPosition += 1;
#else
		// Increment our position within the APU read buffer.
		pcmBufferPlaybackPosition += 4;
#endif

		// Place sound data into channel 0 PCM input.
		if (DrvAPU_GetThresholdValue() == eDRVAPU_BUF3_TH)
		{ 
			DrvAPU_SetApuData13bitCh0(0, pcmData[0]); 
			DrvAPU_SetApuData13bitCh0(1, pcmData[1]);
			DrvAPU_SetApuData13bitCh0(2, pcmData[2]);
			DrvAPU_SetApuData13bitCh0(3, pcmData[3]);
			DrvAPU_SetIntThreshold(eDRVAPU_BUF7_TH);
		}
		else
		{
			DrvAPU_SetApuData13bitCh0(4, pcmData[0]); 
			DrvAPU_SetApuData13bitCh0(5, pcmData[1]);
			DrvAPU_SetApuData13bitCh0(6, pcmData[2]);
			DrvAPU_SetApuData13bitCh0(7, pcmData[3]);
			DrvAPU_SetIntThreshold(eDRVAPU_BUF3_TH);
		}

		// Have we reached the end of the read buffer?
		if (pcmBufferPlaybackPosition >= NUSOUNDEX_DECODE_SAMPLE_PER_FRAME)
		{
			// Yes.  Reset the the APU read buffer position.
			pcmBufferPlaybackPosition = 0;

			// Reset the play flag.
			pcmBufferReadyFlag[pcmBufferPlaybackIndex] = FALSE;

			// Increment to the next APU buffer.
			pcmBufferPlaybackIndex = (pcmBufferPlaybackIndex + 1) % PCM_BUFFER_COUNT;

			// Notify the sound thread to refill the PCM 
			// buffers that were just finished with.
			osSignalSet(soundThreadId, SOUND_SIGNAL_UPDATE);
		}
	}
  
	DrvAPU_ClearThresholdIntFlag();
}

// Initialize the sound buffers to start playing a sound.
static BOOL soundStart(void)
{
	UINT16 i;
	UINT16 divider;
	UINT16 preScaler;
	UINT16 sampleRate;
	UINT32 audioFilePosition;
	BOOL keepPlaying = FALSE;

	// Set the file position of the audio sample.
	audioFilePosition = 0;

	// Initialize the PCM buffer indicies.
	pcmBufferDecodeIndex = 0;
	pcmBufferPlaybackIndex = 1;
	pcmBufferPlaybackPosition = 0;

	// Initialize the audio decoder.
	if ((sampleRate = NuSoundEx_DecodeInitiate(decodeWorkBuf, decodeTempBuf, audioFilePosition, soundDataCallback)) != 0)
	{
		// Initialize the PCM buffers as not ready yet.
		for (i = 0; i < PCM_BUFFER_COUNT; i++) pcmBufferReadyFlag[i] = FALSE;

		// Fill the buffers with sound data.
		keepPlaying = soundFillPCMBuffers();
	}

	// Is there sound data to play?
	if (keepPlaying)
	{
		// Initialize the DAC filter.
#if (defined(__N572__)&&(APU_UPSAMPLE == 2))
		NuDACFilterEx_Up2Initial(filterUp2WorkBuf);
#elif (defined(__N572__)&&(APU_UPSAMPLE == 4))
		NuDACFilterEx_Up4Initial(filterUp4WorkBuf);
#endif

		// Set volume to the current level.
		DrvAPU_SetVolume(soundVolume);

		// Enabled DAC and timer to play audio data.
		DrvAPU_EnableDAC();

		// Enable timer 0 sample rate controller.
		DrvTimer_EnableIntTmr0();  

		// Determine the prescaler and divider for the current sample rate.
		divider = sampleRateTable[0][2];
		preScaler = sampleRateTable[0][1];
		for (i = 0 ; i < SAMPLE_RATE_TABLE_COUNT ; i++)
		{
			if (sampleRateTable[i][0] == sampleRate)
			{
				divider = sampleRateTable[i][2];
				preScaler = sampleRateTable[i][1];
				break;
			}
		}

		// Set APU prescale and divider for channel 0 playback.
		DrvAPU_StartCh0(preScaler, divider); 
	}

	return keepPlaying;
}

// End the sound and clean up the DAC.
static void soundEnd(void)
{
	// Disable the DAC.
	DrvAPU_DisableDAC();

	// Stop channel 0 playback.
	DrvAPU_StopCh0();

	// Disable the timer 0 sample rate controller.
	DrvTimer_DisableIntTmr0();
}

// Update the sound buffers.
static BOOL soundUpdate(void)
{
	BOOL keepPlaying;

	// Fill the sound buffers with additional PCM data.
	keepPlaying = soundFillPCMBuffers();

	// If we are finished cleanup the DAC.
	if (!keepPlaying) soundEnd();
	
	return keepPlaying;
}

//
// Global Functions
//

// Have the sound thread play the indicated sound file.
void soundPlay(char *fileName)
{
	// Copy the filename into the sound file name buffer.
	strlcpy(soundFileName, fileName, SPIFS_FILENAME_LEN);
	
	// Signal the sound thread to play the file.
	osSignalSet(soundThreadId, SOUND_SIGNAL_START);
}

// Stop playing any sound that is currently playing.
void soundStop(void)
{
	// Signal the sound thread to play the file.
	osSignalSet(soundThreadId, SOUND_SIGNAL_STOP);
}

// Set the volume (0-6) of the sound.
void soundSetVolume(UINT16 volume)
{
	switch (volume)
	{
		case 0:
			// Minimum volume.
			soundVolume = eDRVAPU_VOLUME_NEG_18DB;
			break;
		case 1:
			soundVolume = eDRVAPU_VOLUME_NEG_15DB;
			break;
		case 2:
			soundVolume = eDRVAPU_VOLUME_NEG_12DB;
			break;
		case 3:
			soundVolume = eDRVAPU_VOLUME_NEG_9DB;
			break;
		case 4:
			soundVolume = eDRVAPU_VOLUME_NEG_6DB;
			break;
		case 5:
			soundVolume = eDRVAPU_VOLUME_NEG_3DB;
			break;
		case 6:
			// Maximum volume.
			soundVolume = eDRVAPU_VOLUME_0DB;
			break;
	}
}

// Sound processing thread.
void soundThread(void const *argument)
{
	// Reset the sound filename.
	soundFileName[0] = 0;

	// Open the APU driver.
	DrvAPU_Open();

	// Enable power amplifier.
	DrvAPU_BypassPA(eDRVAPU_NONBYPASS_PA);
	DrvAPU_EnablePA();

	// Set maximum volume and APU read threshold value.
	DrvAPU_SetVolume(soundVolume);
	DrvAPU_SetIntThreshold(eDRVAPU_BUF3_TH);
	DrvAPU_EnableThresholdInt();

	// Set 16 bits audio data to APU channel 1. 
	DrvAPU_SetApuDataCh1(0x00);

	// Select timer0 clock source to 48MHz for APU
	DrvCLK_SetClkSrcTmr0(eDRVCLK_TIMERSRC_48M);

	// We loop forever processing sound events.
	for (;;)
	{
		// Wait for a signal indicating sound activity.
		osEvent evt = osSignalWait(0, osWaitForever);

		// Skip if this isn't an event signal.
		if (evt.status != osEventSignal) continue;

		// This is a signal from other threads to
		// start playing a sound file.
		if (evt.value.signals & SOUND_SIGNAL_START) 
		{
			// Open the indicated sound filename for reading.
			if (spiFSOpen(soundFileName, FALSE, &soundFile))
			{
				// Start playing the sound file.
				if (!soundStart())
				{
					// No sound data. We can now close the file.
					spiFSClose(&soundFile);
				}
			}

			// Reset the sound filename.
			soundFileName[0] = 0;
		}

		// This is a signal from other threads to
		// stop playing a sound file.
		if (evt.value.signals & SOUND_SIGNAL_STOP) 
		{
			// Finish up the DAC.
			soundEnd();
			
			// Close the file.
			spiFSClose(&soundFile);
		}

		// This is a signal from the IRQ handler to 
		// add more data to the PCM buffers.
		if (evt.value.signals & SOUND_SIGNAL_UPDATE) 
		{
			// Fill the PCM buffers with additional data.
			if (!soundUpdate())
			{
				// Finished. We can now close the file.
				spiFSClose(&soundFile);
			}
		}
	}
}
