#include <stdio.h>
#include <string.h>
#include "cmsis_os.h"
#include "Command.h"
#include "Spi.h"
#include "Debug.h"
#include "Led.h"

// Manage memory for command messages.
osPoolDef(commandMemPool, 8, commandMessage);
osPoolId commandMemPool;

// Queue for command messages.
osMessageQDef(commandQueue, 8, commandMessage);
osMessageQId commandQueue;

// Command processing thread.
osThreadId commandProcessingThreadID;

// Initialize command processing.
void commandInit(void)
{
	// Initialize the message and memory pool for command processing.
  commandMemPool = osPoolCreate(osPool(commandMemPool));
  commandQueue = osMessageCreate(osMessageQ(commandQueue), NULL);
}

void commandSendResponse(commandMessage *response)
{
	// XXX Not yet implemented.
}

void ProcessEyeSequenceCommand(UINT8 data)
{
  UINT8 eyeSequence = data & 0x7F;
	BOOL eyeLoop = ((data & 0x80) != 0x00);
	ledEyeSequence(eyeSequence, eyeLoop, 0);
}

void ProcessEyeSequenceParamCommand(UINT8 data1, UINT8 data2)
{
  UINT8 eyeSequence = data1 & 0x7F;
	BOOL eyeLoop = ((data1 & 0x80) != 0x00);
	ledEyeSequence(eyeSequence, eyeLoop, data2);
}

UINT8 clap_level = 250;
UINT8 handleCommand(commandMessage *command, UINT8 index)
{
	UINT8 cmd = command->buffer[index];
	UINT8* buffer = command->buffer;
	
  switch(cmd) 
	{
		case COMMAND_TYPE_NONE_SYNC:
			return 1;
		
		case COMMAND_TYPE_BODY_MOTION:
			DEBUG_PRINTF("Motion command: \n");
			return 5;
		case COMMAND_TYPE_BODY_MOTION_WITH_PARAM:
			DEBUG_PRINTF("Motion command with param: \n");
			return 4;
		case COMMAND_TYPE_FRONT_LED:
			DEBUG_PRINTF("Front led command:  \n");
      return 4;
		case COMMAND_TYPE_TAIL_LED:
			DEBUG_PRINTF("Tail led command:  \n");
      return 2;
		case COMMAND_TYPE_AUDIO:
			DEBUG_PRINTF("Audio command:  \n");
		  return 2;
		case COMMAND_TYPE_HEAD_PAN:
			DEBUG_PRINTF("Head Pan command:  \n");
      return 3;
		case COMMAND_TYPE_HEAD_TILT:
			DEBUG_PRINTF("Head Tilt command:  \n");
      return 3;
		
		case COMMAND_TYPE_EYE_BRIGHTNESS:
			DEBUG_PRINTF("Eye Command: \n");
			ledEyeBrightness(buffer[index+1]);
			return 2;
		
		case COMMAND_TYPE_EYE_PATTERN:
			DEBUG_PRINTF("Eye Pattern Command: \n");
			ledEyeSequence(0, FALSE, 0);
			ledEyePattern(buffer[index+1], buffer[index+2]);
			return 3;
		
		case COMMAND_TYPE_EYE_SEQUENCE:
			DEBUG_PRINTF("Eye Sequence Command: \n");
			ProcessEyeSequenceCommand(buffer[index+1]);
			return 2;
		
    case COMMAND_TYPE_EYE_SEQUENCE_PARAM:
			DEBUG_PRINTF("Eye Sequence Param Command: \n");
			ProcessEyeSequenceParamCommand(buffer[index+1], buffer[index+2]);
			return 3;
		
		case COMMAND_TYPE_LEFT_EAR:
			DEBUG_PRINTF("Left Ear Command: \n");
			ledLeftEarlightConfigure(buffer[index+1], buffer[index+2], buffer[index+3]);
			return 4;
		
		case COMMAND_TYPE_RIGHT_EAR:
			DEBUG_PRINTF("Right Ear Command: \n");
			ledRightEarlightConfigure(buffer[index+1], buffer[index+2], buffer[index+3]);
			return 4;
		
		case COMMAND_TYPE_BUTTON_LED:
			DEBUG_PRINTF("Button Led Command: \n");
			ledTopButtonLightConfigure(buffer[index+1], buffer[index+2]);
			return 3;
		
		case COMMAND_TYPE_AUDIO_VOLUME:
			return 2;
		case COMMAND_TYPE_POWER_OFF:
			return 1;
		case COMMAND_TYPE_PAN_PGAIN:
			return 3;
		case COMMAND_TYPE_PAN_DGAIN:
			return 3;
		case COMMAND_TYPE_PAN_IGAIN:
			return 3;
		case COMMAND_TYPE_PAN_BOOST:
			return 3;
		case COMMAND_TYPE_TILT_PGAIN:
			return 3;
		case COMMAND_TYPE_TILT_DGAIN:
			return 3;
		case COMMAND_TYPE_TILT_IGAIN:
			return 3;
		case COMMAND_TYPE_TILT_BOOST:
			return 3;

		case COMMAND_TYPE_CLAP_LEVEL:
			clap_level = buffer[index+1];
		  return 2;
		case COMMAND_TYPE_DEBUG:
			return 17;
				
		default:
			DEBUG_PRINTF("Error parsing command:  \n");
	}

	return 100;
}

// Handle command event processing.
void commandProcessingThread(void const *argument)
{
	UINT8 currentIndex;

	for (;;)
	{
		// Wait for a command message.
		osEvent commandEvent = osMessageGet(commandQueue, osWaitForever);

		// Is this a command event?
		if (commandEvent.status == osEventMessage)
		{
			// Point to the message.
			commandMessage *message = (commandMessage *) commandEvent.value.p;

			// Execute all the commands
			currentIndex = 0;
			while (currentIndex < message->length) {
				currentIndex += handleCommand(message, currentIndex);
			}

			// Free the message.
			osPoolFree(commandMemPool, message);
		}
	}
}


