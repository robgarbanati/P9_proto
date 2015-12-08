#include "cmsis_os.h"
#include "Platform.h"
#include "Spi.h"
#include "Led.h"
#include "Driver/DrvGPIO.h"
#include "Driver/DrvPWM.h"
#include "Driver/DrvCLK.h"
#include "Driver/DrvTimer.h"
#include "Debug.h"
#include "Adc.h"

//
// Global Variables
//

// Eye thread.
osThreadId eyeThreadID;

sequence_step* current_sequence = NULL;
int current_sequence_index = 0;
int current_sequence_size = 0;
unsigned long current_sequence_next_count = 0;
BOOL current_sequence_repeat = FALSE;
UINT8 eye_sequence_param = 0;

UINT32 count = 0;
  
sequence_step eye_pattern_blink_half[] =
{
    {EYE_FULL_OPEN, 200},
    {EYE_PARTLY_OPEN_1, 3},
    {EYE_PARTLY_OPEN_2, 3},
    {EYE_PARTLY_OPEN_3, 3},
    {EYE_FULL_SHUT, 20},
    {EYE_PARTLY_OPEN_3, 3},
    {EYE_PARTLY_OPEN_2, 3},
    {EYE_PARTLY_OPEN_1, 3},
};

sequence_step eye_pattern_blink_full[] =
{
    {EYE_FULL_OPEN, 200},
    {TOP_EYELID_CLOSING_1, 2},
    {TOP_EYELID_CLOSING_2, 2},
    {TOP_EYELID_CLOSING_3, 2},
    {TOP_EYELID_CLOSING_4, 1},
    {TOP_EYELID_CLOSING_5, 1},
    {TOP_EYELID_CLOSING_6, 1},
    {TOP_EYELID_CLOSING_7, 1},
    {TOP_EYELID_CLOSING_8, 1},
    {EYE_FULL_SHUT, 20},
    {TOP_EYELID_CLOSING_8, 1},
    {TOP_EYELID_CLOSING_7, 2},
    {TOP_EYELID_CLOSING_6, 2},
    {TOP_EYELID_CLOSING_5, 2},
    {TOP_EYELID_CLOSING_4, 1},
    {TOP_EYELID_CLOSING_3, 1},
    {TOP_EYELID_CLOSING_2, 1},
    {TOP_EYELID_CLOSING_1, 1},
};

sequence_step eye_pattern_blink_full_fast[] =
{
    {EYE_FULL_OPEN, 10},
    {TOP_EYELID_CLOSING_1, 2},
    {TOP_EYELID_CLOSING_2, 2},
    {TOP_EYELID_CLOSING_3, 2},
    {TOP_EYELID_CLOSING_4, 1},
    {TOP_EYELID_CLOSING_5, 1},
    {TOP_EYELID_CLOSING_6, 1},
    {TOP_EYELID_CLOSING_7, 1},
    {TOP_EYELID_CLOSING_8, 1},
    {EYE_FULL_SHUT, 10},
    {TOP_EYELID_CLOSING_8, 1},
    {TOP_EYELID_CLOSING_7, 2},
    {TOP_EYELID_CLOSING_6, 2},
    {TOP_EYELID_CLOSING_5, 2},
    {TOP_EYELID_CLOSING_4, 1},
    {TOP_EYELID_CLOSING_3, 1},
    {TOP_EYELID_CLOSING_2, 1},
    {TOP_EYELID_CLOSING_1, 1},
};

sequence_step eye_pattern_circle[] =
{
    {0x0007, 1},
    {0x000E, 1},
    {0x001C, 1},
    {0x0038, 1},
    {0x0070, 1},
    {0x00E0, 1},
    {0x01C0, 1},
    {0x0380, 1},
    {0x0700, 1},
    {0x0E00, 1},
    {0x1C00, 1},
    {0x3800, 1},
    {0x7000, 1},
    {0xE000, 1},
    {0xC001, 1},
    {0x8003, 1},
};

//
// Local Declarations
//

//
// Mutex Declarations
//

// LED mutex.
osMutexId ledMutex;
osMutexDef(ledMutex);

//
// Global Functions
//

// Initialize main board LEDs.
void ledInit(void)
{
  // Create the LED mutex.
  ledMutex = osMutexCreate(osMutex(ledMutex));

  //
  // Configure onboard PWM control of eye and bi-color LED.
  //

  // Select the 48MHz clock for the PWM.
  DrvCLK_SetClkSrcPwm(eDRVCLK_PWMSRC_48M);  

  // Enable PWM clock.
  DrvPWM_Open();  

  // Disable the PWM inverter.
  DrvPWM_DisablePwmInverter();

  // Enable output PWM for LEDs
  DrvPWM_EnablePwmOutput0();

  // Enable output PWM for red green LED
  DrvPWM_EnablePwmOutput1();
  DrvPWM_EnablePwmOutput2();

  // Enable the PWM Timer 0 in toggle mode. This timer is shared all LEDs.
  // PWM freq = eDRVCLK_PWMSRC_48M / (u16PreScale*eClockDiv*CNR*2)
  // current PWM = 48M/(1*2*1024) = 23.4khz
  DrvPWM_StartPwmTimer(1, 2 - 1, eDRVPWM_CLOCK_DIV_1, 1024 - 1);

  // Reset the eye and bi-color duty cycles.
  DrvPWM_SetComparatorPwm0(0);
  DrvPWM_SetComparatorPwm1(0);
  DrvPWM_SetComparatorPwm2(0);

  //
  // Configure the IO Expander control of LEDs.
  //
  
  // Enable PWM on the eight ear LED outputs.
  spiWriteIOExpander8(0x3c, 0xff);

  // Set the PWM clock divider to 8MHz / 512.
  spiWriteIOExpander8(0x3d, 0x0f);

  // Disable PWM on the four LED outputs.
  spiWriteIOExpander8(0x3c, 0x00);

  // Initialize the duty cycles.
  spiWriteIOExpander8(0x1c, 0x00);
  spiWriteIOExpander8(0x1d, 0x00);
  spiWriteIOExpander8(0x1e, 0x00);
  spiWriteIOExpander8(0x1f, 0x00);

  // Turn off all eye LEDs.
  spiWriteIOExpander8(0x01, 0x00);
  spiWriteIOExpander8(0x02, 0x00);
}

void ledTopBrightness(UINT8 redLevel, UINT8 greenLevel)
{
  UINT16 redDutyCycle;
  UINT16 greenDutyCycle;

  // Wait for exclusive access to the LED hardware.
  osMutexWait(ledMutex, osWaitForever);

  // Set the duty cycles.
  redDutyCycle = ((UINT16) (255 - redLevel)) << 2;
  greenDutyCycle = ((UINT16) (255 - greenLevel)) << 2;
  DrvPWM_SetComparatorPwm1(redDutyCycle);  
  DrvPWM_SetComparatorPwm2(greenDutyCycle);

  // Release exclusive access to the LED hardware.
  osMutexRelease(ledMutex);
}

void ledLeftEarBrightness(UINT8 redLevel, UINT8 greenLevel, UINT8 blueLevel)
{
  UINT8 pwmEnable;
  UINT8 pwmOutput;

  // Wait for exclusive access to the LED hardware.
  osMutexWait(ledMutex, osWaitForever);

  // Configure the PWM enable.  When an LED is turned off, we completely
  // disable output to the LEDs as the PWM circuitry in the IO expander
  // allows a single pulse to the LEDs when set to the minimal level.
  pwmEnable = spiReadIOExpander8(0x3c);
  pwmEnable &= 0xf8;
  pwmEnable |= (redLevel > 0) ? 0x02 : 0x00;
  pwmEnable |= (greenLevel > 0) ? 0x04 : 0x00;
  pwmEnable |= (blueLevel > 0) ? 0x01 : 0x00;
  spiWriteIOExpander8(0x3c, pwmEnable);

  // Set the output for each color LED.
  pwmOutput = spiReadIOExpander8(0x00);
  pwmOutput &= 0xf8;
  pwmOutput |= (redLevel > 0) ? 0x00 : 0x02;
  pwmOutput |= (greenLevel > 0) ? 0x00 : 0x04;
  pwmOutput |= (blueLevel > 0) ? 0x00 : 0x01;
  spiWriteIOExpander8(0x00, pwmOutput);

  // Configure the PWM duty cycle for each color LED.
  spiWriteIOExpander8(0x1c, blueLevel);
  spiWriteIOExpander8(0x1d, redLevel);
  spiWriteIOExpander8(0x1e, greenLevel);

  // Release exclusive access to the LED hardware.
  osMutexRelease(ledMutex);
}

void ledRightEarBrightness(UINT8 redLevel, UINT8 greenLevel, UINT8 blueLevel)
{
  UINT8 pwmEnable;
  UINT8 pwmOutput;

  // Wait for exclusive access to the LED hardware.
  osMutexWait(ledMutex, osWaitForever);

  // Configure the PWM enable.  When an LED is turned off, we completely
  // disable output to the LEDs as the PWM circuitry in the IO expander
  // allows a single pulse to the LEDs when set to the minimal level.
  pwmEnable = spiReadIOExpander8(0x3c);
  pwmEnable &= 0xc7;
  pwmEnable |= (redLevel > 0) ? 0x10 : 0x00;
  pwmEnable |= (greenLevel > 0) ? 0x20 : 0x00;
  pwmEnable |= (blueLevel > 0) ? 0x08 : 0x00;
  spiWriteIOExpander8(0x3c, pwmEnable);

  // Set the output for each color LED.
  pwmOutput = spiReadIOExpander8(0x00);
  pwmOutput &= 0xc7;
  pwmOutput |= (redLevel > 0) ? 0x00 : 0x10;
  pwmOutput |= (greenLevel > 0) ? 0x00 : 0x20;
  pwmOutput |= (blueLevel > 0) ? 0x00 : 0x08;
  spiWriteIOExpander8(0x00, pwmOutput);

  // Configure the PWM duty cycle for each color LED.
  spiWriteIOExpander8(0x1f, blueLevel);
  spiWriteIOExpander8(0x2c, redLevel);
  spiWriteIOExpander8(0x2d, greenLevel);

  // Release exclusive access to the LED hardware.
  osMutexRelease(ledMutex);
}

UINT8 convertColor(UINT8 value)
{
  UINT8 color = 0;
  
  color = (value << 4) | value;
  
  return color;
}

void ledLeftEarlightConfigure(UINT8 red, UINT8 green, UINT8 blue)
{
  ledLeftEarBrightness(red, green, blue);
}

void ledRightEarlightConfigure(UINT8 red, UINT8 green, UINT8 blue)
{
  ledRightEarBrightness(red, green, blue);
}

void ledTopButtonLightConfigure(UINT8 red, UINT8 green)
{
  ledTopBrightness(red, green);  
}
void ledEyeBrightness(UINT8 level)
{
  UINT16 dutyCycle;

  // Wait for exclusive access to the LED hardware.
  osMutexWait(ledMutex, osWaitForever);

  // Set the duty cycles.
  dutyCycle = ((UINT16) (255 - level)) << 2;
  DrvPWM_SetComparatorPwm0(dutyCycle);  

  // Release exclusive access to the LED hardware.
  osMutexRelease(ledMutex);
}

void ledEyePattern(UINT8 pattern1, UINT8 pattern2)
{
  // Invert the patterns.
  pattern1 = ~pattern1;
  pattern2 = ~pattern2;
  
  // Wait for exclusive access to the LED hardware.
  osMutexWait(ledMutex, osWaitForever);

  // Set the pattern for the eye LEDs.
  spiWriteIOExpander8(0x01, pattern1);
  spiWriteIOExpander8(0x02, pattern2);

  // Release exclusive access to the LED hardware.
  osMutexRelease(ledMutex);
}

void ledEyePattern16(UINT16 pattern)
{
  // Invert the patterns.
  UINT8 pattern1 = ~(UINT8)((pattern & 0xFF00) >> 8);
  UINT8 pattern2 = ~(UINT8)(pattern & 0xFF);
  
  // Wait for exclusive access to the LED hardware.
  osMutexWait(ledMutex, osWaitForever);

  // Set the pattern for the eye LEDs.
  spiWriteIOExpander8(0x01, pattern1);
  spiWriteIOExpander8(0x02, pattern2);

  // Release exclusive access to the LED hardware.
  osMutexRelease(ledMutex);
}


void ledEyeBrightnessPattern(UINT8 level, UINT16 pattern)
{
  // Set the eye brightness.
  ledEyeBrightness(level);

  // Set the eye pattern.
  ledEyePattern((UINT8) pattern, (UINT8) (pattern >> 8));
}

  void TMR2_IRQHandler()
  {
    // Clear the timer 2 interrupt flag.
    DrvTimer_ClearIntFlagTmr2();

    // Signal the waiting control thread.
    osSignalSet(eyeThreadID, EYE_INTERRUPT_SIGNAL);
  }

  static void eyeTimerInit(UINT32 ticksPerSec)
  {
    UINT16 preScale;
    UINT32 clockSrc;
    UINT32 compareValue;

    // Set the timer 2 clock source.
    DrvCLK_SetClkSrcTmr2(eDRVCLK_TIMERSRC_48M);
    
    // Get the clock source frequency.
    clockSrc = DrvCLK_GetClkTmr2();
    
    // Sanity check the values.
    if ((clockSrc < 2) || (ticksPerSec == 0)) return;

    // Determine prescale and compare values that generate the correct timer period.
    for (preScale = 1; preScale < 256; preScale++)
    {
      // Generate the compare value for this prescale value.
      compareValue = clockSrc / (ticksPerSec * preScale);

      // The compare value must > 1 and < 65536.  If not, keep going.
      if ((compareValue > 1) && (compareValue < 65536))
        break;
    }

    // Make sure we generated a valid valid prescale and compare value?
    if (compareValue >= 65536 || compareValue <= 1) return;

    // Configure timer 2 in periodic mode.
    DrvTimer_OpenTmr2(DRVTIMER_PERIODIC_MODE | preScale, (UINT16) compareValue);

    // Enable timer 2 interrupt.
    DrvTimer_EnableIntTmr2();
    DrvTimer_EnableTmr2();
  }

#define SOUND_BUFFER_SIZE 19
#define SOUND_BUFFER_OLD_SIZE 6
#define SOUND_BUFFER_CURRENT_SIZE 3
#define SOUND_BUFFER_NEW_SIZE 6

INT16 sound_buffer[SOUND_BUFFER_SIZE];
UINT8 sound_buffer_index = 0;
BOOL clap = FALSE;

extern UINT8 clap_level;

void detect_clap(UINT8 index)
{
  UINT8 i = 0;
  INT16 max_old = 0;
  INT16 max_current = 0;
  INT16 max_new = 0;
  static INT32 lastCount = 0;

  max_old = sound_buffer[index];
  for(i = 0; i < SOUND_BUFFER_OLD_SIZE; i++)
  {
    if(max_old < sound_buffer[index])
      max_old = sound_buffer[index];
    index++;
    if(index >= SOUND_BUFFER_SIZE) index = 0;
  }
  
  max_current = sound_buffer[index];
  for(i = 0; i < SOUND_BUFFER_CURRENT_SIZE; i++)
  {
    if(max_current < sound_buffer[index])
      max_current = sound_buffer[index];
    index++;
    if(index >= SOUND_BUFFER_SIZE) index = 0;
  }
  
  max_new = sound_buffer[index];
  for(i = 0; i < SOUND_BUFFER_NEW_SIZE; i++)
  {
    if(max_new < sound_buffer[index])
      max_new = sound_buffer[index];
    index++;
    if(index >= SOUND_BUFFER_SIZE) index = 0;
  }
  
  // DEBUG_PRINTF("%d %d %d\n", max_old, max_current, max_new);
  // Whenever we hear a clap, keep the clap indication on for 90 ms
  if((count - lastCount) == 9)
    clap = FALSE;
    
  if((count - lastCount) > 10)
  {
    if( (max_current - max_old) > clap_level )
    {
      if( (max_current - max_new) > clap_level )
      {
        clap = TRUE;
        lastCount = count;
        DEBUG_PRINTF("Clap\n");
      }
    }
  }
}

BOOL get_clap()
{
  return clap;
}

void ProcessMicrophone()
{
  INT16 micValue = adcGetMicrophone() - 400;
  if(micValue < 0) micValue = -1 * micValue;
  //DEBUG_PRINTF("micValue: %d \n", micValue);
  sound_buffer[sound_buffer_index] = micValue;
  sound_buffer_index++;
  if(sound_buffer_index >= SOUND_BUFFER_SIZE)
    sound_buffer_index = 0;
  
  detect_clap(sound_buffer_index);
}

// Thread that manages SPI communication with the robot body.
void eyeThread(void const *argument)
{
  osEvent evt;
  
  // Initialize the eye brightness and pattern.
  ledEyeBrightnessPattern(255, 0);

  // Initialize the timer for 100 ticks per second.
  eyeTimerInit(100);
  
  // Start with blinking sequence
  ledEyeSequence(SEQUENCE_FULL_BLINK, TRUE, 0);
  
  // Main loop that handles eye control
  for (;;)
  {
    count++;
    
    // Wait for timer to kick off a control loop.
    evt = osSignalWait(EYE_INTERRUPT_SIGNAL, osWaitForever);

    // Did we receive the signal?
    if (evt.status == osEventSignal)
    {
      // First get micrphone readings
      // Should move this to a different place, or rename this file and thread
      ProcessMicrophone();
      
      if(current_sequence == NULL)
      {
        continue;
      }

      if(count < current_sequence_next_count)
      {
        continue;
      }
    
      if(current_sequence_index < current_sequence_size)
      {
        ledEyePattern16(current_sequence[current_sequence_index].state);
        if(eye_sequence_param == 0)
          current_sequence_next_count += current_sequence[current_sequence_index].count;
        else
          current_sequence_next_count += current_sequence[current_sequence_index].count * eye_sequence_param;
        current_sequence_index++;
      }
      else if(current_sequence_repeat)
      {
        current_sequence_index = 0;
        current_sequence_next_count = count;
      }
      else
      {
        current_sequence = NULL;
        ledEyePattern16(EYE_FULL_OPEN);
      }
    }
  }
}


void ledEyeSequence(UINT8 sequence_num, BOOL repeat, UINT8 param)
{
  eye_sequence_param = param;
  
  if(sequence_num == SEQUENCE_ALL_OFF)
  {
    current_sequence = NULL;
    ledEyePattern16(EYE_FULL_SHUT);
  }
  else if(sequence_num == SEQUENCE_ALL_ON)
  {
    current_sequence = NULL;
    ledEyePattern16(EYE_FULL_OPEN);
  }
  else
  {
    if(sequence_num == SEQUENCE_HALF_BLINK)
    {
      current_sequence = eye_pattern_blink_half;
      current_sequence_size = SEQUENCE_HALF_BLINK_SIZE;
    }
    else if(sequence_num == SEQUENCE_FULL_BLINK)
    {
      current_sequence = eye_pattern_blink_full;
      current_sequence_size = SEQUENCE_FULL_BLINK_SIZE;
    }
    else if(sequence_num == SEQUENCE_CIRCLE)
    {
        current_sequence = eye_pattern_circle;
        current_sequence_size = SEQUENCE_CIRCLE_SIZE;
    }
    else if(sequence_num == SEQUENCE_FAST_BLINK)
    {
        current_sequence = eye_pattern_blink_full_fast;
        current_sequence_size = SEQUENCE_FAST_BLINK_SIZE;
    }
    else
    {
      current_sequence = NULL;
      ledEyePattern16(EYE_FULL_OPEN);
    }
    
    current_sequence_index = 0;
    current_sequence_next_count = count;
    current_sequence_repeat = repeat;
  }
}
