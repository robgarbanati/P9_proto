#include "Platform.h"
#include <stdlib.h>

#include "app_SinDrive.h"
#include "app_OptoSensor.h"

extern INT16 global_phy;
INT16 start_phy;
INT16 stop_phy;
INT16 delta_phy;
INT16 delta_abs;

INT16 offset_CW, offset_CCW, offset_average;
UINT8 offset_CW_saved = 0;
UINT8 offset_CCW_saved = 0;

extern UINT16 amplitudeFactor;
extern UINT32 driveAmplitude;
UINT16 correction;

void OptoSensor_process_transition(UINT8 pin_state)
{
	
	if (pin_state == HIGH)
	{
		start_phy = global_phy;	
	} else {
		stop_phy = global_phy;
		delta_phy = stop_phy - start_phy;
		
		correction = (driveAmplitude * 1000) / (FULL_TURN / 2);
		delta_phy  = (delta_phy * correction) / 1000;
		
		delta_abs = abs(delta_phy);
		if ((delta_abs < 425) && (delta_abs > 175) && (amplitudeFactor >= MAX_AMPLITUDE_FACTOR))
		{

			if (delta_phy >=0)
			{
				offset_CW = global_phy;
				offset_CW_saved = 1;
			} else {
				offset_CCW = global_phy;
				offset_CCW_saved = 1;
			}
			
			if (offset_CW_saved && offset_CCW_saved)
			{
				offset_CW_saved = 0;
				offset_CCW_saved = 0;
				
				offset_average = (offset_CW + offset_CCW) / 2;
				
				phy_measured_offset = offset_average;
			}
		}
	}
}

