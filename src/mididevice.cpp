//
// mididevice.cpp
//
// MiniDexed - Dexed FM synthesizer for bare metal Raspberry Pi
// Copyright (C) 2022  The MiniDexed Team
//
// Original author of this class:
//	R. Stange <rsta2@o2online.de>
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.
//

#include <circle/logger.h>
#include "mididevice.h"
#include "minidexed.h"
#include "config.h"
#include <stdio.h>
#include <assert.h>

LOGMODULE ("mididevice");

#define MIDI_NOTE_OFF		0b1000
#define MIDI_NOTE_ON		0b1001
#define MIDI_AFTERTOUCH		0b1010			// TODO
#define MIDI_CHANNEL_AFTERTOUCH 0b1101   // right now Synth_Dexed just manage Channel Aftertouch not Polyphonic AT -> 0b1010
#define MIDI_CONTROL_CHANGE	0b1011
	#define MIDI_CC_BANK_SELECT_MSB		0
	#define MIDI_CC_MODULATION			1
	#define MIDI_CC_BREATH_CONTROLLER	2 
	#define MIDI_CC_FOOT_PEDAL 		4
	#define MIDI_CC_VOLUME				7
	#define MIDI_CC_PAN_POSITION		10
	#define MIDI_CC_BANK_SELECT_LSB		32
	#define MIDI_CC_BANK_SUSTAIN		64
	#define MIDI_CC_RESONANCE			71
	#define MIDI_CC_FREQUENCY_CUTOFF	74
	#define MIDI_CC_REVERB_LEVEL		91
	#define MIDI_CC_DETUNE_LEVEL		94
	#define MIDI_CC_ALL_SOUND_OFF		120
	#define MIDI_CC_ALL_NOTES_OFF		123
#define MIDI_PROGRAM_CHANGE	0b1100
#define MIDI_PITCH_BEND		0b1110

#define MIDI_SYSTEM_EXCLUSIVE_BEGIN	0xF0
#define MIDI_SYSTEM_EXCLUSIVE_END	0xF7
#define MIDI_TIMING_CLOCK	0xF8
#define MIDI_ACTIVE_SENSING	0xFE

CMIDIDevice::TDeviceMap CMIDIDevice::s_DeviceMap;

CMIDIDevice::CMIDIDevice (CMiniDexed *pSynthesizer, CConfig *pConfig)
:	m_pSynthesizer (pSynthesizer),
	m_pConfig (pConfig)
{
}

CMIDIDevice::~CMIDIDevice (void)
{
	m_pSynthesizer = 0;
}

void CMIDIDevice::MIDIMessageHandler (const u8 *pMessage, size_t nLength, unsigned nCable)
{
	// The packet contents are just normal MIDI data - see
	// https://www.midi.org/specifications/item/table-1-summary-of-midi-message

	if (false)
	{
		switch (nLength)
		{
		case 1:
			if (   pMessage[0] != MIDI_TIMING_CLOCK
			    && pMessage[0] != MIDI_ACTIVE_SENSING)
			{
				printf ("MIDI%u: %02X\n", nCable, (unsigned) pMessage[0]);
			}
			break;

		case 2:
			printf ("MIDI%u: %02X %02X\n", nCable,
				(unsigned) pMessage[0], (unsigned) pMessage[1]);
			break;

		case 3:
			printf ("MIDI%u: %02X %02X %02X\n", nCable,
				(unsigned) pMessage[0], (unsigned) pMessage[1],
				(unsigned) pMessage[2]);
			break;
		default:
			switch(pMessage[0])
			{
				case MIDI_SYSTEM_EXCLUSIVE_BEGIN:
					printf("MIDI%u: SysEx data length: [%d]:",nCable, uint16_t(nLength));
					for (uint16_t i = 0; i < nLength; i++)
					{
						if((i % 16) == 0)
							printf("\n%04d:",i);
						printf(" 0x%02x",pMessage[i]);
					}
					printf("\n");
					break;
				default:
					printf("MIDI%u: Unhandled MIDI event type %0x02x\n",nCable,pMessage[0]);
			}
			break;
		}
	}

	// Only for debugging:
/*
	if(pMessage[0]==MIDI_SYSTEM_EXCLUSIVE_BEGIN)
	{
		printf("MIDI%u: SysEx data length: [%d]:",nCable, uint16_t(nLength));
		for (uint16_t i = 0; i < nLength; i++)
		{
			if((i % 16) == 0)
				printf("\n%04d:",i);
			printf(" 0x%02x",pMessage[i]);
		}
		printf("\n");
	}
*/

	if (nLength < 2)
	{
		// LOGERR("MIDI message is shorter than 2 bytes!");
		return;
	}

	m_MIDISpinLock.Acquire ();

	u8 ucStatus  = pMessage[0];
	u8 ucChannel = ucStatus & 0x0F;
	u8 ucType    = ucStatus >> 4;

	m_MIDISpinLock.Release ();
}

void CMIDIDevice::AddDevice (const char *pDeviceName)
{
	assert (pDeviceName);

	assert (m_DeviceName.empty ());
	m_DeviceName = pDeviceName;
	assert (!m_DeviceName.empty ());

	s_DeviceMap.insert (std::pair<std::string, CMIDIDevice *> (pDeviceName, this));
}
