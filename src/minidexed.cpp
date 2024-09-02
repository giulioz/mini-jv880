//
// minidexed.cpp
//
// MiniDexed - Dexed FM synthesizer for bare metal Raspberry Pi
// Copyright (C) 2022  The MiniDexed Team
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
#include "minidexed.h"
#include <circle/logger.h>
#include <circle/memory.h>
#include <circle/sound/pwmsoundbasedevice.h>
#include <circle/sound/i2ssoundbasedevice.h>
#include <circle/sound/hdmisoundbasedevice.h>
#include <circle/gpiopin.h>
#include <string.h>
#include <stdio.h>
#include <assert.h>

LOGMODULE ("minidexed");

CMiniDexed::CMiniDexed (CConfig *pConfig, CInterruptSystem *pInterrupt,
			CGPIOManager *pGPIOManager, CI2CMaster *pI2CMaster, FATFS *pFileSystem)
:
	m_pConfig (pConfig),
	m_PCKeyboard (this, pConfig),
	m_SerialMIDI (this, pInterrupt, pConfig),
	m_bUseSerial (false),
	m_pSoundDevice (0),
	m_bChannelsSwapped (pConfig->GetChannelsSwapped ()),
	m_GetChunkTimer ("GetChunk",
			 1000000U * pConfig->GetChunkSize ()/2 / pConfig->GetSampleRate ()),
	m_bProfileEnabled (m_pConfig->GetProfileEnabled ())
{
	assert (m_pConfig);

	for (unsigned i = 0; i < CConfig::MaxUSBMIDIDevices; i++)
	{
		m_pMIDIKeyboard[i] = new CMIDIKeyboard (this, pConfig, i);
		assert (m_pMIDIKeyboard[i]);
	}

	// select the sound device
	const char *pDeviceName = pConfig->GetSoundDevice ();
	if (strcmp (pDeviceName, "i2s") == 0)
	{
		LOGNOTE ("I2S mode");
		m_pSoundDevice = new CI2SSoundBaseDevice (pInterrupt, pConfig->GetSampleRate (),
								pConfig->GetChunkSize (), false,
								pI2CMaster, pConfig->GetDACI2CAddress (),
								CI2SSoundBaseDevice::DeviceModeTXOnly,
								2);  // 2 channels - L+R
	}
	else if (strcmp (pDeviceName, "hdmi") == 0)
	{
#if RASPPI==5
		LOGNOTE ("HDMI mode NOT supported on RPI 5.");
#else
		LOGNOTE ("HDMI mode");

		m_pSoundDevice = new CHDMISoundBaseDevice (pInterrupt, pConfig->GetSampleRate (),
							   pConfig->GetChunkSize ());

		// The channels are swapped by default in the HDMI sound driver.
		// TODO: Remove this line, when this has been fixed in the driver.
		m_bChannelsSwapped = !m_bChannelsSwapped;
#endif
	}
	else
	{
		LOGNOTE ("PWM mode");

		m_pSoundDevice = new CPWMSoundBaseDevice (pInterrupt, pConfig->GetSampleRate (),
							  pConfig->GetChunkSize ());
	}
};

bool CMiniDexed::Initialize (void)
{
	assert (m_pConfig);
	assert (m_pSoundDevice);

	if (m_SerialMIDI.Initialize ())
	{
		LOGNOTE ("Serial MIDI interface enabled");

		m_bUseSerial = true;
	}
	
	LOGNOTE ("hehe1");
	
	// setup and start the sound device
	int Channels = 2;	// 16-bit Stereo
	// Need 2 x ChunkSize / Channel queue frames as the audio driver uses
	// two DMA channels each of ChunkSize and one single single frame
	// contains a sample for each of all the channels.
	//
	// See discussion here: https://github.com/rsta2/circle/discussions/453
	if (!m_pSoundDevice->AllocateQueueFrames (2 * m_pConfig->GetChunkSize () / Channels))
	{
		LOGERR ("Cannot allocate sound queue");

		return false;
	}

	m_pSoundDevice->SetWriteFormat (SoundFormatSigned16, Channels);

	m_nQueueSizeFrames = m_pSoundDevice->GetQueueSizeFrames ();

	m_pSoundDevice->Start ();

	LOGNOTE ("what");

	return true;
}

void CMiniDexed::Process (bool bPlugAndPlayUpdated)
{
	LOGNOTE ("process1");

	for (unsigned i = 0; i < CConfig::MaxUSBMIDIDevices; i++)
	{
		assert (m_pMIDIKeyboard[i]);
		m_pMIDIKeyboard[i]->Process (bPlugAndPlayUpdated);
	}

	m_PCKeyboard.Process (bPlugAndPlayUpdated);

	if (m_bUseSerial)
	{
		m_SerialMIDI.Process ();
	}

	if (m_bProfileEnabled)
	{
		m_GetChunkTimer.Dump ();
	}

	LOGNOTE ("process2");
}

void CMiniDexed::ProcessSound (void)
{
	assert (m_pSoundDevice);

	unsigned nFrames = m_nQueueSizeFrames - m_pSoundDevice->GetQueueFramesAvail ();
	if (nFrames >= m_nQueueSizeFrames/2)
	{
		if (m_bProfileEnabled)
		{
			m_GetChunkTimer.Start ();
		}

		// Convert single float array (mono) to int16 array
		int16_t tmp_int[nFrames];

		if (m_pSoundDevice->Write (tmp_int, sizeof(tmp_int)) != (int) sizeof(tmp_int))
		{
			LOGERR ("Sound data dropped");
		}

		if (m_bProfileEnabled)
		{
			m_GetChunkTimer.Stop ();
		}
	}
}
