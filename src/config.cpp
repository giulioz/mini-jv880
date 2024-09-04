//
// config.cpp
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
#include "config.h"

CConfig::CConfig (FATFS *pFileSystem)
:	m_Properties ("minijv880.ini", pFileSystem)
{
}

CConfig::~CConfig (void)
{
}

void CConfig::Load (void)
{
	m_Properties.Load ();
	
	m_SoundDevice = m_Properties.GetString ("SoundDevice", "pwm");

	if (m_SoundDevice == "hdmi") {
		m_nChunkSize = m_Properties.GetNumber ("ChunkSize", 384*6);
	}
	else
	{
#ifdef ARM_ALLOW_MULTI_CORE
		m_nChunkSize = m_Properties.GetNumber ("ChunkSize", 256);  // 128 per channel
#else
		m_nChunkSize = m_Properties.GetNumber ("ChunkSize", 1024);
#endif
	}
	m_nDACI2CAddress = m_Properties.GetNumber ("DACI2CAddress", 0);
	m_bChannelsSwapped = m_Properties.GetNumber ("ChannelsSwapped", 0) != 0;

	m_nMIDIBaudRate = m_Properties.GetNumber ("MIDIBaudRate", 31250);

	m_bProfileEnabled = m_Properties.GetNumber ("ProfileEnabled", 0) != 0;
}

const char *CConfig::GetSoundDevice (void) const
{
	return m_SoundDevice.c_str ();
}

unsigned CConfig::GetChunkSize (void) const
{
	return m_nChunkSize;
}

unsigned CConfig::GetDACI2CAddress (void) const
{
	return m_nDACI2CAddress;
}

bool CConfig::GetChannelsSwapped (void) const
{
	return m_bChannelsSwapped;
}

unsigned CConfig::GetMIDIBaudRate (void) const
{
	return m_nMIDIBaudRate;
}

bool CConfig::GetProfileEnabled (void) const
{
	return m_bProfileEnabled;
}
