//
// kernel.cpp
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
#include "kernel.h"
#include <circle/logger.h>
#include <circle/synchronize.h>
#include <circle/gpiopin.h>
#include <assert.h>
#include <circle/usb/usbhcidevice.h>

LOGMODULE ("kernel");

CKernel *CKernel::s_pThis = 0;

CKernel::CKernel (void)
:	CStdlibAppStdio ("minijv880"),
	m_Config (&mFileSystem),
	m_GPIOManager (&mInterrupt),
 	m_I2CMaster (CMachineInfo::Get ()->GetDevice (DeviceI2CMaster), TRUE),
	m_pJV880 (0)
{
	s_pThis = this;

	// mActLED.Blink (5);	// show we are alive
}

CKernel::~CKernel(void)
{
	s_pThis = 0;
}

bool CKernel::Initialize (void)
{
	if (!CStdlibAppStdio::Initialize ())
	{
		return FALSE;
	}

	mLogger.RegisterPanicHandler (PanicHandler);

	if (!m_GPIOManager.Initialize ())
	{
		return FALSE;
	}

	if (!m_I2CMaster.Initialize ())
	{
		return FALSE;
	}

	m_Config.Load ();

	m_pUSB = new CUSBHCIDevice (&mInterrupt, &mTimer, TRUE);
	if (!m_pUSB->Initialize ())
	{
		return FALSE;
	}
	
	m_pJV880 = new CMiniJV880 (&m_Config, &mInterrupt, &m_GPIOManager, &m_I2CMaster, &mFileSystem, &mScreenUnbuffered);
	assert (m_pJV880);

	if (!m_pJV880->Initialize ())
	{
		return FALSE;
	}

	return TRUE;
}

CStdlibApp::TShutdownMode CKernel::Run (void)
{
	assert (m_pJV880);

	while (42 == 42)
	{
		boolean bUpdated = m_pUSB->UpdatePlugAndPlay ();

		m_pJV880->Process(bUpdated);

		if (mbScreenAvailable)
		{
			mScreen.Update ();
		}

		m_CPUThrottle.Update ();
	}

	return ShutdownHalt;
}

void CKernel::PanicHandler (void)
{
	LOGNOTE ("panic!");

	EnableIRQs ();

	if (s_pThis->mbScreenAvailable)
	{
		s_pThis->mScreen.Update (4096);
	}
}
