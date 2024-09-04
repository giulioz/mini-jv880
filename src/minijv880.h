//
// minidexed.h
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
#ifndef _minijv880_h
#define _minijv880_h

#define ARM_ALLOW_MULTI_CORE

#include "config.h"
#include "emulator/mcu.h"
#include "midikeyboard.h"
#include "pckeyboard.h"
#include "perftimer.h"
#include "serialmididevice.h"
#include <circle/gpiomanager.h>
#include <circle/i2cmaster.h>
#include <circle/interrupt.h>
#include <circle/multicore.h>
#include <circle/screen.h>
#include <circle/sound/soundbasedevice.h>
#include <circle/spimaster.h>
#include <circle/spinlock.h>
#include <circle/types.h>
#include <fatfs/ff.h>
#include <stdint.h>
#include <string>

class CMiniJV880 : public CMultiCoreSupport {
public:
  CMiniJV880(CConfig *pConfig, CInterruptSystem *pInterrupt,
             CGPIOManager *pGPIOManager, CI2CMaster *pI2CMaster,
             FATFS *pFileSystem, CScreenDevice *mScreenUnbuffered);

  bool Initialize(void);
  void Process(bool bPlugAndPlayUpdated);

  virtual void Run(unsigned nCore) override;

  MCU mcu;

private:
  CConfig *m_pConfig;
  FATFS *m_pFileSystem;

  CMIDIKeyboard *m_pMIDIKeyboard[CConfig::MaxUSBMIDIDevices];
  CPCKeyboard m_PCKeyboard;
  CSerialMIDIDevice m_SerialMIDI;
  bool m_bUseSerial;

  CSoundBaseDevice *m_pSoundDevice;
  bool m_bChannelsSwapped;
  unsigned m_nQueueSizeFrames;

  CPerformanceTimer m_GetChunkTimer;
  bool m_bProfileEnabled;

  CScreenDevice *m_ScreenUnbuffered;
};

#endif
