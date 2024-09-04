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
#include <assert.h>
#include <circle/gpiopin.h>
#include <circle/logger.h>
#include <circle/memory.h>
#include <circle/sound/hdmisoundbasedevice.h>
#include <circle/sound/i2ssoundbasedevice.h>
#include <circle/sound/pwmsoundbasedevice.h>
#include <cmath>
#include <stdio.h>
#include <string.h>

LOGMODULE("minidexed");

CMiniDexed::CMiniDexed(CConfig *pConfig, CInterruptSystem *pInterrupt,
                       CGPIOManager *pGPIOManager, CI2CMaster *pI2CMaster,
                       FATFS *pFileSystem, CScreenDevice *mScreenUnbuffered)
    : CMultiCoreSupport(CMemorySystem::Get()), m_pConfig(pConfig),
      m_pFileSystem(pFileSystem), m_PCKeyboard(this, pConfig),
      m_SerialMIDI(this, pInterrupt, pConfig), m_bUseSerial(false),
      m_pSoundDevice(0), m_bChannelsSwapped(pConfig->GetChannelsSwapped()),
      m_GetChunkTimer("GetChunk", 1000000U * pConfig->GetChunkSize() / 4 / 32000),
      m_bProfileEnabled(m_pConfig->GetProfileEnabled()),
      m_ScreenUnbuffered(mScreenUnbuffered) {
  assert(m_pConfig);

  for (unsigned i = 0; i < CConfig::MaxUSBMIDIDevices; i++) {
    m_pMIDIKeyboard[i] = new CMIDIKeyboard(this, pConfig, i);
    assert(m_pMIDIKeyboard[i]);
  }

  // select the sound device
  const char *pDeviceName = pConfig->GetSoundDevice();
  if (strcmp(pDeviceName, "i2s") == 0) {
    LOGNOTE("I2S mode");
    m_pSoundDevice = new CI2SSoundBaseDevice(
        pInterrupt, 32000, pConfig->GetChunkSize(),
        false, pI2CMaster, pConfig->GetDACI2CAddress(),
        CI2SSoundBaseDevice::DeviceModeTXOnly,
        2); // 2 channels - L+R
  } else if (strcmp(pDeviceName, "hdmi") == 0) {
#if RASPPI == 5
    LOGNOTE("HDMI mode NOT supported on RPI 5.");
#else
    LOGNOTE("HDMI mode");

    m_pSoundDevice = new CHDMISoundBaseDevice(
        pInterrupt, 32000, pConfig->GetChunkSize());

    // The channels are swapped by default in the HDMI sound driver.
    // TODO: Remove this line, when this has been fixed in the driver.
    m_bChannelsSwapped = !m_bChannelsSwapped;
#endif
  } else {
    LOGNOTE("PWM mode");

    m_pSoundDevice = new CPWMSoundBaseDevice(
        pInterrupt, 32000, pConfig->GetChunkSize());
  }
};

bool CMiniDexed::Initialize(void) {
  assert(m_pConfig);
  assert(m_pSoundDevice);

  LOGNOTE("Loading emu files");
  uint8_t *rom1 = (uint8_t *)malloc(ROM1_SIZE);
  uint8_t *rom2 = (uint8_t *)malloc(ROM2_SIZE);
  uint8_t *nvram = (uint8_t *)malloc(NVRAM_SIZE);
  uint8_t *pcm1 = (uint8_t *)malloc(0x200000);
  uint8_t *pcm2 = (uint8_t *)malloc(0x200000);

  FIL f;
  unsigned int nBytesRead = 0;
  if (f_open(&f, "jv880_rom1.bin", FA_READ | FA_OPEN_EXISTING) != FR_OK) {
    LOGERR("Cannot open jv880_rom1.bin");
    return false;
  }
  f_read(&f, rom1, ROM1_SIZE, &nBytesRead);
  f_close(&f);
  if (f_open(&f, "jv880_rom2.bin", FA_READ | FA_OPEN_EXISTING) != FR_OK) {
    LOGERR("Cannot open jv880_rom1.bin");
    return false;
  }
  f_read(&f, rom2, ROM2_SIZE, &nBytesRead);
  f_close(&f);
  if (f_open(&f, "jv880_nvram.bin", FA_READ | FA_OPEN_EXISTING) != FR_OK) {
    LOGERR("Cannot open jv880_rom1.bin");
    return false;
  }
  f_read(&f, nvram, NVRAM_SIZE, &nBytesRead);
  f_close(&f);
  if (f_open(&f, "jv880_waverom1.bin", FA_READ | FA_OPEN_EXISTING) != FR_OK) {
    LOGERR("Cannot open jv880_rom1.bin");
    return false;
  }
  f_read(&f, pcm1, 0x200000, &nBytesRead);
  f_close(&f);
  if (f_open(&f, "jv880_waverom2.bin", FA_READ | FA_OPEN_EXISTING) != FR_OK) {
    LOGERR("Cannot open jv880_rom1.bin");
    return false;
  }
  f_read(&f, pcm2, 0x200000, &nBytesRead);
  f_close(&f);
  LOGNOTE("Emu files loaded");

  mcu.startSC55(rom1, rom2, pcm1, pcm2, nvram);
  free(rom1);
  free(rom2);
  free(nvram);
  free(pcm1);
  free(pcm2);

  if (m_SerialMIDI.Initialize()) {
    LOGNOTE("Serial MIDI interface enabled");

    m_bUseSerial = true;
  }

  // setup and start the sound device
  int Channels = 2; // 16-bit Stereo
  // Need 2 x ChunkSize / Channel queue frames as the audio driver uses
  // two DMA channels each of ChunkSize and one single single frame
  // contains a sample for each of all the channels.
  //
  // See discussion here: https://github.com/rsta2/circle/discussions/453
  if (!m_pSoundDevice->AllocateQueueFrames(2 * m_pConfig->GetChunkSize() /
                                           Channels)) {
    LOGERR("Cannot allocate sound queue");

    return false;
  }

  m_pSoundDevice->SetWriteFormat(SoundFormatSigned16, Channels);

  m_nQueueSizeFrames = m_pSoundDevice->GetQueueSizeFrames();

  m_pSoundDevice->Start();

  CMultiCoreSupport::Initialize();

  return true;
}

double avg = 0;
int cnt = 0;
int nSamples = 0;

void CMiniDexed::Process(bool bPlugAndPlayUpdated) {
  for (unsigned i = 0; i < CConfig::MaxUSBMIDIDevices; i++) {
    assert(m_pMIDIKeyboard[i]);
    m_pMIDIKeyboard[i]->Process(bPlugAndPlayUpdated);
  }

  m_PCKeyboard.Process(bPlugAndPlayUpdated);

  if (m_bUseSerial) {
    m_SerialMIDI.Process();
  }
}

void CMiniDexed::Run(unsigned nCore) {
  assert(1 <= nCore && nCore < CORES);

  if (nCore == 1) {
    // screen update
    while (true) {
      uint32_t *lcd_buffer = mcu.lcd.LCD_Update();
      for (size_t y = 0; y < lcd_height; y++) {
        for (size_t x = 0; x < lcd_width; x++) {
          m_ScreenUnbuffered->SetPixel(x + 800, y + 100,
                                       lcd_buffer[y * lcd_width + x]);
        }
      }
    }
  } else if (nCore == 2) {
    // emulator
    while (true) {
      unsigned nFrames =
          m_nQueueSizeFrames - m_pSoundDevice->GetQueueFramesAvail();
      if (nFrames >= m_nQueueSizeFrames / 2) {
        // unsigned int startT = CTimer::GetClockTicks();

        nSamples = (int)nFrames * 2;
        // mcu.updateSC55(nSamples);

        mcu.sample_write_ptr = 0;
        while (mcu.sample_write_ptr < nSamples) {
          if (!mcu.mcu.ex_ignore)
            mcu.MCU_Interrupt_Handle();
          else
            mcu.mcu.ex_ignore = 0;

          if (!mcu.mcu.sleep)
            mcu.MCU_ReadInstruction();

          mcu.mcu.cycles += 12; // FIXME: assume 12 cycles per instruction

          mcu.TIMER_Clock(mcu.mcu.cycles);
          mcu.MCU_UpdateUART_RX();
          mcu.MCU_UpdateUART_TX();
          mcu.MCU_UpdateAnalog(mcu.mcu.cycles);

          // mcu.pcm.PCM_Update(mcu.mcu.cycles);
        }

        // unsigned int endT = CTimer::GetClockTicks();
        // avg = avg == 0 ? (endT - startT) : avg * 0.99 + (endT - startT) *
        // 0.01; if (cnt++ == 100) {
        //   LOGNOTE("%f\n", avg);
        //   cnt = 0;
        // }

        int len = nSamples * sizeof(int16_t);
        if (m_pSoundDevice->Write(mcu.sample_buffer, len) != len) {
          LOGERR("Sound data dropped");
        }
      }
    }
    // LOGNOTE("%d samples in %d time", nFrames, m_GetChunkTimer);
  } else if (nCore == 3) {
    // pcm chip
    while (true) {
      // while (mcu.sample_write_ptr >= nSamples) {
      // }
      mcu.pcm.PCM_Update(mcu.mcu.cycles);
    }
  }
}
