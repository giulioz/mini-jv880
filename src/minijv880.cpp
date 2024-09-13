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
#include "minijv880.h"
#include <assert.h>
#include <circle/devicenameservice.h>
#include <circle/gpiopin.h>
#include <circle/logger.h>
#include <circle/memory.h>
#include <circle/sound/hdmisoundbasedevice.h>
#include <circle/sound/i2ssoundbasedevice.h>
#include <circle/sound/pwmsoundbasedevice.h>
#include <circle/usb/usbmidihost.h>
#include <stdio.h>
#include <string.h>

void set_pixel(unsigned char *screen, int x, int y, bool value) {
  if (!value)
    screen[(y / 8) * 128 + x] |= 1 << (y % 8);
  else
    screen[(y / 8) * 128 + x] &= ~(1 << (y % 8));
}

CMiniJV880 *CMiniJV880::s_pThis = 0;

LOGMODULE("minijv880");

CMiniJV880::CMiniJV880(CConfig *pConfig, CInterruptSystem *pInterrupt,
                       CGPIOManager *pGPIOManager, CI2CMaster *pI2CMaster,
                       FATFS *pFileSystem, CScreenDevice *mScreenUnbuffered)
    : CMultiCoreSupport(CMemorySystem::Get()), m_pConfig(pConfig),
      m_pFileSystem(pFileSystem), m_pSoundDevice(0),
      m_bChannelsSwapped(pConfig->GetChannelsSwapped()),
      m_ScreenUnbuffered(mScreenUnbuffered) {
  assert(m_pConfig);

  s_pThis = this;

  // select the sound device
  const char *pDeviceName = pConfig->GetSoundDevice();
  if (strcmp(pDeviceName, "i2s") == 0) {
    LOGNOTE("I2S mode");
    m_pSoundDevice = new CI2SSoundBaseDevice(
        pInterrupt, 32000, pConfig->GetChunkSize(), false, pI2CMaster,
        pConfig->GetDACI2CAddress(), CI2SSoundBaseDevice::DeviceModeTXOnly,
        2); // 2 channels - L+R
  } else if (strcmp(pDeviceName, "hdmi") == 0) {
#if RASPPI == 5
    LOGNOTE("HDMI mode NOT supported on RPI 5.");
#else
    LOGNOTE("HDMI mode");

    m_pSoundDevice =
        new CHDMISoundBaseDevice(pInterrupt, 32000, pConfig->GetChunkSize());

    // The channels are swapped by default in the HDMI sound driver.
    // TODO: Remove this line, when this has been fixed in the driver.
    m_bChannelsSwapped = !m_bChannelsSwapped;
#endif
  } else {
    LOGNOTE("PWM mode");

    m_pSoundDevice =
        new CPWMSoundBaseDevice(pInterrupt, 32000, pConfig->GetChunkSize());
  }

  screen_buffer = (u8 *)malloc(512);
};

bool CMiniJV880::Initialize(void) {
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

void CMiniJV880::Process(bool bPlugAndPlayUpdated) {
  uint32_t *lcd_buffer = mcu.lcd.LCD_Update();

  for (size_t y = 0; y < lcd_height; y++) {
    for (size_t x = 0; x < lcd_width; x++) {
      m_ScreenUnbuffered->SetPixel(x + 800, y + 100,
                                   lcd_buffer[y * lcd_width + x]);
    }
  }

  if (m_KompleteKontrol != 0) {
    m_KompleteKontrol->Update();

    uint32_t btn = 0;
    if (m_KompleteKontrol->status.left)
      btn |= 1 << MCU_BUTTON_CURSOR_L;
    else
      btn &= ~(1 << MCU_BUTTON_CURSOR_L);
    if (m_KompleteKontrol->status.right)
      btn |= 1 << MCU_BUTTON_CURSOR_R;
    else
      btn &= ~(1 << MCU_BUTTON_CURSOR_R);
    if (m_KompleteKontrol->status.loop)
      btn |= 1 << MCU_BUTTON_TONE_SELECT;
    else
      btn &= ~(1 << MCU_BUTTON_TONE_SELECT);
    if (m_KompleteKontrol->status.metro)
      btn |= 1 << MCU_BUTTON_MUTE;
    else
      btn &= ~(1 << MCU_BUTTON_MUTE);
    if (m_KompleteKontrol->status.tempo)
      btn |= 1 << MCU_BUTTON_DATA;
    else
      btn &= ~(1 << MCU_BUTTON_DATA);
    if (m_KompleteKontrol->status.undo)
      btn |= 1 << MCU_BUTTON_MONITOR;
    else
      btn &= ~(1 << MCU_BUTTON_MONITOR);
    if (m_KompleteKontrol->status.quantize)
      btn |= 1 << MCU_BUTTON_COMPARE;
    else
      btn &= ~(1 << MCU_BUTTON_COMPARE);
    if (m_KompleteKontrol->status.jstick_push)
      btn |= 1 << MCU_BUTTON_ENTER;
    else
      btn &= ~(1 << MCU_BUTTON_ENTER);
    if (m_KompleteKontrol->status.ideas)
      btn |= 1 << MCU_BUTTON_UTILITY;
    else
      btn &= ~(1 << MCU_BUTTON_UTILITY);
    if (m_KompleteKontrol->status.play)
      btn |= 1 << MCU_BUTTON_PREVIEW;
    else
      btn &= ~(1 << MCU_BUTTON_PREVIEW);
    if (m_KompleteKontrol->status.quantize)
      btn |= 1 << MCU_BUTTON_PATCH_PERFORM;
    else
      btn &= ~(1 << MCU_BUTTON_PATCH_PERFORM);
    if (m_KompleteKontrol->status.shift)
      btn |= 1 << MCU_BUTTON_EDIT;
    else
      btn &= ~(1 << MCU_BUTTON_EDIT);
    if (m_KompleteKontrol->status.scale)
      btn |= 1 << MCU_BUTTON_SYSTEM;
    else
      btn &= ~(1 << MCU_BUTTON_SYSTEM);
    if (m_KompleteKontrol->status.arp)
      btn |= 1 << MCU_BUTTON_RHYTHM;
    else
      btn &= ~(1 << MCU_BUTTON_RHYTHM);
    mcu.mcu_button_pressed = btn;

    if (m_KompleteKontrol->status.jstick_val > lastEncoderPos ||
        (lastEncoderPos == 15 && m_KompleteKontrol->status.jstick_val == 0))
      mcu.MCU_EncoderTrigger(1);
    else if (m_KompleteKontrol->status.jstick_val < lastEncoderPos ||
             (lastEncoderPos == 0 &&
              m_KompleteKontrol->status.jstick_val == 15))
      mcu.MCU_EncoderTrigger(0);
    lastEncoderPos = m_KompleteKontrol->status.jstick_val;

    for (size_t y = 0; y < 32; y++) {
      for (size_t x = 0; x < 128; x++) {
        int destX = (int)(((float)x / 128) * 820);
        int destY = (int)(((float)y / 32) * 100);
        int sum = 0;
        for (int py = -1; py <= 1; py++) {
          for (int px = -1; px <= 1; px++) {
            if ((destY + py) >= 0 && (destX + px) >= 0) {
              bool pixel =
                  mcu.lcd.lcd_buffer[destY + py][destX + px] == lcd_col1;
              sum += pixel;
            }
          }
        }

        bool pixel = sum > 0;
        // bool pixel = mcu.lcd.lcd_buffer[destY][destX] == lcd_col1;
        set_pixel(screen_buffer, x, y, pixel);

        // m_ScreenUnbuffered->SetPixel(x + 800, y + 300, pixel ? 0xFFFF : 0x0000);
      }
    }

    KompleteKontrolScreenCommand tmp;
    for (size_t row = 0; row < 4; row++) {
      for (size_t column = 0; column < 4; column++) {
        tmp.lengthRow = 1;
        tmp.lengthCol = 32;
        tmp.offsetRow = row;
        tmp.offsetCol = column * 32;
        memcpy(tmp.content, screen_buffer + row * 128 + column * 32, 32);
        m_KompleteKontrol->SendScreen(&tmp);
      }
    }
  }

  if (!bPlugAndPlayUpdated)
    return;

  if (m_pMIDIDevice == 0) {
    m_pMIDIDevice =
        (CUSBMIDIDevice *)CDeviceNameService::Get()->GetDevice("umidi1", FALSE);
    if (m_pMIDIDevice != 0) {
      m_pMIDIDevice->RegisterPacketHandler(USBMIDIMessageHandler);
      m_pMIDIDevice->RegisterRemovedHandler(DeviceRemovedHandler, this);
    }
  }

  if (m_KompleteKontrol == 0) {
    m_KompleteKontrol =
        (CUSBKompleteKontrolDevice *)CDeviceNameService::Get()->GetDevice(
            "kompletekontrol1", FALSE);
    if (m_KompleteKontrol != 0) {
      // m_KompleteKontrol->RegisterPacketHandler(s_pMIDIPacketHandler[m_nInstance]);
      m_KompleteKontrol->RegisterRemovedHandler(DeviceRemovedHandler, this);

      m_KompleteKontrol->DisableLocalControls();
      // m_KompleteKontrol->SendLEDs();

      // u8 content[256] = {0};
      // for (size_t i = 0; i < 256; i++) {
      //   content[i] = 0xff;
      // }
      // m_KompleteKontrol->SendScreenUpper(content);
      // m_KompleteKontrol->SendScreenLower(content);
    }
  }
}

void CMiniJV880::USBMIDIMessageHandler(unsigned nCable, u8 *pPacket,
                                       unsigned nLength) {
  // LOGERR("CMiniJV880::USBMIDIMessageHandler");
  CMiniJV880 *pThis = static_cast<CMiniJV880 *>(s_pThis);
  pThis->mcu.postMidiSC55(pPacket, nLength);
}

void CMiniJV880::DeviceRemovedHandler(CDevice *pDevice, void *pContext) {
  LOGERR("CMiniJV880::DeviceRemovedHandler");

  CMiniJV880 *pThis = static_cast<CMiniJV880 *>(pContext);
  assert(pThis != 0);

  if (pDevice == pThis->m_pMIDIDevice)
    pThis->m_pMIDIDevice = 0;
  if (pDevice == pThis->m_KompleteKontrol)
    pThis->m_KompleteKontrol = 0;
}

// double avg = 0;
// int cnt = 0;
int nSamples = 0;

void CMiniJV880::Run(unsigned nCore) {
  assert(1 <= nCore && nCore < CORES);

  if (nCore == 1) {
    // while (true) {
    //   if (m_pMIDIDevice != 0) {
    //     assert(m_pMIDIDevice->hostDevice != 0);
    //     m_pMIDIDevice->hostDevice->Update();
    //   }
    // }
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
