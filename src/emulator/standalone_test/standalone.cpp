#define STANDALONE

#include "../mcu.h"
#include "SDL.h"
#include <chrono>

SDL_AudioDeviceID sdl_audio;
SDL_Window *window;
SDL_Renderer *renderer;
SDL_Texture *texture;

MCU mcu;
bool working = true;
int nSamples = 0;
static SDL_mutex *work_thread_lock;
// 13333 iters per loop

int SDLCALL pcm_thread(void *data) {
  while (working) {
    while (mcu.sample_write_ptr >= nSamples) {
      // SDL_LockMutex(work_thread_lock);
    }
    mcu.pcm.PCM_Update(mcu.mcu.cycles);
  }
  return 0;
}

double avg = 0;
int cnt = 0;
void audio_callback(void * /*userdata*/, Uint8 *stream, int len) {
  auto start = std::chrono::high_resolution_clock::now();

  nSamples = len / sizeof(int16_t);
  // mcu.updateSC55(nSamples);

  mcu.sample_write_ptr = 0;
  while (mcu.sample_write_ptr < nSamples) {
    // for (size_t i = 0; i < 13333; i++) {
    // SDL_UnlockMutex(work_thread_lock);

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
  if (nSamples != mcu.sample_write_ptr)
    printf("expected %d rendered %d\n", nSamples, mcu.sample_write_ptr);

  memcpy(stream, mcu.sample_buffer, len);

  auto stop = std::chrono::high_resolution_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  avg = avg == 0 ? duration.count() : avg * 0.99 + duration.count() * 0.01;
  if (cnt++ == 100) {
    printf("%f\n", avg);
    cnt = 0;
  }
}

const int button_map_jv880[][2] = {
    SDL_SCANCODE_P,         MCU_BUTTON_PREVIEW, SDL_SCANCODE_LEFT,
    MCU_BUTTON_CURSOR_L,    SDL_SCANCODE_RIGHT, MCU_BUTTON_CURSOR_R,
    SDL_SCANCODE_TAB,       MCU_BUTTON_DATA,    SDL_SCANCODE_Q,
    MCU_BUTTON_TONE_SELECT, SDL_SCANCODE_A,     MCU_BUTTON_PATCH_PERFORM,
    SDL_SCANCODE_W,         MCU_BUTTON_EDIT,    SDL_SCANCODE_E,
    MCU_BUTTON_SYSTEM,      SDL_SCANCODE_R,     MCU_BUTTON_RHYTHM,
    SDL_SCANCODE_T,         MCU_BUTTON_UTILITY, SDL_SCANCODE_S,
    MCU_BUTTON_MUTE,        SDL_SCANCODE_D,     MCU_BUTTON_MONITOR,
    SDL_SCANCODE_F,         MCU_BUTTON_COMPARE, SDL_SCANCODE_G,
    MCU_BUTTON_ENTER,
};

int main() {
  SDL_Init(SDL_INIT_AUDIO | SDL_INIT_VIDEO | SDL_INIT_TIMER);

  work_thread_lock = SDL_CreateMutex();

  SDL_AudioSpec spec = {};
  SDL_AudioSpec spec_actual = {};
  spec.format = AUDIO_S16SYS;
  spec.freq = 32000 * 2;
  spec.channels = 2;
  spec.callback = audio_callback;
  spec.samples = 256 * 2;
  sdl_audio = SDL_OpenAudioDevice(NULL, 0, &spec, &spec_actual, 0);

  window = SDL_CreateWindow("JV880", SDL_WINDOWPOS_UNDEFINED,
                            SDL_WINDOWPOS_UNDEFINED, lcd_width, lcd_height,
                            SDL_WINDOW_SHOWN);
  renderer = SDL_CreateRenderer(window, -1, 0);
  texture =
      SDL_CreateTexture(renderer, SDL_PIXELFORMAT_BGR888,
                        SDL_TEXTUREACCESS_STREAMING, lcd_width, lcd_height);

  uint8_t rom1[ROM1_SIZE];
  uint8_t rom2[ROM2_SIZE];
  uint8_t nvram[NVRAM_SIZE];
  uint8_t pcm1[0x200000];
  uint8_t pcm2[0x200000];
  FILE *f = NULL;

  f = fopen("jv880_rom1.bin", "rb");
  fread(rom1, 1, ROM1_SIZE, f);
  fclose(f);
  f = fopen("jv880_rom2.bin", "rb");
  fread(rom2, 1, ROM2_SIZE, f);
  fclose(f);
  f = fopen("jv880_nvram.bin", "rb");
  fread(nvram, 1, NVRAM_SIZE, f);
  fclose(f);
  f = fopen("jv880_waverom1.bin", "rb");
  fread(pcm1, 1, 0x200000, f);
  fclose(f);
  f = fopen("jv880_waverom2.bin", "rb");
  fread(pcm2, 1, 0x200000, f);
  fclose(f);

  mcu.startSC55(rom1, rom2, pcm1, pcm2, nvram);

  SDL_Thread *thread = SDL_CreateThread(pcm_thread, "pcm thread", 0);
  SDL_PauseAudioDevice(sdl_audio, 0);

  while (working) {
    SDL_Event sdl_event;
    while (SDL_PollEvent(&sdl_event)) {
      if (sdl_event.type == SDL_KEYDOWN) {
        if (sdl_event.key.keysym.scancode == SDL_SCANCODE_COMMA)
          mcu.MCU_EncoderTrigger(0);
        if (sdl_event.key.keysym.scancode == SDL_SCANCODE_PERIOD)
          mcu.MCU_EncoderTrigger(1);
      }

      switch (sdl_event.type) {
      case SDL_QUIT:
        working = false;
        break;

      case SDL_KEYDOWN:
      case SDL_KEYUP: {
        if (sdl_event.key.repeat)
          continue;

        int mask = 0;
        auto button_map = button_map_jv880;
        auto button_size =
            sizeof(button_map_jv880) / sizeof(button_map_jv880[0]);
        for (size_t i = 0; i < button_size; i++) {
          if (button_map[i][0] == sdl_event.key.keysym.scancode)
            mask |= (1 << button_map[i][1]);
        }

        if (sdl_event.type == SDL_KEYDOWN)
          mcu.mcu_button_pressed |= mask;
        else
          mcu.mcu_button_pressed &= ~mask;
      }
      }
    }

    uint32_t *lcd_buffer = mcu.lcd.LCD_Update();
    SDL_UpdateTexture(texture, NULL, lcd_buffer, lcd_width * 4);
    SDL_RenderCopy(renderer, texture, NULL, NULL);
    SDL_RenderPresent(renderer);
  }

  SDL_DestroyMutex(work_thread_lock);
  SDL_CloseAudio();
  SDL_Quit();
}
