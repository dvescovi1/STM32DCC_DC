#include "decoder.hpp"
#include <climits>
#include <cstdio>

#include "main.h"

extern "C" void decoder_main(void);

void Decoder::direction(uint16_t addr, bool dir) {}

void Decoder::speed(uint16_t addr, int32_t speed) {
  if (speed) {
    printf("\nDecoder: accelerate to speed step %d\n", speed);
    bsp_write_green_led(true);
  } else {
    printf("Decoder: stop\n");
    bsp_write_green_led(false);
  }
}

void Decoder::function(uint16_t addr, uint32_t mask, uint32_t state) {
  if (!(mask & 0b0'1000u)) return;
  else if (state & 0b0'1000u) {
    printf("Decoder: set function F3\n");
    bsp_write_yellow_led(true);
  } else {
    printf("Decoder: clear function F3\n");
    bsp_write_yellow_led(false);
  }
}

void Decoder::serviceModeHook(bool service_mode) {}

void Decoder::serviceAck() {}

void Decoder::transmitBiDi(std::span<uint8_t const> bytes) {}

uint8_t Decoder::readCv(uint32_t cv_addr, uint8_t) {
  if (cv_addr >= size(_cvs)) return 0u;
  return _cvs[cv_addr];
}

uint8_t Decoder::writeCv(uint32_t cv_addr, uint8_t byte) {
  if (cv_addr >= size(_cvs)) return 0u;
  return _cvs[cv_addr] = byte;
}

bool Decoder::readCv(uint32_t cv_addr, bool, uint32_t pos) { return false; }

bool Decoder::writeCv(uint32_t cv_addr, bool bit, uint32_t pos) {
  return false;
}

Decoder decoder;

//static uint32_t active_input_state = 0;


extern "C" int _gettimeofday(struct timeval* ptimeval,
                  void* ptimezone __attribute__((unused))) {
  uint32_t const tick_ms = HAL_GetTick();
  ptimeval->tv_sec = tick_ms / 1000;
  ptimeval->tv_usec = static_cast<suseconds_t>(tick_ms % 1000);
  return 0;
}

#if 0
extern "C" void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    static uint32_t last_edge = 0;

    if (htim->Instance == TIM15 && htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        uint32_t now = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        uint32_t delta = (now >= last_edge) ? (now - last_edge) : (0xFFFF - last_edge + now + 1);
        last_edge = now;

        decoder.receive(delta);

    }
}
#endif

extern "C" void TIM15_IRQHandler() {
   // Get captured value
  // Get captured value
  uint32_t const ccr = LL_TIM_IC_GetCaptureCH1(TIM15);

  // Toggle input TI1 and TI2
  LL_TIM_CC_DisableChannel(TIM15, LL_TIM_CHANNEL_CH1);
  LL_TIM_IC_SetActiveInput(
    TIM15,
    LL_TIM_CHANNEL_CH1,
    LL_TIM_IC_GetActiveInput(TIM15, LL_TIM_CHANNEL_CH1) ==
        LL_TIM_ACTIVEINPUT_DIRECTTI
      ? LL_TIM_ACTIVEINPUT_INDIRECTTI
      : LL_TIM_ACTIVEINPUT_DIRECTTI);
  LL_TIM_CC_EnableChannel(TIM15, LL_TIM_CHANNEL_CH1);

  // Subtract captured value from running counter
  __disable_irq();
  LL_TIM_SetCounter(TIM15, LL_TIM_GetCounter(TIM15) - ccr);
  __enable_irq();

  // Clear capture/compare interrupt flag
  while (LL_TIM_IsActiveFlag_CC1(TIM15)) LL_TIM_ClearFlag_CC1(TIM15);
  
  decoder.receive(ccr);
}


#if 0
extern "C" void TIM15_IRQHandler() {
   // Get captured value
  uint32_t ccr = HAL_TIM_ReadCapturedValue(&htim15, TIM_CHANNEL_1);

  HAL_TIM_IC_Stop(&htim15, TIM_CHANNEL_1);

  TIM_IC_InitTypeDef sConfigIC;
  sConfigIC.ICPolarity  = TIM_INPUTCHANNELPOLARITY_RISING; // Adjust if needed
  sConfigIC.ICSelection = (active_input_state == 0) ? TIM_ICSELECTION_INDIRECTTI : TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter    = 0;

  HAL_TIM_IC_ConfigChannel(&htim15, &sConfigIC, TIM_CHANNEL_1);
  HAL_TIM_IC_Start(&htim15, TIM_CHANNEL_1);

  active_input_state ^= 1;  // Toggle between 0 and 1

  __disable_irq();
  __HAL_TIM_SET_COUNTER(&htim15, __HAL_TIM_GET_COUNTER(&htim15) - ccr);
  __enable_irq();

  __HAL_TIM_CLEAR_FLAG(&htim15, TIM_FLAG_CC1);

  decoder.receive(ccr);
}
#endif
void decoder_main() {
  decoder.init();
  printf("SystemCoreClock = %lu Hz\r\n", SystemCoreClock);

#if defined(DEBUG)
  SCB->CCR &= ~SCB_CCR_UNALIGN_TRP_Msk;
#endif

  printf("\n\nBoot\n");
  for (;;) {
    decoder.execute();
    HAL_Delay(5u);
  }
}
