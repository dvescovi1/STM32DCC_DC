#include "bsp.h"
#include <errno.h>
#include <stddef.h>
#include <stdio.h>
#include <sys/time.h>
#include <sys/stat.h>

#if 0
#include "stm32h5xx_ll_icache.h"
#include "stm32h5xx_ll_pwr.h"
#include "stm32h5xx_ll_crs.h"
#include "stm32h5xx_ll_rcc.h"
#include "stm32h5xx_ll_bus.h"
#include "stm32h5xx_ll_system.h"
#include "stm32h5xx_ll_exti.h"
#include "stm32h5xx_ll_cortex.h"
#include "stm32h5xx_ll_utils.h"
#include "stm32h5xx_ll_dma.h"
#include "stm32h5xx_ll_tim.h"
#include "stm32h5xx_ll_usart.h"
#include "stm32h5xx_ll_gpio.h"
#include "stm32h5xx_ll_utils.h"
#include "stm32h5xx_ll_bus.h"


// Track pins
#define TRACK_N_PIN LL_GPIO_PIN_4
#define TRACK_P_PIN LL_GPIO_PIN_5
#define TRACK_GPIO_Port GPIOE
#define TRACK_N_BS_Pos GPIO_BSRR_BS4_Pos
#define TRACK_N_BR_Pos GPIO_BSRR_BR4_Pos
#define TRACK_P_BS_Pos GPIO_BSRR_BS5_Pos
#define TRACK_P_BR_Pos GPIO_BSRR_BR5_Pos
#endif

int _gettimeofday(struct timeval* ptimeval,
                  void* ptimezone __attribute__((unused))) {
  uint32_t const tick_ms = HAL_GetTick();
  ptimeval->tv_sec = tick_ms / 1000;
  ptimeval->tv_usec = (suseconds_t)tick_ms % 1000;
  return 0;
}


#if 0
void bsp_init_decoder(void) {

  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = TRACK_N_PIN | TRACK_P_PIN;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_DOWN;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_4;
  LL_GPIO_Init(TRACK_GPIO_Port, &GPIO_InitStruct);

  // Peripheral clock enable
  LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_TIM15);

  // TIM15 interrupt
  NVIC_SetPriority(TIM15_IRQn,
                   NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 0, 0));
  NVIC_EnableIRQ(TIM15_IRQn);

  LL_TIM_InitTypeDef TIM_InitStruct = {0};
  TIM_InitStruct.Prescaler = (uint16_t)(SystemCoreClock / 1000000 -1);
  TIM_InitStruct.CounterMode = LL_TIM_COUNTERMODE_UP;
  TIM_InitStruct.Autoreload = 65535;
  TIM_InitStruct.ClockDivision = LL_TIM_CLOCKDIVISION_DIV1;
  TIM_InitStruct.RepetitionCounter = 0;
  LL_TIM_Init(TIM15, &TIM_InitStruct);
  LL_TIM_DisableARRPreload(TIM15);
  LL_TIM_SetClockSource(TIM15, LL_TIM_CLOCKSOURCE_INTERNAL);
  LL_TIM_SetTriggerOutput(TIM15, LL_TIM_TRGO_RESET);
  LL_TIM_DisableMasterSlaveMode(TIM15);
  LL_TIM_IC_SetActiveInput(
    TIM15, LL_TIM_CHANNEL_CH1, LL_TIM_ACTIVEINPUT_DIRECTTI);
  LL_TIM_IC_SetPrescaler(TIM15, LL_TIM_CHANNEL_CH1, LL_TIM_ICPSC_DIV1);
  LL_TIM_IC_SetFilter(TIM15, LL_TIM_CHANNEL_CH1, LL_TIM_IC_FILTER_FDIV1);
  LL_TIM_IC_SetPolarity(TIM15, LL_TIM_CHANNEL_CH1, LL_TIM_IC_POLARITY_RISING);

  // Enable CH1
  LL_TIM_CC_EnableChannel(TIM15, LL_TIM_CHANNEL_CH1);
  LL_TIM_EnableIT_CC1(TIM15);
  LL_TIM_EnableCounter(TIM15);
}


// Handle timer interrupt for decoder
//
// Toggle input between TI1 and TI2, subtract captured value from running
// counter and clear capture/compare interrupt flag.
uint32_t bsp_decoder_irq(void) {
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

  return ccr;
}


uint32_t bsp_decoder_irq(void) {
  // Get captured value
  uint32_t ccr = HAL_TIM_ReadCapturedValue(&htim15, TIM_CHANNEL_1);

  // Toggle input polarity between direct and indirect
  TIM_IC_InitTypeDef sConfigIC;
  HAL_TIM_IC_Stop(&htim15, TIM_CHANNEL_1);
  HAL_TIM_IC_GetConfig(&htim15, TIM_CHANNEL_1, &sConfigIC);

  sConfigIC.ICPolarity = (sConfigIC.ICPolarity == TIM_INPUTCHANNELPOLARITY_RISING) ?
                          TIM_INPUTCHANNELPOLARITY_FALLING :
                          TIM_INPUTCHANNELPOLARITY_RISING;

  HAL_TIM_IC_ConfigChannel(&htim15, &sConfigIC, TIM_CHANNEL_1);
  HAL_TIM_IC_Start(&htim15, TIM_CHANNEL_1);

  // Subtract captured value from running counter
  __disable_irq();
  __HAL_TIM_SET_COUNTER(&htim15, __HAL_TIM_GET_COUNTER(&htim15) - ccr);
  __enable_irq();

  // Clear capture/compare interrupt flag
  __HAL_TIM_CLEAR_IT(&htim15, TIM_IT_CC1);

  return ccr;
}
#endif

