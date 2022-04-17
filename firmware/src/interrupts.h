#ifndef __INTERRUPTS_H
#define __INTERRUPTS_H

extern "C" {

void SysTick_Handler();
void ADC1_COMP_IRQHandler();
void PVD_IRQHandler();
void RCC_IRQHandler();
void RTC_IRQHandler();
void TIM2_IRQHandler();

}

extern volatile uint32_t g_sysTicks;

#endif  // __INTERRUPTS_H defined