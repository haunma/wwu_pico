// C-linkage interrupt-handler stubs to call the actual handlers inside C++ classes
#include "stm32l0xx.h"
#include "DPower.h"
#include "DRadio.h"
#include "DRTC.h"
#include <stdint.h>

volatile uint32_t g_sysTicks = 0;

extern "C" {

void SysTick_Handler()
{
    g_sysTicks++;
}


// Programmable voltage detector (detect over- and under-voltage conditions)
void PVD_IRQHandler()
{
    // Clear interrupt flag, then check if VDD was rising or falling
    EXTI->PR = (1 << 16);
    if (EXTI->FTSR & (1 << 16))
    {   // rising VDD
        g_power.HighVDD();
    }
    else
    {
        // falling VDD
    }
}


// LSE CSS (SiT1566 loss of clock)
void RCC_IRQHandler()
{
    // Disable LSE CSS, and clear the interrupt flag
    RCC->CSR &= (~RCC_CSR_LSECSSON);
    RCC->CICR = RCC_CICR_CSSLSEC;

    g_rtc.LSEFailure();
}


// RTC alarm or periodic wakeup timer
void RTC_IRQHandler()
{
    // Clear interrupt flag
    EXTI->PR = (1 << 17) | (1 << 20);

    if (RTC->ISR & RTC_ISR_WUTF)
    {
        RTC->ISR = 0;  // clear interrupt flag
        //g_rtc.RTCWake();
        g_radio.Tick();   // for sending WSPR
    }
    else
    {
        RTC->ISR = 0;  // clear interrupt flag
        g_rtc.RTCAlarm();
    }
}


// End-of-conversion on ADC1
void ADC1_COMP_IRQHandler()
{
    // Clear interrupt flags
    ADC1->ISR = 0xffffffff;
}


// Timer 2 rollover (tick for sending CW)
void TIM2_IRQHandler()
{
    TIM2->SR = 0;  // clear interrupt flag
    g_radio.Tick();
}

} // extern "C"
