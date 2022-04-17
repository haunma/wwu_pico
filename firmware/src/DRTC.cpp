#include "DRTC.h"
#include "stm32l0xx.h"

DRTC::DRTC() :
    LSEFailed(false), CalCount(0)
{
}


void DRTC::Init()
{
    // PWR clock enable
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;

    // PWR_CR: Enable RTC access
    PWR->CR |= PWR_CR_DBP;

    // Set RTC clock source to LSE and bypass LSE oscillator circuit
    RCC->CSR = RCC_CSR_RTCSEL_LSE | RCC_CSR_LSEBYP | RCC_CSR_LSEON;

    // Enable the RTC
    RCC->CSR |= RCC_CSR_RTCEN;

    // Enable CSS interrupt on LSE failure
    RCC->CIER |= RCC_CIER_CSSLSE;
    RCC->CSR |= RCC_CSR_LSECSSON;
    NVIC_EnableIRQ(RCC_IRQn);

    // Unlock the RTC registers (only required after RTC-domain reset, but harmless?)
    RTC->WPR = 0xca;
    RTC->WPR = 0x53;

    // Request initialization mode, and wait for it to happen
    RTC->ISR |= RTC_ISR_INIT;
    while (!(RTC->ISR & RTC_ISR_INITF))
    { }

    // These may persist through a reset, so (particularly for debugging) make
    // sure they start out disabled.
    ClearAlarm();
    ClearPeriodicWake();

    // Prescalers divide 32.768 kHz -> 256 Hz (asynchronously)
    // and then 256 Hz -> 1 Hz (synchronously)
    RTC->PRER = (127 << RTC_PRER_PREDIV_A_Pos) | (255 << RTC_PRER_PREDIV_S_Pos);

/*
    // Initial time (24-hour format is the default)
    RTC->TR = (2 << RTC_TR_HT_Pos)
            | (0 << RTC_TR_HU_Pos)
            | (0 << RTC_TR_MNT_Pos)
            | (0 << RTC_TR_MNU_Pos)
            | (0 << RTC_TR_ST_Pos)
            | (0 << RTC_TR_SU_Pos);
    // Initial date
    RTC->DR = (2 << RTC_DR_YT_Pos)
            | (2 << RTC_DR_YU_Pos)
            | (2 << RTC_DR_WDU_Pos)
            | (0 << RTC_DR_MT_Pos)
            | (3 << RTC_DR_MU_Pos)
            | (2 << RTC_DR_DT_Pos)
            | (2 << RTC_DR_DU_Pos);
*/

    // Exit init mode
    RTC->ISR &= (~RTC_ISR_INIT);

    // Enable RTC interrupt on EXTI rising edge (periodic wakeup or alarm)
    EXTI->RTSR |= ((1 << 17) | (1 << 20));
    EXTI->IMR |= ((1 << 17) | (1 << 20));  // TODO is EMR bit also needed for wakeup?
    NVIC_EnableIRQ(RTC_IRQn);
}



// Use the RTC alarm A to get an alarm interrupt when the clock matches the
// given hours/minutes/seconds.  If any of the parameters are negative, that
// field will be a wildcard (e.g. wake at a certain time in every hour).
void DRTC::SetAlarm(int hh, int mm, int ss, uint8_t frac256, bool interrupt)
{
    RTC->CR &= (~RTC_CR_ALRAE);

    uint8_t hbits = 0x80;
    if (hh >= 0)
    {
        hbits = ((hh / 10) << 4) | (hh % 10);
    }

    uint8_t mbits = 0x80;
    if (mm >= 0)
    {
        mbits = ((mm / 10) << 4) | (mm % 10);
    }

    uint8_t sbits = 0x80;
    if (ss >= 0)
    {
        sbits = ((ss / 10) << 4) | (ss % 10);
    }

    while (!(RTC->ISR & RTC_ISR_ALRAWF))
    { }

    // Set alarm with date mask enabled (so date doesn't matter)
    RTC->ALRMAR = RTC_ALRMAR_MSK4 | (hbits << 16) | (mbits << 8) | sbits;
    // Set subseconds match if frac256 != 0
    if (frac256 != 0)
    {
        RTC->ALRMASSR = (8 << RTC_ALRMASSR_MASKSS_Pos) | (uint32_t) frac256;
    }

    RTC->CR |= interrupt ? (RTC_CR_ALRAE | RTC_CR_ALRAIE) : RTC_CR_ALRAE;
}



// Stop the alarm function.
void DRTC::ClearAlarm()
{
    RTC->CR &= (~(RTC_CR_ALRAE | RTC_CR_ALRAIE));
}



// The RTC alarm interrupt fired.
void DRTC::RTCAlarm()
{
    // TODO ... hook into scheduler?
    __NOP();

    if (Debug)
    {
        unsigned s = RTC->TR & 0x7f;
        bool even = ((s & 0x01) == 0);
        GPIOA->BSRR = (even || (s == 1)) ? (1 << 22) : (1 << 6);
    }
}



// Use the RTC wake-up timer functionality to get a periodic wake-up interrupt.
// If hires is true, the intervaL is counted in seconds, for up to 18h.
// If hires is false, the interval is counted in 1/16384 s, for up to 4s.
void DRTC::SetPeriodicWake(uint16_t interval, bool hires, bool interrupt)
{
    // Request wake-up write access, and wait for it to happen
    RTC->CR &= (~RTC_CR_WUTE);
    while (!(RTC->ISR & RTC_ISR_WUTWF))
    { }

    // Choose either 1-Hz (hires=false) or RTCCLK/2 (hires=true) clock
    RTC->CR = (RTC->CR & ~RTC_CR_WUCKSEL) | ((hires ? 3 : 4) << RTC_CR_WUCKSEL_Pos);
    RTC->WUTR = interval - 1;
    RTC->CR |= interrupt ? (RTC_CR_WUTE | RTC_CR_WUTIE) : RTC_CR_WUTE;
}



// Stop the periodic wake-up timer.
void DRTC::ClearPeriodicWake()
{
    // Request wake-up write access, and wait for it to happen
    RTC->CR &= (~RTC_CR_WUTE);
    while (!(RTC->ISR & RTC_ISR_WUTWF))
    { }

    RTC->CR &= (~(RTC_CR_WUTE | RTC_CR_WUTIE));
}



// The RTC wake-up interrupt fired.  Use this to calibrate the MSI by counting
// cycles over one second.
void DRTC::RTCWake()
{
    uint16_t count = TIM2->CNT;

}



// Calibrate the MSI using the LSE (via the RTC periodic wake-up).  This takes
// a couple of seconds and returns the true frequency of the MSI to within a
// couple ppm, plus uncertainty in the LSE.
unsigned DRTC::RunMSICal()
{
    return 0;
}



// Called when the LSE "clock security system" interrupt fires
void DRTC::LSEFailure()
{
    LSEFailed = true;
}
