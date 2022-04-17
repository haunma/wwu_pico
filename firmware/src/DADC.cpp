#include "DADC.h"
#include "DGPIO.h"
#include "stm32l0xx.h"

#include <stdint.h>


DADC::DADC()
//    : BufferFilling(Ping), SineAmp(InitialSineAmp), PhaseAcc(0), PhaseInc(InitialPhaseInc),
//      ILO(InitialSineAmp), QLO(0.0f)
{
}


void DADC::Init()
{
    // Clock enable
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADCEN;

    // Pinmux: PA1, PA2, PA3 are analog inputs
    GPIOA->MODER = (GPIOA->MODER & 0xffffff03) | (3 << 1*2) | (3 << 2*2) | (3 << 3*2);

    // ADC init:
    // CCR: LFMEN=1 ("low-freq" mode for ADC clock < 3.5 MHz)
    ADC->CCR = ADC_CCR_LFMEN | ADC_CCR_VREFEN;
    // CFGR1: defaults
    // CFGR2: CKMODE=3 (clocked by PCLK with no prescaling), OVSS=4 (right-shift 4 bits),
    //        OVSR=7 (oversample 256x), OVSE=1 (oversampling enabled)
    ADC1->CFGR2 = (3 << ADC_CFGR2_CKMODE_Pos) | (4 << ADC_CFGR2_OVSS_Pos)
                | (7 << ADC_CFGR2_OVSR_Pos) | ADC_CFGR2_OVSE;
    // SMPR: SMP=6 (next-to-longest sampling time, 79.5 ADC clocks)
    ADC1->SMPR = 6;

    // Perform initial ADC calibration
    Wake();
}


void DADC::Sleep()
{
    ADC1->CR &= (~ADC_CR_ADEN);
    ADC1->CR &= (~ADC_CR_ADVREGEN);
}


void DADC::Wake()
{
    // Run calibration; this takes ~ 50 us and also sets ADVREGEN=1
    ADC1->CR |= ADC_CR_ADCAL;
    while (ADC1->CR & ADC_CR_ADCAL)
    { }
}


void DADC::Read(float *i_h, float *i_va, float *i_vb, float *vdd)
{
    // Enable the ADC
    ADC1->ISR |= ADC_ISR_ADRDY;
    ADC1->CR |= ADC_CR_ADEN;
    while (!(ADC1->ISR & ADC_ISR_ADRDY))
    { }

    // Get calibration data stored in NVM
    float vref_cal = (float) *((uint16_t *) 0x1ff80078);

    // Turn on the shunt MOSFETs (power still provided by supercap)
    g_gpio.Set(DGPIO::SHUNT_HORIZ);
    g_gpio.Set(DGPIO::SHUNT_VERTA);
    g_gpio.Set(DGPIO::SHUNT_VERTB);

    // Read Vref_int and calculate VDDA (= VDD)
    ADC1->CHSELR = 1 << 17;
    ADC1->CR |= ADC_CR_ADSTART;
    while (!(ADC1->ISR & ADC_ISR_EOC))
    { }
    // Calculate VDDA (=VDD) based on the stored vref_int data, with a small
    // compensation (-2000 ppm) for low temperature.  (This could be further
    // characterized, then made more accurate using an actual temperature
    // measurement.)
    *vdd = 47.904f * vref_cal / (float) ADC1->DR;  // 2.994V * 16 = 47.904

    // Read horizontal cell shunt voltage and convert to current
    ADC1->CHSELR = 1 << 1;
    ADC1->CR |= ADC_CR_ADSTART;
    while (!(ADC1->ISR & ADC_ISR_EOC))
    { }
    // TODO 4096 or 4095?
    *i_h = 3.91251e-7f * (*vdd) * (float) ADC1->DR;  // 1 / 4096 / 39 ohms / 16

    // Read vertical cell A shunt voltage and convert to current
    ADC1->CHSELR = 1 << 2;
    ADC1->CR |= ADC_CR_ADSTART;
    while (!(ADC1->ISR & ADC_ISR_EOC))
    { }
    *i_va = 3.91251e-7f * (*vdd) * (float) ADC1->DR;  // 1 / 4096 / 39 ohms

    // Read vertical cell B shunt voltage and convert to current
    ADC1->CHSELR = 1 << 3;
    ADC1->CR |= ADC_CR_ADSTART;
    while (!(ADC1->ISR & ADC_ISR_EOC))
    { }
    *i_vb = 3.91251e-7f * (*vdd) * (float) ADC1->DR;  // 1 / 4096 / 39 ohms

    // Turn on the shunt MOSFETs (power still provided by supercap)
    g_gpio.Clear(DGPIO::SHUNT_HORIZ);
    g_gpio.Clear(DGPIO::SHUNT_VERTA);
    g_gpio.Clear(DGPIO::SHUNT_VERTB);

    // Disable the ADC
    ADC1->CR |= ADC_CR_ADDIS;
}
