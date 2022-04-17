#include "DADC.h"
#include "DGPIO.h"
#include "DI2C.h"
#include "DPower.h"
#include "DRadio.h"
#include "DRTC.h"

#include "stm32l0xx.h"
#include <stdio.h>
#include <stdlib.h>

void LED(bool on);
void SystemInit();

// Are we in debug (ground testing) mode?
bool Debug = false;
// Standard CMSIS clock-freq global
uint32_t SystemCoreClock = 2097000u;


// Driver and module instantiation
DADC   g_adc;
DGPIO  g_gpio;
DI2C   g_i2c;
DPower g_power;
DRadio g_radio;
DRTC   g_rtc;


void main()
{
    // Set PA6 to an input with weak pull-down.  If it remains high (in IDR)
    // then the breakaway debug tab is still connected: set the debug flag.
    // If it reads low, then the tab has been removed: don't do debug stuff.
    // (Depending on the LED characteristics at low current, this is not
    // guaranteed to work, but it seems ok in the lab which is good enough...)
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
    GPIOA->MODER = (GPIOA->MODER & 0xffffcfff);                   // input mode
    GPIOA->PUPDR = (GPIOA->PUPDR & 0xffffcfff) | (2 << 2*6);      // enable pull-down
    if (GPIOA->IDR & 0x40)
    {
        Debug = true;
        LED(false);
        GPIOA->PUPDR = (GPIOA->PUPDR & 0xffffcfff);               // disable pull-up/down
        GPIOA->MODER = (GPIOA->MODER & 0xffffcfff) | (1 << 2*6);  // output mode

        // Enable clock trim control on PC15 (pulled low => clock runs 500 ppm fast)
        RCC->IOPENR |= RCC_IOPENR_GPIOCEN;
        GPIOC->MODER = (GPIOC->MODER & 0x3fffffff);               // input mode
        GPIOC->PUPDR = (GPIOC->PUPDR & 0x3fffffff) | (1 << 2*15); // enable pullup
    }
    else
    {
        Debug = false;
        GPIOA->PUPDR = (GPIOA->PUPDR  & 0xffffcfff);              // disable pull-up/down
        GPIOA->MODER = (GPIOA->MODER  & 0xffffcfff) | (3 << 2*6); // analog mode
    }


    // Initialize drivers
    g_gpio.Init();
    g_power.Init();
    g_rtc.Init();
    g_adc.Init();
    g_i2c.Init();
    g_radio.Init();

    //g_radio.SendWSPR();

    volatile unsigned count;

    if (Debug)
    {
        // Blink LED at 0.5 Hz to allow trimming of the RTC time
        g_rtc.SetAlarm(-1, -1, -1, 0, true);

        uint32_t trimPinState = GPIOC->IDR & 0x8000;
        while(1)
        {
            count = 0;
            while (++count < 50000)
            { }
            
            uint32_t tmp = GPIOC->IDR & 0x8000;
            if (tmp != trimPinState)
            {
                trimPinState = tmp;
                if (trimPinState == 0)
                {   // PC15 pulled low: set the RTC to run fast
                    while (RTC->ISR & RTC_ISR_RECALPF)
                    { }   // wait until safe to write CALR register
                    RTC->CALR = RTC_CALR_CALP;
                }
                else
                {  // PC15 floating high: set the RTC to normal
                    while (RTC->ISR & RTC_ISR_RECALPF)
                    { }   // wait until safe to write CALR register
                    RTC->CALR = 0;
                }
            }
        }
    }


    //g_rtc.SetPeriodicWake(30, false, false);

    while (1)
    {
        g_i2c.Deinit();
        PWR->CR |= PWR_CR_CWUF;
        while (PWR->CSR & PWR_CSR_WUF)
        { }
        g_power.Stop();
    }



    while (1)
    {
        LED(true);

        count = 0;
        while (++count < 50000)
        { }

        LED(false);

        count = 0;
        while (++count < 50000)
        { }
    }

/*
    volatile unsigned count;
    volatile bool flag = true;
    while (1)
    {
        g_i2c.Init();
        //if (!g_radio.Enable())
        //{
        //    flag = false;
        //}

        if (flag) LED(true);

        g_radio.SendCW("cq cq de kj6pc", 16, 20);

        count = 0;
        while (++count < 2400000)
        { }

        if (flag) LED(false);

        count = 0;
        while (++count < 2400000)
        { }

        //g_radio.Disable();
        g_i2c.Deinit();

        PWR->CR |= PWR_CR_CWUF;
        while (PWR->CSR & PWR_CSR_WUF)
        { }
        g_power.Stop();
    }
*/

/*
    float i_h, i_va, i_vb, vdd;
    unsigned limit;
    while (1)
    {
        g_radio.SendCW("cq cq de kj6pc", 14, 20);

        limit = (g_power.VDDGood() ? 25000 : 5000) + (g_rtc.LSEGood() ? 25000 : 5000);
        count = 0;
        while (count++ < limit)
        { }
        LED(true);

        count = 0;
        while (count++ < limit)
        { }
        LED(false);

        count = 0;
        while (count++ < 2000000)
        { }

        //g_adc.Read(&i_h, &i_va, &i_vb, &vdd);
    }
*/
}


// Control the debug LED.  This is done here instead of DGPIO because in a flight
// configuration the driving pin should be left in its reset default (analog mode
// to save power).
void LED(bool on)
{
    if (Debug)
    {
        GPIOA->BSRR = on ? (1 << 22) : (1 << 6);  // PA6 low to turn on the LED
    }
}


// Dummy function to make startup happy; in STM32L031 there is nothing to do
// here if we are happy with the default ~ 2.1 MHz MSI clock.
void SystemInit()
{
}
