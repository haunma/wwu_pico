#include "DPower.h"
#include "DADC.h"
#include "stm32l0xx.h"

__ramfunc static void ProgramOptionBOR();


DPower::DPower() :
    OverVoltage(false)
{
}


void DPower::Init()
{
    // PWR and SYSCFG clock enables
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;

    // Enable VREFINT for ADC
    //SYSCFG->CFGR3 = SYSCFG_CFGR3_ENBUF_VREFINT_ADC;

    // TODO disable SYSCFG clock here?

    // Set SLEEPDEEP in the Cortex-M0 system control register to choose Stop
    // mode when WFI instruction is encountered
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    // PWR_CR: Enable RTC access; put internal vreg in low-power and turn off
    // vrefint during stop mode; set voltage range 3 (1.2V low performance);
    // enable and choose "level 4" (2.7V) for the prog. voltage detector (PVD)
    PWR->CR = PWR_CR_LPSDSR | PWR_CR_ULP | (3 << PWR_CR_VOS_Pos)
            | PWR_CR_CWUF | PWR_CR_DBP | PWR_CR_PLS_LEV4 | PWR_CR_PVDE;
    // TODO... setting ULP will disable PVD overvoltage detection during sleep.
    // Is this OK?  Should be if we wake up periodically?

    // Check option-byte BOR setting.  If this is the first time we have booted
    // this particular part (or the option bytes got reset somehow), fix this
    // by writing the second option byte to set BOR level 1 (1.8V).
    if ((FLASH->OPTR & FLASH_OPTR_BOR_LEV) != 0x00080000)
    {
        // This will end with a special "option-byte-reloading" reset
        // and never return.
        ProgramOptionBOR();
    }

// TODO forced to disable this due to spurious PVD interrupts when the Si5351 load switch toggles :(
/*
    // Enable PVD interrupt on EXTI16 falling edge (VDD exceeds safe value; note
    // backwards sense of the PVDO flag which requires falling-edge sensitivity)
    EXTI->FTSR |= (1 << 16);
    //EXTI->RTSR |= (1 << 16); // if we want an interrupt when VDD falls below 2.7V too
    //EXTI->EMR |= (1 << 16): // TODO needed for wakeup?
    EXTI->IMR |= (1 << 16);
    NVIC_EnableIRQ(PVD_IRQn);
*/
}



// Enter stop mode.  In this mode all clocks are stopped (except the external
// 32.768 kHz) but Vcore is still powered and RAM/state are retained.  Supply
// current is well under 1 uA.
//
// Current budget in stop mode:
//   MCU       1 uA
//   SiT1566   5 uA (always on)
//   LPS225HB  1 uA (in power-down mode)
//   Si5351A   0 uA (load switch off)
//
void DPower::Stop()
{
    // VREFint must be powered off before entering stop, to achieve < 1 uA.
    // This happens automatically if PWR_CR_ULP is set.

    // The internal voltage regulator must also be put in low-power mode.
    // This happens automatically (?) depending on PWR_CR_LPSDSR and PWR_CR_LPDS.

    // The ADC should be disabled by setting ADC_CR_ADDIS.
    //g_adc.Sleep();

    // Set RCC_CFGR_STOPWUCK to use MSI upon wakeup.  (This is the default.)
    // See ref man p154 and p159 (RTC auto-wake from stop mode).
    __WFI();
}



// Called when the PVD interrupt fires from VDD rising above 2.7V
void DPower::HighVDD()
{
    OverVoltage = true;
}


// Program the option bytes (mainly for setting a low brown-out reset threshold).
#define FLASH_PEKEY1               (0x89ABCDEFu)
#define FLASH_PEKEY2               (0x02030405u)
#define FLASH_OPTKEY1              (0xFBEAD9C8u)
#define FLASH_OPTKEY2              (0x24252627u)
__ramfunc static void ProgramOptionBOR()
{
    // Wait till no operation is ongoing
    while (FLASH->SR & FLASH_SR_BSY)
    { }

    // Check that the PELOCK is unlocked
    if (FLASH->PECR & FLASH_PECR_PELOCK)
    {
        // Perform unlock sequence
        FLASH->PEKEYR = FLASH_PEKEY1;
        FLASH->PEKEYR = FLASH_PEKEY2;
    }

    // Check if the OPTLOCK is unlocked
    if (FLASH->PECR & FLASH_PECR_OPTLOCK)
    {
        // Perform unlock sequence
        FLASH->OPTKEYR = FLASH_OPTKEY1;
        FLASH->OPTKEYR = FLASH_OPTKEY2;
    }

    // Write a 32-bit word value at the second option byte address;
    // the 16-bit data is extended with its compement.
    //
    // The factory value was 0x7f8f8070.  This is changed to 0x7f878078,
    // which sets BOR_LEV=1000 for BOR Level 1 (around 1.8V), for more
    // robust reset when the supercap voltage is low.
    *(__IO uint32_t *)(0x1ff80004) = 0x7f878078;
    while (FLASH->SR & FLASH_SR_BSY)
    { }

    if (FLASH->SR & FLASH_SR_EOP)
    {
        FLASH->SR = FLASH_SR_EOP;
    }
    else
    {
        __BKPT(0);
    }

    // Reload option bytes and reset (this will re-lock also)
    FLASH->PECR |= FLASH_PECR_OBL_LAUNCH;
}
