#include "DI2C.h"
#include "stm32l0xx.h"


DI2C::DI2C()
{
}


void DI2C::Init()
{
    // Clock enables: I2C1 and GPIOA
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;

    // Reset I2C1, which was shut down during sleep
    RCC->APB1RSTR = RCC_APB1RSTR_I2C1RST;
    __NOP();
    __NOP();
    __NOP();
    RCC->APB1RSTR = 0;

    // Timing from STM32CubeMX, assuming I2C clock is MSI @ 2.097 MHz (no prescaling):
    //   300 kHz (FM)  0x00000002 (tSCLL 3: 1.43us  tSCLH 1: 0.48us  tSDADEL 0  tSCLDEL 1)
    //   200 kHz (FM)  0x00000004 (tSCLL 5: 2.38us  tSCLH 1: 0.48us  tSDADEL 0  tSCLDEL 1)
    //   100 kHz       0x00000608 (tSCLL 9: 4.29us  tSCLH 7: 3.34us  tSDADEL 0  tSCLDEL 1)
    //    20 kHz       0x0000065c (tSCLL 93:44.3us  tSCLH 7: 3.34us  tSDADEL 0  tSCLDEL 1)
    I2C1->TIMINGR = 0x002;

    // I2C1_CR1: PE=1
    I2C1->CR1 = I2C_CR1_PE;

    // Pinmux: PA9=SCL and PA10=SDA (alt func #1), with internal pullups
    GPIOA->OTYPER = (GPIOA->OTYPER & 0x0000f9ff) | (1 << 9) | (1 << 10);      // set open-drain
    GPIOA->PUPDR  = (GPIOA->PUPDR  & 0xffc3ffff) | (1 << 2*9) | (1 << 2*10);  // enable pull-ups
    GPIOA->AFR[1] = (GPIOA->AFR[1] & 0xfffff00f) | (1 << 4*(9-8)) | (1 << 4*(10-8)); // set AF1
    GPIOA->MODER  = (GPIOA->MODER  & 0xffc3ffff) | (2 << 2*9) | (2 << 2*10);  // alt-func mode
}



void DI2C::Deinit()
{
    // Before sleeping, disable the pull-ups and set the I2C lines low so they won't
    // source current to the switched-off Si5351.
    GPIOA->ODR    = (GPIOA->ODR    & 0x0000f9ff);   // set low
    GPIOA->MODER  = (GPIOA->MODER  & 0xffc3ffff)  | (1 << 2*9) | (1 << 2*10);  // output mode
    GPIOA->OTYPER = (GPIOA->OTYPER & 0x0000f9ff);   // not open-drain
    GPIOA->PUPDR  = (GPIOA->PUPDR  & 0xffc3ffff);   // push-pull

    // Disable the I2C peripheral
    I2C1->CR1 = 0;
    RCC->APB1ENR &= (~RCC_APB1ENR_I2C1EN);
}



int DI2C::Write(enum DI2C::Device dev, uint8_t reg, uint8_t data)
{
    int timeout;

    // I2C1_CR2: AUTOEND=0, NBYTES=1, STOP=0, START=0 (set later), RD_WRN=0 (write xfer), SADD=0x5c
    I2C1->CR2 = (2 << I2C_CR2_NBYTES_Pos) | (dev << 1);

    // Wait for TX empty
    timeout = TIMEOUT_COUNTS;
    while (!(I2C1->ISR & I2C_ISR_TXE))
    {
        if (--timeout == 0)
        {
            return -1;
        }
    }

    // Send register byte
    I2C1->TXDR = reg;
    I2C1->CR2 |= I2C_CR2_START;

    // Wait until we can send more data
    timeout = TIMEOUT_COUNTS;
    while (!(I2C1->ISR & I2C_ISR_TXIS))
    {
        if (--timeout == 0)
        {
            return -1;
        }
    }

    // Send data byte
    I2C1->TXDR = data;

    // Wait for TX complete
    timeout = TIMEOUT_COUNTS;
    while (!(I2C1->ISR & I2C_ISR_TC))
    {
        if (--timeout == 0)
        {
            return -1;
        }
    }

    return 0;
}



uint8_t DI2C::Read(enum DI2C::Device dev, uint8_t reg)
{
    int timeout;

    // I2C1_CR2: AUTOEND=0, NBYTES=1, STOP=0, START=0 (set later), RD_WRN=0 (write xfer), SADD=0x5c
    I2C1->CR2 = (1 << I2C_CR2_NBYTES_Pos) | (dev << 1);

    // Wait for TX empty
    timeout = TIMEOUT_COUNTS;
    while (!(I2C1->ISR & I2C_ISR_TXE))
    {
        if (--timeout == 0)
        {
            return -1;
        }
    }

    // Send register byte
    I2C1->TXDR = reg;
    I2C1->CR2 |= I2C_CR2_START;

    // Wait for TX complete
    timeout = TIMEOUT_COUNTS;
    while (!(I2C1->ISR & I2C_ISR_TC))
    {
        if (--timeout == 0)
        {
            return -1;
        }
    }

    I2C1->CR2 = I2C_CR2_RD_WRN | I2C_CR2_AUTOEND | (1 << I2C_CR2_NBYTES_Pos) | (dev << 1);
    I2C1->CR2 |= I2C_CR2_START;
    
    // Wait for RX data
    timeout = TIMEOUT_COUNTS;
    while (!(I2C1->ISR & I2C_ISR_RXNE))
    {
        if (--timeout == 0)
        {
            return -1;
        }
    }

    return I2C1->RXDR;
}



// Check that the LPS225HB pressure sensor is accessible and responding.
bool DI2C::DetectPressure()
{
    return (Read(ADDR_LPS225HB, 0x0f) == 0xb1);  // ID register
}


// Check that the Si5351 clock chip is accessible and responding. There is no
// official ID register, so this may not be valid for other chip revisions...
bool DI2C::DetectTX()
{   
    return (Read(ADDR_SI5351, 0x00) == 0x11);
}
