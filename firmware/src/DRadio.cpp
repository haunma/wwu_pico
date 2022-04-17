#include "DRadio.h"
#include "DGPIO.h"
#include "DI2C.h"
#include "DRTC.h"
#include "si5351.h"
#include "stm32l0xx.h"
#include <math.h>
#include <stdint.h>
#include <string.h>

// Master list of transmit frequencies
// NOTE: order must match exactly with the FreqName enum in DRadio.h!
const DRadio::FreqTable DRadio::freqs[] = {
    //  name          freq  odiv
    { CW_40M,      7052000,  113 },
    { CW_30M,     10122000,   79 },
    { CW_20M,     14052000,   56 },
    { CW_17M,     18072000,   44 },
    { CW_15M,     21052000,   38 },
    { CW_12M,     24902000,   32 },
    { CW_10M,     28052000,   28 },
    { WSPR_40M,    7040100,  113 },
    { WSPR_30M,   10140200,   78 },
    { WSPR_20M,   14097100,   56 },
    { WSPR_17M,   18106100,   44 },
    { WSPR_15M,   21096100,   37 },
    { WSPR_12M,   24926100,   32 },
    { WSPR_10M,   28126100,   28 },
};


// Frequency offset (in 256ths of a Hz) which is added to all target frequencies
// before programming the Si5351.  This hard-coded value is intended to compensate
// for 25-MHz crystal offset and (if properly characterized) temperature coef.
const static int CalOffset256 = 0;


// Morse code character definitions, read LSB first.  1 is carrier on and
// 0 is carrier off.  The final 1 is not transmitted; it is a marker for the
// inter-character space.
const static uint32_t MorseCode[] = {
    0b00000000000000000000000010000000,  // space
    0b00000000000000000000000100011101,  // a
    0b00000000000000000001000101010111,  // b
    0b00000000000000000100010111010111,  // c
    0b00000000000000000000010001010111,  // d
    0b00000000000000000000000000010001,  // e
    0b00000000000000000001000101110101,  // f
    0b00000000000000000001000101110111,  // g
    0b00000000000000000000010001010101,  // h
    0b00000000000000000000000001000101,  // i
    0b00000000000000010001110111011101,  // j
    0b00000000000000000001000111010111,  // k
    0b00000000000000000001000101011101,  // l
    0b00000000000000000000010001110111,  // m
    0b00000000000000000000000100010111,  // n
    0b00000000000000000100011101110111,  // o
    0b00000000000000000100010111011101,  // p
    0b00000000000000010001110101110111,  // q
    0b00000000000000000000010001011101,  // r
    0b00000000000000000000000100010101,  // s
    0b00000000000000000000000001000111,  // t
    0b00000000000000000000010001110101,  // u
    0b00000000000000000001000111010101,  // v
    0b00000000000000000001000111011101,  // w
    0b00000000000000000100011101010111,  // x
    0b00000000000000010001110111010111,  // y
    0b00000000000000000100010101110111,  // z
    0b00000000010001110111011101110111,  // 0
    0b00000000000100011101110111011101,  // 1
    0b00000000000001000111011101110101,  // 2
    0b00000000000000010001110111010101,  // 3
    0b00000000000000000100011101010101,  // 4
    0b00000000000000000001000101010101,  // 5
    0b00000000000000000100010101010111,  // 6
    0b00000000000000010001010101110111,  // 7
    0b00000000000001000101011101110111,  // 8
    0b00000000000100010111011101110111   // 9
};


// 8-bit indices for interleaving the convolutionally coded WSPR data
// (saves a bunch of algorithmic bit-reversing)
const static uint8_t InterleaveLUT[] = {
    0x00, 0x80, 0x40, 0x20, 0xa0, 0x60, 0x10, 0x90,
    0x50, 0x30, 0x70, 0x08, 0x88, 0x48, 0x28, 0x68,
    0x18, 0x98, 0x58, 0x38, 0x78, 0x04, 0x84, 0x44,
    0x24, 0x64, 0x14, 0x94, 0x54, 0x34, 0x74, 0x0c,
    0x8c, 0x4c, 0x2c, 0x6c, 0x1c, 0x9c, 0x5c, 0x3c,
    0x7c, 0x02, 0x82, 0x42, 0x22, 0x62, 0x12, 0x92,
    0x52, 0x32, 0x72, 0x0a, 0x8a, 0x4a, 0x2a, 0x6a,
    0x1a, 0x9a, 0x5a, 0x3a, 0x7a, 0x06, 0x86, 0x46,
    0x26, 0x66, 0x16, 0x96, 0x56, 0x36, 0x76, 0x0e,
    0x8e, 0x4e, 0x2e, 0x6e, 0x1e, 0x9e, 0x5e, 0x3e,
    0x7e, 0x01, 0x81, 0x41, 0x21, 0xa1, 0x61, 0x11,
    0x91, 0x51, 0x31, 0x71, 0x09, 0x89, 0x49, 0x29,
    0x69, 0x19, 0x99, 0x59, 0x39, 0x79, 0x05, 0x85,
    0x45, 0x25, 0x65, 0x15, 0x95, 0x55, 0x35, 0x75,
    0x0d, 0x8d, 0x4d, 0x2d, 0x6d, 0x1d, 0x9d, 0x5d,
    0x3d, 0x7d, 0x03, 0x83, 0x43, 0x23, 0x63, 0x13,
    0x93, 0x53, 0x33, 0x73, 0x0b, 0x8b, 0x4b, 0x2b,
    0x6b, 0x1b, 0x9b, 0x5b, 0x3b, 0x7b, 0x07, 0x87,
    0x47, 0x27, 0x67, 0x17, 0x97, 0x57, 0x37, 0x77,
    0x0f, 0x8f, 0x4f, 0x2f, 0x6f, 0x1f, 0x9f, 0x5f,
    0x3f, 0x7f
};


DRadio::DRadio() : State(IDLE), Count(0), Ptr(0), Code(0)
{
    // Initialize Si5351 driver structure with "safe" values
    Si5351_StructInit(&RadioConfig);

    RadioConfig.OSC.OSC_XTAL_Load = XTAL_Load_10_pF;
    RadioConfig.PLL[0].PLL_Clock_Source = PLL_Clock_Source_XTAL;
    RadioConfig.MS[0].MS_Clock_Source = MS_Clock_Source_PLLA;
    RadioConfig.CLK[0].CLK_R_Div = CLK_R_Div1;
    RadioConfig.CLK[0].CLK_Disable_State = CLK_Disable_State_LOW;
    RadioConfig.CLK[0].CLK_Enable = ON;
    RadioConfig.CLK[0].CLK_PowerDown = ON;
}


void DRadio::Init()
{
}



// Power on the Si5351 and its verify presence on the I2C bus.
bool DRadio::Enable()
{
    // Pull Q4 gate low to turn on the PMOS switch, then wait for Si5351 to power up.
    g_gpio.Clear(DGPIO::ENA_5351);

    // Give it some time to switch on and initialize.
    volatile unsigned count = 0;
    while (count++ < 50000)
    { }
    
    // Try reading a register.
    if (g_i2c.DetectTX())
    {
        return true;
    }

    // Oops--something's wrong :(
    g_gpio.Set(DGPIO::ENA_5351);
    return false;
}


// Power off the Si5351.
void DRadio::Disable()
{
    g_gpio.Set(DGPIO::ENA_5351);
}




// Start sending an ASCII message (lower-case letters only!) in morse code at
// the specified speed in words per minute.  The function is non-blocking, but
// the message buffer must remain valid until the message has been sent.
void DRadio::SendCW(char *msg, int len, int wpm)
{
    if (State == IDLE)
    {
        State = SENDING_CW;

        // Turn on the Si5351
        if (Enable()) // this involves a short delay
        {
            FindSynthParams(&RadioConfig, freqs[CW_20M].freq, 0, freqs[CW_20M].odiv);
            //Si5351_PLLConfig(&RadioConfig, PLL_A);
            Si5351_Init(&RadioConfig);
            Si5351_PLLReset(PLL_A);

            // Enable TIM2 to provide periodic interrupts every "dit" interval
            RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

            // Initialize state for ISR
            Count = len;
            Ptr = msg;
            Code = AsciiToMorse(*Ptr);

            // TIM2 configuration: "dit" clock
            // CR1: defaults (up-counter, counter disabled)
            TIM2->CR1 = 0;
            // PSC: prescale by 1000x is sufficient to obtain any reasonable CW speed;
            //      the "dit" rate in Hz is 50*WPM/60 so ARR = 12582/(5*WPM) - 1
            TIM2->PSC = 999;
            TIM2->ARR = 12582 / (5*wpm) - 1;
            // EGR: Generate an update event to force these values into the active registers
            TIM2->EGR = TIM_EGR_UG;
            // Enable periodic interrupts (on "update events") and unmask them in NVIC
            TIM2->DIER = TIM_DIER_UIE;
            NVIC_EnableIRQ(TIM2_IRQn);

            // Start the timer and call the first Tick() manually
            TIM2->CR1 |= TIM_CR1_CEN;
            //Tick();
        }
        else
        {
            State = IDLE;
            // TODO error handling
            __BKPT(0);
        }
    }
}



// Convert an ASCII character to a morse symbol from the const MorseCode array.
// Each character includes an inter-character space of three Farnsworth intervals
// at the end.  (An inter-word space gets its own morse symbol (index 0) with
// seven Farnsworth intervals.)
uint32_t DRadio::AsciiToMorse(char symbol)
{
    if (symbol >= 48 && symbol <= 57)
    {   // digit
        return MorseCode[symbol - 48 + 27];
    }
    else if (symbol >= 97 && symbol <= 122)
    {   // lower-case letter
        return MorseCode[symbol - 97 + 1];
    }
    else
    {    // space, or any disallowed character (like upper-case letters)
         return MorseCode[0];
    }
}



// This is called by a periodic timer interrupt once per "dit" interval to
// update the Si5351 carrier output.
void DRadio::Tick()
{
    switch (State)
    {
        case SENDING_CW:
        {
            if (Code == 1)
            {
                if (--Count > 0)
                {
                    Code = AsciiToMorse(*(++Ptr));
                }
                else
                {   // Done sending the message; disable TIM2
                    NVIC_DisableIRQ(TIM2_IRQn);
                    TIM2->CR1 = 0;
                    TIM2->DIER = 0;
                    RCC->APB1ENR &= (~RCC_APB1ENR_TIM2EN);
                    // Turn off the Si5351
                    Disable();
                    // Reset state
                    State = IDLE;
                    return;
                }
            }

            // Set the CLK0_PDN bit directly instead of going through the driver,
            // since we don't need to set a bunch of other clock power params and
            // we don't need a read-modify-write.  This assumes we already know
            // the other bits in the register (MS clock source, MS integer mode,
            // CLK drive strength and source, etc.)!
            if (Code & 1)
            {
                // turn carrier on
                //g_gpio.Clear(DGPIO::LED);
                if (g_i2c.Write(DI2C::ADDR_SI5351, 16, 0x4f) < 0)
                {
                    __BKPT(0);
                }
            }
            else
            {
                // turn carrier off
                //g_gpio.Set(DGPIO::LED);
                if (g_i2c.Write(DI2C::ADDR_SI5351, 16, 0xcf) < 0)
                {
                    __BKPT(0);
                }
            }

            Code >>= 1;

            break;
        }

        case SENDING_WSPR:
        {
            if (++Count >= WSPR_SYMBOL_COUNT)
            {
                // Stop the RTC wake-up interrupts
                g_rtc.ClearPeriodicWake();
                // Disable the RF output
                //g_i2c.Write(DI2C::ADDR_SI5351, 3, 1);
                g_i2c.Write(DI2C::ADDR_SI5351, 16, 0xcf);
                // Turn off the Si5351
                Disable();
                // Reset state
                State = IDLE;
            }
            else
            {   // Transmit the next symbol
                FindSynthParams(&RadioConfig, freqs[WSPR_20M].freq, 375*WSPRSymbols[Count], freqs[WSPR_20M].odiv);
                Si5351_PLLConfig(&RadioConfig, PLL_A);
            }

            break;
        }
    }
}



// Start sending a WSPR transmission.
void DRadio::SendWSPR()
{
    if (State == IDLE)
    {
        State = SENDING_WSPR;
        Count = 0;

        // Turn on the Si5351
        if (Enable()) // this involves a short delay
        {
            // Create the vector of 162 4-FSK symbols to transmit
            // (TODO encode telemetry and add args to SendWSPR!)
            WSPREncode(WSPRSymbols, WSPR_Type1);

            // Tune to the first symbol frequency and then reset the PLL (no resets
            // needed after this).
            // TODO: is there merit in a random +/- 100 Hz shift like Zachtek does?
            FindSynthParams(&RadioConfig, freqs[WSPR_20M].freq, 375*WSPRSymbols[0], freqs[WSPR_20M].odiv);
            //Si5351_PLLConfig(&RadioConfig, PLL_A);
            Si5351_Init(&RadioConfig);
            Si5351_PLLReset(PLL_A);

            // Enable the RF output
            //if (g_i2c.Write(DI2C::ADDR_SI5351, 3, 0) < 0)
            if (g_i2c.Write(DI2C::ADDR_SI5351, 16, 0x4f) < 0)
            {
                __BKPT(0);
            }

            // Use the RTC wake-up counter for the WSPR symbol period, to avoid having
            // to calibrate the MSI.  The correct period is 8192/12000 = 0.682667s but
            // we are forced to approximate that by 11185/16384 = 0.682678s, so we will
            // run slow by ~ 12 us per symbol.
            g_rtc.SetPeriodicWake(11185, true, true);
        }
        else
        {
            State = IDLE;
            // TODO error handling
            __BKPT(0);
        }
    }
}



// six-character callsign: KJ6PC_
// callsign values:  20, 19, 6, 25, 12, 36
// 28-bit callsign number:
//    (((((((((20 * 36) + 19) * 10) + 6) * 27) + (25-10)) * 27) + (12-10)) * 27) + (36-10) = 145586483
//
// four-character locator for WWU is DN06:
// locator values:  3, 13, 0, 6
// power estimate: 12 dBm
// 22-bit locator/power number:
//   (((179 - 10*3 - 0)*180 + 10*13 + 6) * 128) + 12 + 64 = 3450444
//
// overall 50-bit payload = (145586483 << 22) + 3450444 = 0x22b5e4cf4a64c

// Adapted from https://github.com/HarrydeBug/1011-WSPR-TX_LP1 "wspr_encode"
void DRadio::WSPREncode(/* TBD data to encode */ uint8_t *symbols, WSPRType type)
{
    uint32_t n, m;    // 28-bit (callsign) and 22-bit (loc+power) components of the 50-bit message

    switch (type) {
        case WSPR_Type1:  // Normal coding with callsign, four-char locator, and power
            // Pre-computed KJ6PC  DN06  12 dBm
            n = 145586483;
            m = 3450444;
            break;

        case WSPR_Type2:  // Type 2 messages are not of interest and thus not supported
            n = 0;
            m = 0;
            break;

        case WSPR_Type3:  // Hashed callsign, six-char locator, and power
            //encode the six letter Maidenhear postion in to n that is usually used for callsign coding, reshuffle the character order to conform to the callsign rules
            //n = WSPRCode(GadgetData.WSPRData.MaidenHead6[1]);
            //n = n * 36 + WSPRCode(GadgetData.WSPRData.MaidenHead6[2]);
            //n = n * 10 + WSPRCode(GadgetData.WSPRData.MaidenHead6[3]);
            //n = n * 27 + (WSPRCode(GadgetData.WSPRData.MaidenHead6[4]) - 10);
            //n = n * 27 + (WSPRCode(GadgetData.WSPRData.MaidenHead6[5]) - 10);
            //n = n * 27 + (WSPRCode(GadgetData.WSPRData.MaidenHead6[0]) - 10);
            //m = 128 * WSPRCallHash(call) - power - 1 + 64;
            n = 0;  //TBD
            m = 128 * WSPRCallHash("KJ6PC") - 12 - 1 + 64;
            break;
    }

    // Pack callsign (28 bits) and locator/power (22 bits) into work array.
    uint8_t c[11];

    // A little less work to start with the least-significant bits
    c[3] = (uint8_t)((n & 0x0f) << 4);
    n = n >> 4;
    c[2] = (uint8_t)(n & 0xff);
    n = n >> 8;
    c[1] = (uint8_t)(n & 0xff);
    n = n >> 8;
    c[0] = (uint8_t)(n & 0xff);

    c[6] = (uint8_t)((m & 0x03) << 6);
    m = m >> 2;
    c[5] = (uint8_t)(m & 0xff);
    m = m >> 8;
    c[4] = (uint8_t)(m & 0xff);
    m = m >> 8;
    c[3] |= (uint8_t)(m & 0x0f);
    c[7] = 0;
    c[8] = 0;
    c[9] = 0;
    c[10] = 0;

    // Convolutional code
    uint8_t s[WSPR_SYMBOL_COUNT];
    WSPRConvolve(c, s, 11, WSPR_SYMBOL_COUNT);

    // Interleave
    uint8_t d[WSPR_SYMBOL_COUNT];
    for (uint8_t j = 0; j < WSPR_SYMBOL_COUNT; j++)
    {
        d[InterleaveLUT[j]] = s[j];
    }
    memcpy(s, d, WSPR_SYMBOL_COUNT);

    // Merge with sync vector
    WSPRMergeSyncVector(s, symbols);
}



// From https://github.com/HarrydeBug/1011-WSPR-TX_LP1 "convolve"
void DRadio::WSPRConvolve(uint8_t *c, uint8_t *s, int message_size, int bit_size)
{
    uint32_t reg_0 = 0;
    uint32_t reg_1 = 0;
    uint32_t reg_temp;
    int bit_count = 0;

    for (int i = 0; i < message_size; i++)
    {
        for (int j = 0; j < 8; j++)
        {
            // Set input bit according the MSB of current element
            uint32_t input_bit = (((c[i] << j) & 0x80) == 0x80) ? 1 : 0;

            // Shift both registers and put in the new input bit
            reg_0 <<= 1;
            reg_1 <<= 1;
            reg_0 |= input_bit;
            reg_1 |= input_bit;

            // AND Register 0 with feedback taps, calculate parity
            reg_temp = reg_0 & 0xf2d05351;
            uint8_t parity_bit = 0;
            for (int k = 0; k < 32; k++)
            {
                parity_bit = parity_bit ^ (reg_temp & 0x01);
                reg_temp >>= 1;
            }
            s[bit_count] = parity_bit;
            bit_count++;

            // AND Register 1 with feedback taps, calculate parity
            reg_temp = reg_1 & 0xe4613c47;
            parity_bit = 0;
            for (int k = 0; k < 32; k++)
            {
                parity_bit = parity_bit ^ (reg_temp & 0x01);
                reg_temp = reg_temp >> 1;
            }
            s[bit_count] = parity_bit;
            bit_count++;
            if (bit_count >= bit_size)
            {
                break;
            }
        }
    }
}


// From https://github.com/HarrydeBug/1011-WSPR-TX_LP1 "wspr_merge_sync_vector"
void DRadio::WSPRMergeSyncVector(uint8_t *g, uint8_t *symbols)
{
    const uint8_t sync_vector[WSPR_SYMBOL_COUNT] =
    { 1, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0, 1, 0, 0,
      1, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 1, 0, 0,
      0, 0, 0, 0, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 1, 1, 0, 1,
      0, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0, 1, 0, 0, 1, 0,
      1, 1, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1,
      0, 0, 1, 0, 0, 1, 1, 1, 0, 1, 1, 0, 0, 1, 1, 0, 1, 0, 0, 0, 1,
      1, 1, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0,
      1, 1, 0, 1, 0, 1, 1, 0, 0, 0, 1, 1, 0, 0, 0
    };

    for (int i = 0; i < WSPR_SYMBOL_COUNT; i++)
    {
        symbols[i] = sync_vector[i] + (2 * g[i]);
    }
}


// Callsign hash for WSPR type-3 messages.
// (by RFZero www.rfzero.net modified by SM7PNV and simplified by KJ6PC)
// "call" should be just the callsign without padding.
uint32_t DRadio::WSPRCallHash(const char *call)
{
#define rot(x, k) ((x << k) | (x >> (32 - k)))

    // Assume no callsign suffix or prefix (simplified code -- KJ6PC)
    //uint8_t Length = strlen(call);

    uint32_t a, b, c;
    a = b = c = 0xdeadbeef + 5 /*Length*/ + 146;

    const uint32_t *k = (const uint32_t *) call;

//    switch (Length)   // Length 3-10 chars, thus 0, 1, 2, 11 and 12 omitted
//    {
//      case 10: c += k[2] & 0xffff;   b += k[1]; a += k[0]; break;
//      case 9:  c += k[2] & 0xff;     b += k[1]; a += k[0]; break;
//      case 8:  b += k[1];            a += k[0];            break;
//      case 7:  b += k[1] & 0xffffff; a += k[0];            break;
//      case 6:  b += k[1] & 0xffff;   a += k[0];            break;
//      case 5:  b += k[1] & 0xff;     a += k[0];            break;
//      case 4:  a += k[0];                                  break;
//      case 3:  a += k[0] & 0xffffff;                       break;
//    }
    b += k[1] & 0xff;     a += k[0];

    c ^= b; c -= rot(b, 14);
    a ^= c; a -= rot(c, 11);
    b ^= a; b -= rot(a, 25);
    c ^= b; c -= rot(b, 16);
    a ^= c; a -= rot(c, 4);
    b ^= a; b -= rot(a, 14);
    c ^= b; c -= rot(b, 24);

    return (c & 0xffff);  // 15-bit mask
}


/*
// From https://github.com/HarrydeBug/1011-WSPR-TX_LP1 "wspr_code"
uint8_t WSPRCode(char c)
{
  // Validate the input then return the proper integer code.
  // Return 255 as an error code if the char is not allowed.
  if (isdigit(c))
  {
    return (uint8_t)(c - 48);
  }
  else if (c == ' ')
  {
    return 36;
  }
  else if (c >= 'A' && c <= 'Z')
  {
    return (uint8_t)(c - 55);
  }
  else
  {
    return 255;
  }
}
*/

// Find Si5351 synthesizer parameters for the given frequency, which is split
// into integer hertz and fractional-hertz.  We assume the frequency is above
// 1 MHz.  Note: frac256 is allowed to exceed 256.
//
// Returns true if a PLL reset is needed, false otherwise.
bool DRadio::FindSynthParams(Si5351_ConfigTypeDef *Params, unsigned hz, unsigned frac256, unsigned odiv)
{
    static unsigned OldHz = 0;  // previously set frequency, used to track PLL reset requirement

    // Target PLL frequency is near 800 MHz; the integer output divider should
    // be chosen to achieve that (see frequency table).  Result is represented
    // in 32.8 fixed-point format.
    int64_t pllFreq = odiv * ( ((int64_t) hz << 8) + frac256 + CalOffset256 );

    // Find the fractional PLL parameters a + b/c where a is an integer in
    // [15,90), c is fixed at 2^20 - 1, and b is an integer from 0 to 2^20 - 1.
    //
    // First, the integer part.
    uint32_t a = 0;
    while (pllFreq > (25000000ULL * 256ULL))
    {
        a++;
        pllFreq -= (25000000ULL * 256ULL);
    }

    // Get the numerator b without floating-point math by using most of the
    // 64-bit pllFreq (lower 60 bits, actually), multiplying by a magic
    // constant such that right-shifting yields the properly rounded result.
    // The magic constant is round((2^20 - 1 + 0.5) * (2^40) / (25e6 * 256)).
    uint32_t b = (180143899u * pllFreq) >> 40;
 
    Params->PLL[0].PLL_Multiplier_Integer = a;
    Params->PLL[0].PLL_Multiplier_Numerator = b;
    Params->PLL[0].PLL_Multiplier_Denominator = 1048575;  // c
    Params->MS[0].MS_Divider_Integer = odiv;

    int freqChange = hz - OldHz;
    OldHz = hz;
    if (abs(freqChange) > 100000)
    {
        return true;
    }

    return false;
}
