#pragma once

#include "si5351.h"
#include <stdint.h>

class DRadio
{
public:
    // Beacon frequencies
    enum FreqName
    {
        CW_40M = 0,
        CW_30M,
        CW_20M,
        CW_17M,
        CW_15M,
        CW_12M,
        CW_10M,
        WSPR_40M,
        WSPR_30M,
        WSPR_20M,
        WSPR_17M,
        WSPR_15M,
        WSPR_12M,
        WSPR_10M,
        NUM_FREQS,
    };

    struct FreqTable
    {
        enum FreqName name;
        unsigned freq;
        unsigned odiv;
    };

    static const FreqTable freqs[];

    // WSPR message types
    enum WSPRType
    {
        WSPR_Type1 = 0,
        WSPR_Type2,
        WSPR_Type3,
        NUM_WSPR_TYPES
    };

    // Possible radio states
    enum RadioState
    {
        IDLE = 0,
        SENDING_CW,
        SENDING_WSPR,
        NUM_STATES
    };

    // Other constants
    enum
    {
        WSPR_SYMBOL_COUNT = 162,
    };

    // Constructor/destructor/init
    DRadio();
    virtual ~DRadio() {};

    // Public API
    void Init();
    bool Enable();
    void Disable();
    void SendCW(char *msg, int len, int wpm);
    void SendWSPR();
    void Tick();

    inline RadioState GetState() { return State; }

private:
    Si5351_ConfigTypeDef RadioConfig;
    RadioState State;
    int Count;
    // For CW
    char *Ptr;
    uint32_t Code;
    // For WSPR
    uint8_t WSPRSymbols[WSPR_SYMBOL_COUNT];


    // Si5351 helpers
    bool FindSynthParams(Si5351_ConfigTypeDef *Params, unsigned hz, unsigned frac256, unsigned odiv);

    // CW helpers
    uint32_t AsciiToMorse(char symbol);

    // WSPR helpers
    void WSPREncode(uint8_t *symbols, WSPRType type);
    void WSPRConvolve(uint8_t *c, uint8_t *s, int message_size, int bit_size);
    void WSPRMergeSyncVector(uint8_t *g, uint8_t *symbols);
    uint32_t WSPRCallHash(const char *call);

    DRadio(const DRadio&);
    void operator=(const DRadio&);
};

extern DRadio g_radio;
