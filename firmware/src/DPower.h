#pragma once

#include <stdint.h>

class DPower
{
public:
    // Constructor/destructor/init
    DPower();
    virtual ~DPower() {};

    // Public API
    void Init();       // Call during system init
    void Stop();

    inline bool VDDGood() { return (!OverVoltage); }

    // PVD interrupt handler
    void HighVDD();

private:
    bool OverVoltage;
    uint16_t CalCount;

    DPower(const DPower&);
    void operator=(const DPower&);
};

extern DPower g_power;
