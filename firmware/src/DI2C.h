#pragma once

#include <stdint.h>

class DI2C
{
public:
    enum Device
    {
        ADDR_LPS225HB = 0x53,
        ADDR_SI5351 = 0x60,
    };

    enum
    {
        TIMEOUT_COUNTS = 10000,
    };

    DI2C();
    virtual ~DI2C (){};

    // Public API
    void Init();
    void Deinit();
    int Write(enum Device dev, uint8_t reg, uint8_t data);
    uint8_t Read(enum Device dev, uint8_t reg);

    void Tick();
    bool DetectPressure();
    bool DetectTX();

private:
    volatile bool Busy;
    int Count;
    int *pSeq;
    uint8_t Data;

    DI2C(const DI2C&);
    void operator=(const DI2C&);
};

extern DI2C g_i2c;
