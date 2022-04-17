#pragma once

#include <stdint.h>

class DADC
{
public:
    // Constructor/destructor/init
    DADC();
    virtual ~DADC() {};

    // Public API
    void Init();       // Call during system init
    void Sleep();
    void Wake();
    void Read(float *vdd, float *i_h, float *i_va, float *i_vb);

private:

    DADC(const DADC&);
    void operator=(const DADC&);
};

extern DADC g_adc;
