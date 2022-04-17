#pragma once

#include <stdint.h>

class DRTC
{
public:
    // Constructor/destructor/init
    DRTC();
    virtual ~DRTC() {};

    // Public API
    void Init();       // Call during system init
    void SetAlarm(int hh, int mm, int ss, uint8_t frac256, bool interrupt);
    void ClearAlarm();
    void SetPeriodicWake(uint16_t interval, bool hires, bool interrupt);
    void ClearPeriodicWake();
    unsigned RunMSICal();

    inline bool LSEGood() { return (!LSEFailed); }

    // RTC interrupt handlers
    void RTCWake();
    void RTCAlarm();
    void LSEFailure();

private:
    bool LSEFailed;
    uint16_t CalCount;

    DRTC(const DRTC&);
    void operator=(const DRTC&);
};

extern DRTC g_rtc;
extern bool Debug;
