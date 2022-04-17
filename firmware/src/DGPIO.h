#pragma once

class DGPIO
{
public:
    enum GPIOName
    {
        SHUNT_HORIZ = 0,
        SHUNT_VERTA,
        SHUNT_VERTB,
        ENA_5351,
        NUM_GPIONAMES,
    };

    enum GPIOPort
    {
        PortA = 0,
        PortB,
        PortC,
        NUM_GPIOPORTS
    };

    enum GPIOIO
    {
        Input,
        Output,
        NUM_GPIOIOS
    };

    enum GPIOPUPD
    {
        PU,
        PD,
        Float,
        NUM_GPIOPUPDS
    };
        
    struct GPIOTable
    {
        enum GPIOName name : 8;
        unsigned pin       : 8;
        enum GPIOPort port : 4;
        enum GPIOIO io     : 4;
        enum GPIOPUPD pupd : 4;
        unsigned init      : 4;
    };

    static const GPIOTable gpios[];

    DGPIO();
    virtual ~DGPIO (){};

    // Public API
    void Init();

    // Client API
    bool Status(GPIOName name);
    void Set(GPIOName name);
    void Clear(GPIOName name);
    void Toggle(GPIOName name);

private:
    DGPIO(const DGPIO&);
    void operator=(const DGPIO&);
};

extern DGPIO g_gpio;
