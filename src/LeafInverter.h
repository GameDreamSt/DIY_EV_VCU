
#include <string>

class CAN;

namespace LeafInverter
{
    struct Stats
    {
        unsigned short inverterVoltage = 0;

        short rpm = 0;
        short motorTorque = 0;
        float motorPower = 0;

        float inverter_temperature = 0;
        float motor_temperature = 0;

        bool error_state = false;

        std::string GetString();
    };

    void Initialize();

    void Msgs10ms(CAN *can);
    void Msgs100ms(CAN *can);
    bool HandleCAN(unsigned int canID, int inFrameSize, unsigned char inFrame[8]);

    Stats GetStats();
    void ClearStats();

    void SetTorqueNm(int value);
}