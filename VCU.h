
#include <vector>

class String;

enum PDMType
{
    ZE0_2011_2013,  // Default
    AZE0_2014_2017, // If CAN message 0x393 is identified on the EV-CAN
    ZE1_2018,       // If CAN message 0x1ED is identified on the EV-CAN
};

struct PDMStatus
{
    float plugVoltage;
    unsigned char plugVoltageMode;
    float activePowerKw;
    float availablePowerKw;
    bool plugInserted;

    String GetString();
};

struct Stats
{
    short rpm = 0;
    short motorTorque = 0;
    float motorPower = 0;

    float inverter_temperature = 0;
    float motor_temperature = 0;

    String GetString();
};

struct InverterStatus
{
    PDMType PDMModelType;

    unsigned short inverterVoltage = 0;
    float batteryVoltage = 0;

    Stats stats;

    bool error_state = false;
};

namespace VCU
{
void Initialize();
void Tick();

InverterStatus GetInverterStatus();
Stats GetMaxRecordedStats();
void ClearMaxRecordedStats();
PDMStatus GetPDMStatus();

void SetFinalTorqueRequest(short value);
bool SetMaxTorqueRequest(short value);
void SetBatteryDischargeLimit(float kilowatts);
void ToggleThrottlePrint();

bool ToggleGen2Codes();
bool TogglePDMCAN();

bool IsIgnitionOn();

void SetContactorForTesting(int value);
} // namespace VCU
