
#include <vector>
#include <string>

class Throttle;

struct Stats
{
    short rpm = 0;
    short motorTorque = 0;
    float motorPower = 0;

    float inverter_temperature = 0;
    float motor_temperature = 0;

    std::string GetString();
};

struct InverterStatus
{
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
std::string GetOBCStatus();

void SetFinalTorqueRequest(short value);
bool SetMaxTorqueRequest(short value);

bool SetRegenTorque(short value);
bool SetRegenRPMRange(short lowRPM, short highRPM);

void SetBatteryDischargeLimit(float kilowatts);
void ToggleThrottlePrint();
Throttle* GetVacuumSensor();

void SendCustomCanMessage(unsigned int ID, unsigned char data[8]);

bool ToggleGen2Codes();
bool TogglePDMCAN();

bool IsIgnitionOn();

void SetContactorForTesting(int value);
} // namespace VCU
