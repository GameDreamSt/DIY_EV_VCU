
#include <vector>
#include <Arduino.h>

class Throttle;

enum PDMType
{
    ZE0_2011_2013,  // Default
    AZE0_2014_2017, // If CAN message 0x393 is identified on the EV-CAN
    ZE1_2018,       // If CAN message 0x1ED is identified on the EV-CAN
};

struct PDMStatus
{
    unsigned char sleepEnabled;
    float plugVoltage;
    float activePowerKw;
    float availablePowerKw;
    bool plugInserted;
    unsigned char chargerStatus;
    unsigned char DCtoDCStatus;

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
