
#include <vector>
#include <string>

class Throttle;

namespace VCU
{
    void Initialize();
    void Tick();

    std::string GetOBCStatus();
    bool WantsToCharge();

    void ToggleDebugPP();

    void ToggleThrottlePrint();
    void ToggleThrottlePrintDetailed();

    bool SetRegenTorque(short value);
    bool SetRegenRPMRange(short lowRPM, short highRPM);

    Throttle* GetVacuumSensor();

    void SendCustomCanMessage(unsigned int ID, unsigned char data[8]);

    bool IsIgnitionOn();

    void SetContactorForTesting(int value);
} // namespace VCU
