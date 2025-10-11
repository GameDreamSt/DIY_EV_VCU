
#include <string>

class CAN;

enum CmdChargeStatus
{
    Off,
    Connect,
    Charge,
};

struct DCDC_Data
{
    float supplyVoltage;
    float supplyCurrent;

    bool ready;
    bool operating;
    bool error;

    short temperature[3];

    short MaxTemperature();
    std::string GetString();
};

struct OBC_Data
{
    short HV_Voltage;
    short HV_Voltage2;
    float HV_Current;

    short AC_Voltage;

    bool controlPilotDetected;
    char controlPilotDutyCyclePercentage;

    bool ACVoltageDetected;
    bool DCDC_Status_Requested;

    bool error;
    bool charging;

    short temperature[4];

    short MaxTemperature();
    std::string GetString();
};

void SetChargeStatus(CmdChargeStatus status, float OBC_HVTargetVoltage, unsigned char OBC_ChargingCurrent);
DCDC_Data* GetDCDCData();
OBC_Data* GetOBCData();
void OBCMsgs10Ms(CAN *can);
void OBCMsgs100Ms(CAN *can);
bool OBCHandleCAN(unsigned int canID, int inFrameSize, unsigned char inFrame[8]);