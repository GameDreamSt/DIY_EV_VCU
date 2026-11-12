
#include "VCU.h"

#include "EVLib/CAN.h"
#include "EVLib/Timer.h"
#include "EVLib/Sensor.h"
#include "EVLib/Filter.h"
#include "EVLib/EVTime.h"
#include "EVLib/MathUtils.h"
#include "EVLib/ADS_Sensor.h"
#include "EVLib/SerialPrint.h"
#include "EVLib/SerialReader.h"

#include "mcp2515.h"
#include "Contactor.h"
#include "OutlanderOBC.h"
#include "LeafInverter.h"

#include <cstring>
#include <Arduino.h>

using namespace std;

namespace VCU
{

CAN *can = nullptr;
uint8_t *outFrame = new uint8_t[8];

#define HVTimerTime 0.2f

FilterGroup ignitionFilter;
FilterGroup driveModeFilter;
FilterGroup brakesFilter;
FilterGroup reverseFilter;

Timer timer_Frames10 = Timer(0.01f);
Timer timer_Frames100 = Timer(0.1f);
Timer TimerHV = Timer(HVTimerTime);
Timer failureTimer = Timer(3);
Timer throttlePrintTimer = Timer(0.1f);
Timer vacuumPrintTimer = Timer(0.1f);

ThrottleManager throttleManager;
float throttleWarmupTime;

ADS_Sensor adsSensor;

Sensor vacuumSensor;
Sensor proximityPilotSensor;
float vacuumTimeOn;
bool vacuumOn;
bool vacuumPumpFailure;
bool printVacuumSensorData;
void ToggleVacuumSensorDebug() { printVacuumSensorData = !printVacuumSensorData; }

Sensor* GetVacuumSensor() { return &vacuumSensor; }

#define LOWEST_VOLTAGE 210
#define MAX_CHARGE_VOLTAGE 240
#define MAX_CHARGE_CURRENT 12 // limited by OBC

uint16_t ThrotVal = 0; // analog value of throttle position.

#define MAX_TORQUE 280 // Newton-meters
int maxTorqueNm = 280;
int torqueRequestOverrideNm = 0;
float reverseModeTorqueMultiplier = 0.2f;

void SetFinalTorqueRequest(int value)
{
    if (abs(value) > maxTorqueNm)
    {
        torqueRequestOverrideNm = 0;
        PrintSerialMessage("Torque requested is above Max torque of " + ToString(maxTorqueNm) + "!");
    }
    else
        torqueRequestOverrideNm = value;
}

bool SetMaxTorqueRequest(int value)
{
    if (value < 0)
    {
        PrintSerialMessage("Max torque requested is below 0!");
        return false;
    }

    if (value > MAX_TORQUE)
    {
        PrintSerialMessage("Max torque requested is above Max torque of " + ToString(MAX_TORQUE) + "!");
        return false;
    }

    maxTorqueNm = value;
    return true;
}

void SetTorque()
{
    auto parameters = *GetParameters();
    
    if (parameters.size() == 0)
    {
        PrintSerialMessage("Not enough parameters!");
        return;
    }

    short request = stoi(parameters[0]);
    SetFinalTorqueRequest(request);
}

void SetMaxTorque()
{
    auto parameters = *GetParameters();

    if (parameters.size() == 0)
    {
        PrintSerialMessage("Not enough parameters!");
        return;
    }

    short request = stoi(parameters[0]);
    if(SetMaxTorqueRequest(request))
        PrintSerialMessage("Torque set to " + ToString(request));
}

bool regenEnabled = true;
short regenTorque = 200;
short minRegenRPM = 100;  // Don't use 0
short maxRegenRPM = 1000; // Don't use 0
float regenSmoothTorque = 0;
float regenSmoothTorqueTime = 2;
bool SetRegenTorque(short value)
{
    if (value < 0)
    {
        PrintSerialMessage("Regen torque requested is below 0!");
        return false;
    }

    if (value > MAX_TORQUE)
    {
        PrintSerialMessage("Regen torque requested is above Max torque of " + ToString(MAX_TORQUE) + "!");
        return false;
    }

    regenTorque = value;
    return true;
}
bool SetRegenRPMRange(short lowRPM, short highRPM)
{
    if (lowRPM > highRPM)
    {
        PrintSerialMessage("Lower RPM limit can't be higher than high RPM limit!");
        return false;
    }

    if (lowRPM <= 0 || highRPM <= 0)
    {
        PrintSerialMessage("Regen disabled!");
        regenEnabled = false;
        return true;
    }

    if (!regenEnabled)
        PrintSerialMessage("Regen enabled!");
    regenEnabled = true;
    minRegenRPM = lowRPM;
    maxRegenRPM = highRPM;
}

bool ignitionOn;
bool driveMode;

bool can_status;

string GetOBCStatus()
{
    return GetDCDCData()->GetString() + "\n" + GetOBCData()->GetString();
}

enum PPStatus
{
    Unknown,
    NotConnected,
    NoVCUResistor,
    NoInletResistor,
    PlugNotInserted,
    PlugInserted,
    PlugMoving,
};

string PPStatusToString(PPStatus status)
{
    switch(status)
    {
        case PPStatus::Unknown: return "Unknown";
        case PPStatus::NotConnected: return "NotConnected";
        case PPStatus::NoVCUResistor: return "NoVCUResistor";
        case PPStatus::NoInletResistor: return "NoInletResistor";
        case PPStatus::PlugNotInserted: return "PlugNotInserted";
        case PPStatus::PlugInserted: return "PlugInserted";
        case PPStatus::PlugMoving: return "PlugMoving";
        default: return "Not implemented status {" + to_string((int)status) + "}";
    }

    return "";
}

bool printPP;
void ToggleDebugPP()
{
    printPP = !printPP;
}

float lastRawPPValue = 0;
Timer PPTimer = Timer(0.1f);
PPStatus GetProximityPilotStatus() // Assuming VCCVoltage
{
    if(PPTimer.HasTriggered())
    {
        lastRawPPValue = proximityPilotSensor.GetRawValue();
    }
    
    if(FloatAbout(lastRawPPValue, 0, 0.2f))
        return PPStatus::NotConnected;
    if(FloatAbout(lastRawPPValue, 1, 0.05f))
        return PPStatus::NoInletResistor;
    if(FloatAbout(lastRawPPValue, 0.81f, 0.05f))
        return PPStatus::PlugNotInserted;
    if(FloatAbout(lastRawPPValue, 0.40f, 0.05f))
        return PPStatus::PlugInserted;
    if(FloatAbout(lastRawPPValue, 0.58f, 0.05f))
        return PPStatus::PlugMoving;

    return PPStatus::Unknown;
}

bool ChargerStatusPlugInserted()
{
    return GetOBCData()->controlPilotDetected || GetProximityPilotStatus() == PPStatus::PlugInserted;
}

void SendCustomCanMessage(unsigned int ID, unsigned char data[8])
{
    can->Transmit(ID, 8, data);
}

enum MsgID
{
    // BMS
    RcvPowerLimits = 0x1DC, // From BMS, but for now, spoof it
    RcvBatteryState = 0x1DB, // Also for PDM from BMS, but for now, spoof it
    RcvSOC = 0x55B,
    RcvBatteryCapacity = 0x59E,
    RcvChargeStatus = 0x5BC,
};

bool IsCanIDValid(int ID)
{
    switch (ID)
    {
    case 0x1DC:
    case 0x1DB:
    case 0x55B:
    case 0x59E:
    case 0x5BC:
        return true;
    }

    return false;
}

uint16_t Checksum(uint8_t *data)
{
    uint16_t checksum = 0;
    for (int b = 0; b < 7; b++) // 7 CAN data bytes
    {
        uint8_t wholeByte = data[b];
        checksum += wholeByte >> 4;   // XXXX XXXX -> 0000 XXXX
        checksum += wholeByte & 0x0F; // XXXX XXXX & 0000 AAAA -> 0000 XXXX
    }

    return checksum;
}

void CAN_ChecksumNibble(uint8_t *data, uint8_t add, uint8_t mask)
{
    data[7] = (Checksum(data) + add) & mask;
}

ContactorTest contactorTest;

#define CASE_CONTACTOR(pin, test) case pin: digitalWrite(pin, state || contactorTest == test ? HIGH : LOW); break;

void SetContactor(int pinID, bool state)
{
    switch (pinID)
    {
        CASE_CONTACTOR(PIN_PRECHARGE, ContactorTest::Precharge);
        CASE_CONTACTOR(PIN_POS_CONTACTOR, ContactorTest::Positive);
        CASE_CONTACTOR(PIN_NEG_CONTACTOR, ContactorTest::Negative);
        CASE_CONTACTOR(PIN_VACUUM_PUMP, ContactorTest::Vacuum);
        CASE_CONTACTOR(PIN_WATER_PUMP, ContactorTest::Water);
        CASE_CONTACTOR(PIN_DCDC_ENABLE, ContactorTest::DCDC);
        CASE_CONTACTOR(PIN_CHARGER_IGN, ContactorTest::Charger);

    default:
        break;
    }
}

void SetContactorForTesting(int value)
{
    ContactorTest oldValue = contactorTest;
    ContactorTest newValue = (ContactorTest)value;

    if (newValue == contactorTest) // Toggle off
    {
        contactorTest = ContactorTest::None;
        SetContactor((int)oldValue, false);
    }
    else // Switch to new
    {
        contactorTest = newValue;
        SetContactor((int)oldValue, false);
        SetContactor((int)newValue, true);
    }
}

bool adsDebug = false;
void ToggleADSDebug()
{
    adsDebug = !adsDebug;
}

bool initialized = false;
void Initialize()
{
    if (initialized)
        return;
    initialized = true;

    ignitionFilter = FilterGroup();
    ignitionFilter.AddFilter(new BoolFilter(20));
    //ignitionFilter.AddFilter(new TimedFilter(0.1f));
    
    driveModeFilter = FilterGroup();
    driveModeFilter.AddFilter(new BoolFilter(20));
    //driveModeFilter.AddFilter(new TimedFilter(0.1f));

    brakesFilter = FilterGroup();
    brakesFilter.AddFilter(new BoolFilter(20));
    //brakesFilter.AddFilter(new TimedFilter(0.1f));

    reverseFilter = FilterGroup();
    reverseFilter.AddFilter(new BoolFilter(20));
    //reverseFilter.AddFilter(new TimedFilter(0.25f));

    pinMode(PIN_IGNITION, INPUT);
    pinMode(PIN_DRIVE_MODE, INPUT_PULLUP);
    pinMode(PIN_REVERSE, INPUT);
    pinMode(PIN_BRAKES, INPUT);

    pinMode(PIN_PRECHARGE, OUTPUT);
    pinMode(PIN_NEG_CONTACTOR, OUTPUT);
    pinMode(PIN_POS_CONTACTOR, OUTPUT);
    pinMode(PIN_VACUUM_PUMP, OUTPUT);
    pinMode(PIN_WATER_PUMP, OUTPUT);
    pinMode(PIN_DCDC_ENABLE, OUTPUT);
    pinMode(PIN_CHARGER_IGN, OUTPUT);

    SetContactor(PIN_PRECHARGE, false);
    SetContactor(PIN_NEG_CONTACTOR, false);
    SetContactor(PIN_POS_CONTACTOR, false);
    SetContactor(PIN_VACUUM_PUMP, false);
    SetContactor(PIN_WATER_PUMP, false);
    SetContactor(PIN_DCDC_ENABLE, false);
    SetContactor(PIN_CHARGER_IGN, false);

    adsSensor = ADS_Sensor(ADS1115::I2CAddress::Gnd, ADS1115::DataRate::SPS_64, ADS1115::Pga::FSR_4_096V);

    Sensor throttle1 = Sensor([](){ return adsSensor.GetVoltage(CHANNEL_THROTTLE1);});
    throttle1.SetName("T1");
    //throttle1.SetValuesManually(0.28f, 0.88f);
    throttleManager.AddThrottle(throttle1);

    Sensor throttle2 = Sensor([](){ return adsSensor.GetVoltage(CHANNEL_THROTTLE2);});
    throttle2.SetName("T2");
    //throttle2.SetValuesManually(0.22f, 0.5f);
    throttleManager.AddThrottle(throttle2);

    throttleWarmupTime = 1;

    vacuumSensor = Sensor([](){ return adsSensor.GetVoltage(CHANNEL_VACUUM);});
    vacuumSensor.SetName("Vac");
    proximityPilotSensor = Sensor([](){ return adsSensor.GetVoltage(CHANNEL_PP);});
    proximityPilotSensor.SetName("PP");

    AddCommand("torque", SetTorque);
    AddCommand("maxtorque", SetMaxTorque);
    AddCommand("debugads", ToggleADSDebug);

    LeafInverter::Initialize();
}

float prechargeToFailureTime = 0;

bool prechargeFailure = false;
bool prechargeComplete = false;

float lastInverterVoltage;
float lastInverterVoltageTime;

bool wantedToCharge;
bool WantsToCharge()
{
    bool value = ignitionOn && prechargeComplete && ChargerStatusPlugInserted();

    if(wantedToCharge != value)
    {
        wantedToCharge = value;
        if(value)
            PrintSerialMessage("Initializing charging mode");
        else
            PrintSerialMessage("Stopping charging mode");
    }

    return value;
}

void ClearHVData()
{
    lastInverterVoltageTime = lastInverterVoltage = prechargeToFailureTime = 0;
    vacuumPumpFailure = prechargeFailure = prechargeComplete = false;
    LeafInverter::ClearStats();
}

bool IsIgnitionOn()
{
    return ignitionOn;
}

void CheckIgnition()
{
    ignitionFilter.SetData(!digitalRead(PIN_IGNITION)); // PIN ON BY DEFAULT : INPUT_PULLUP

    if (ignitionFilter.GetData() || ChargerStatusPlugInserted())
    {
        if (!ignitionOn)
            PrintSerialMessage("Ignition on");
        ignitionOn = true;
        can_status = true;
    }
    else
    {
        if (ignitionOn)
            PrintSerialMessage("Ignition off");
        ignitionOn = false;
        can_status = false;
        ClearHVData();
    }
}

void CheckDriveMode()
{
    if (ChargerStatusPlugInserted())
    {
        driveMode = false;
        return;
    }

    driveModeFilter.SetData(!digitalRead(PIN_DRIVE_MODE)); // PIN ON BY DEFAULT : INPUT_PULLUP
    brakesFilter.SetData(!digitalRead(PIN_BRAKES));

    if (ignitionOn && driveModeFilter.GetData() && brakesFilter.GetData()) // Must turn starter and press brakes to active drive
    {
        if (!driveMode)
            PrintSerialMessage("Drive mode on");
        driveMode = true;
    }
    else if (!ignitionOn)
    {
        if (driveMode)
            PrintSerialMessage("Drive mode off");
        driveMode = false;
    }
}

int GetMotorVoltage()
{
    return LeafInverter::GetStats().inverterVoltage;
}

void HighVoltageControl()
{
    if (!ignitionOn || prechargeFailure) // || inverterStatus.batteryVoltage < LOWEST_VOLTAGE
    {
        SetContactor(PIN_PRECHARGE, false);
        SetContactor(PIN_POS_CONTACTOR, false);
        SetContactor(PIN_NEG_CONTACTOR, false);
        ClearHVData();
        return;
    }

    if (prechargeComplete)
        return;

    if (prechargeToFailureTime > 5)
    {
        prechargeFailure = true;
        return;
    }

    // float voltageDifference = abs(motorVoltage - inverterStatus.batteryVoltage);

    int motorVoltage = GetMotorVoltage();
    if (abs(motorVoltage - lastInverterVoltage) > 5)
    {
        lastInverterVoltage = motorVoltage;
        lastInverterVoltageTime = 0;
    }

    lastInverterVoltageTime += HVTimerTime;

    if (lastInverterVoltageTime <= 2 || lastInverterVoltage < LOWEST_VOLTAGE) // Must be within 20V and above 180V
    {
        SetContactor(PIN_POS_CONTACTOR, false);
        SetContactor(PIN_NEG_CONTACTOR, true);
        SetContactor(PIN_PRECHARGE, true);
        prechargeToFailureTime += HVTimerTime;
        return;
    }

    prechargeToFailureTime = 0;
    prechargeComplete = true;
    SetContactor(PIN_POS_CONTACTOR, true);
    SetContactor(PIN_NEG_CONTACTOR, true);
    SetContactor(PIN_PRECHARGE, false);
}

bool ChargerControl()
{
    bool shouldChargerBeOn = ignitionOn || GetProximityPilotStatus() >= PPStatus::PlugInserted;

    if(shouldChargerBeOn)
    {
        SetContactor(PIN_CHARGER_IGN, true);
        return shouldChargerBeOn;
    }

    if(GetOBCData()->HV_Current < 1) // Can't allow to turn off if we're still charging
        SetContactor(PIN_CHARGER_IGN, false);
    return shouldChargerBeOn;
}

void HVChargeControl()
{
    if(!ChargerControl())
    {
        SetChargeStatus(CmdChargeStatus::Off, MAX_CHARGE_VOLTAGE, 0);
        return;
    }

    int motorVoltage = GetMotorVoltage();
    bool canCharge = prechargeComplete && motorVoltage < MAX_CHARGE_VOLTAGE && GetOBCData()->MaxTemperature() < 60;

    CmdChargeStatus chargeStatus = ChargerStatusPlugInserted() ? (canCharge ? CmdChargeStatus::Charge : CmdChargeStatus::Connect) : CmdChargeStatus::Off;

    unsigned char chargeCurrent = MAX_CHARGE_CURRENT;

    /*short maxMinVoltDifference = MAX_CHARGE_VOLTAGE - LOWEST_VOLTAGE;
    short slowChargeThreshold = LOWEST_VOLTAGE + maxMinVoltDifference * 0.8;

    if(motorVoltage > slowChargeThreshold)
        chargeCurrent *= 0.5;*/

    SetChargeStatus(chargeStatus, MAX_CHARGE_VOLTAGE, chargeCurrent);
}

Timer lowChargeTimer = Timer(30);
Timer lowChargeCooldownTimer = Timer(5);
bool lowCharging = false;
bool ShouldChargeDCDC()
{
    bool canCharge = prechargeComplete && GetOBCData()->HV_Voltage > LOWEST_VOLTAGE && GetDCDCData()->MaxTemperature() < 60;

    if(!canCharge)
        return false;

    if(driveMode)
        return true;

    if(!ignitionOn && !ChargerStatusPlugInserted())
        return false;

    if(lowCharging)
    {
        if(lowChargeTimer.HasTriggered())
        {
            PrintSerialMessage("DC/DC stopped");
            lowCharging = false;
        }
    }
    else if(lowChargeCooldownTimer.HasTriggered() && GetDCDCData()->supplyVoltage < 12.3f)
    {
        PrintSerialMessage("DC/DC started");
        lowCharging = true;
    }

    return lowCharging;
}

void LowChargeControl()
{
    digitalWrite(PIN_DCDC_ENABLE, ShouldChargeDCDC());
}

bool ShouldTurnOnPump() // Not sure if it's ok to wait for temperatures to rise or we need to circulate the coolant all the time
{
    if(driveMode)
        return true;

    auto inverterStats = LeafInverter::GetStats();
    if(inverterStats.inverter_temperature > 30)
        return true;
    if(inverterStats.motor_temperature > 30)
        return true;
    if(GetOBCData()->MaxTemperature() > 30)
        return true;
    if(GetDCDCData()->MaxTemperature() > 30)
        return true;

    return false;
}

Timer waterPumpTimer = Timer(5);
void TemperatureControl()
{
    if(waterPumpTimer.HasTriggered())
        SetContactor(PIN_WATER_PUMP, ShouldTurnOnPump());
}

void Msgs10ms()
{
    if (!can_status)
        return;

    OBCMsgs10Ms(can);
}

void Msgs100ms()
{
    if (!can_status)
        return;

    LeafInverter::Msgs100ms(can);
    OBCMsgs100Ms(can);
}

bool printThrottle;
void ToggleThrottlePrint()
{
    printThrottle = !printThrottle;
}

bool printThrottleDetailed;
void ToggleThrottlePrintDetailed()
{
    printThrottleDetailed = !printThrottleDetailed;
}

void ReadPedals()
{
    float normalizedThrottle = throttleManager.GetNormalized();
    int torqueRequestNm = 0;
    LeafInverter::SetTorqueNm(0); // Reset

    if(!prechargeComplete)
        return;

    if (torqueRequestOverrideNm != 0)
        torqueRequestNm = torqueRequestOverrideNm;
    else if (driveMode)
    {
        auto inverterStats = LeafInverter::GetStats();
        reverseFilter.SetData(!digitalRead(PIN_REVERSE));
        bool reverseMode = reverseFilter.GetData();

        if (normalizedThrottle < 0.01f && regenEnabled && !reverseMode)
        {
            float normalizedRegen = (float)(inverterStats.rpm - minRegenRPM) / (float)(maxRegenRPM - minRegenRPM);
            normalizedRegen = Max(0, Min(1, normalizedRegen));
            torqueRequestNm = -regenSmoothTorque * normalizedRegen;
            regenSmoothTorque = Max(0, Min(regenSmoothTorque + regenTorque * deltaTime / regenSmoothTorqueTime, regenTorque));
        }
        else
        {
            torqueRequestNm = maxTorqueNm * normalizedThrottle;
            if(reverseMode)
                torqueRequestNm *= reverseModeTorqueMultiplier;
            regenSmoothTorque = 0;
        }
    }

    LeafInverter::SetTorqueNm(torqueRequestNm);
}

can_frame recvFrame;
void ReadCAN()
{
    if (can == nullptr || !can->GetCanData(recvFrame))
        return;

    uint8_t inFrame[8];
    memcpy(&inFrame, recvFrame.data, Min(recvFrame.can_dlc, 8));

    if (!IsCanIDValid(recvFrame.can_id))
    {
        if(LeafInverter::HandleCAN(recvFrame.can_id, recvFrame.can_dlc, inFrame))
            return;

        if(OBCHandleCAN(recvFrame.can_id, recvFrame.can_dlc, inFrame))
            return;

        PrintSerialMessage("Unknown CAN message ID: 0x" + IntToHex(recvFrame.can_id) +
                           " Bytes: " + BytesToString(recvFrame.data, recvFrame.can_dlc));
        return;
    }

    MsgID messageType = (MsgID)recvFrame.can_id;

    // Handle CAN message
    switch (messageType)
    {
        default:
            break;
    }
}

void PrintFailures()
{
    if(failureTimer.HasTriggered())
    {
        if (prechargeFailure)
            PrintSerialMessage("Precharge failure! Inverter didn't reach battery voltage in 5 seconds!");

        //if (vacuumPumpFailure)
            //PrintSerialMessage("Vacuum pump failure! Didn't reach target vacuum in some time...");
    }
}

void PrintDebug()
{
    if(!throttlePrintTimer.HasTriggered())
        return;

    if (printThrottle)
        PrintSerialMessage(ToString(throttleManager.GetNormalized()));

    if(printThrottleDetailed)
        throttleManager.PrintDebugValues();

    if(adsDebug)
        adsSensor.PrintDebug();

    if(printPP)
    {
        auto ppStatus = GetProximityPilotStatus();
        PrintSerialMessage("PPStatus: " + PPStatusToString(ppStatus) + " | " + proximityPilotSensor.GetDebugValuesString());
    }
}

void ControlVacuum()
{
    if(printVacuumSensorData && vacuumPrintTimer.HasTriggered())
    {
        vacuumSensor.PrintDebugValues();
    }

    float rawValue = vacuumSensor.GetRawValue();
    if(rawValue < 0.01f || vacuumPumpFailure || !driveMode || ChargerStatusPlugInserted()) // Disconnected or failed or charging mode
    {
        SetContactor(PIN_VACUUM_PUMP, false);
        vacuumTimeOn = 0;
        return;
    }
        
    if(rawValue > 0.3f)
    {
        SetContactor(PIN_VACUUM_PUMP, true);
        vacuumOn = true;
    }
    else if(rawValue < 0.15f)
    {
        SetContactor(PIN_VACUUM_PUMP, false);
        vacuumOn = false;
    }

    if(vacuumOn && rawValue > 0.4f)
        vacuumTimeOn += deltaTime;
    else
        vacuumTimeOn = 0;

    if(vacuumTimeOn > 10)
        vacuumPumpFailure = true;
}

void Tick()
{
    if (can == nullptr) // Initialize later so the upload process doesn't hang up
        can = new CAN(MKRCAN_MCP2515_CS_PIN);

    if(throttleWarmupTime > 0)
        throttleWarmupTime -= deltaTime;
    else
        throttleManager.Tick();
    vacuumSensor.Tick();
    proximityPilotSensor.Tick();

    adsSensor.Tick();

    ReadPedals();
    ControlVacuum();

    CheckIgnition();
    CheckDriveMode();

    LowChargeControl();
    TemperatureControl();

    if (TimerHV.HasTriggered())
    {
        HighVoltageControl();
        HVChargeControl();
    }

    ReadCAN();

    if (timer_Frames10.HasTriggered())
        Msgs10ms();
    if (timer_Frames100.HasTriggered())
        Msgs100ms();

    PrintFailures();
    PrintDebug();
}
} // namespace VCU