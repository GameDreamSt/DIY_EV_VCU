
#include "VCU.h"

#include "CAN.h"
#include "Contactor.h"
#include "Filter.h"
#include "SerialPrint.h"
#include "Throttle.h"
#include "Timer.h"
#include "Time.h"
#include "mcp2515.h"
#include "OutlanderOBC.h"
#include "Math.h"

String Stats::GetString()
{
    return "RPM: " + ToString(rpm) + "\nMotor torque: " + ToString(motorTorque) + " nm" +
           "\nMotor power: " + FloatToString(motorPower, 1) + " kw" +
           "\nInverter temperature: " + FloatToString(inverter_temperature, 2) + " C " +
           "\nMotor temperature: " + FloatToString(motor_temperature, 2) + " C ";
}

namespace VCU
{
#ifndef byte
#define byte unsigned char
#endif

#ifndef ushort
#define ushort unsigned short
#endif

CAN *can = nullptr;
byte *outFrame = new byte[8];

#define HVTimerTime 0.2f

TimedFilter<bool> ignitionFilter = TimedFilter<bool>(100);
TimedFilter<bool> driveModeFilter = TimedFilter<bool>(100);

Timer timer_Frames10 = Timer(0.01f);
Timer timer_Frames100 = Timer(0.1f);
Timer timer_response_50MS = Timer(0.05f);
Timer TimerHV = Timer(HVTimerTime);
Timer failureTimer = Timer(3);
Timer throttlePrintTimer = Timer(0.1f);

ThrottleManager throttleManager;
Throttle vacuumSensor = Throttle(APIN_VACUUM);
Throttle proximityPilotSensor = Throttle(APIN_CHARGER_PP);
float vacuumTimeOn;
bool vacuumOn;
bool vacuumPumpFailure;

Throttle* GetVacuumSensor() { return &vacuumSensor; }

#define LOWEST_VOLTAGE 210
#define MAX_CHARGE_VOLTAGE 249
#define MAX_CHARGE_CURRENT 12 // limited by OBC

ushort ThrotVal = 0; // analog value of throttle position.

#define MAX_TORQUE 1120 // Max torque request is 1120
short MaxTorque = 1120;
short final_torque_request = 0;
short torqueRequestOverride = 0;
void SetFinalTorqueRequest(short value)
{
    if (abs(value) > MaxTorque)
    {
        torqueRequestOverride = 0;
        PrintSerialMessage("Torque requested is above max torque of " + ToString(MaxTorque) + "!");
    }
    else
        torqueRequestOverride = value;
}

bool SetMaxTorqueRequest(short value)
{
    if (value < 0)
    {
        PrintSerialMessage("Max torque requested is below 0!");
        return false;
    }

    if (value > MAX_TORQUE)
    {
        PrintSerialMessage("Max torque requested is above max torque of " + ToString(MAX_TORQUE) + "!");
        return false;
    }

    MaxTorque = value;
    return true;
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
        PrintSerialMessage("Regen torque requested is above max torque of " + ToString(MAX_TORQUE) + "!");
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

bool gen2Codes = true;
bool ToggleGen2Codes()
{
    gen2Codes = !gen2Codes;
    return gen2Codes;
}

InverterStatus inverterStatus;
InverterStatus GetInverterStatus()
{
    return inverterStatus;
}

Stats maxStats;
Stats GetMaxRecordedStats()
{
    return maxStats;
}
void ClearMaxRecordedStats()
{
    maxStats = Stats();
}

String GetOBCStatus()
{
    return GetDCDCData()->GetString() + "\n" + GetOBCData()->GetString();
}

enum PPStatus
{
    NotConnected,
    NoVCUResistor,
    NoInletResistor,
    PlugNotInserted,
    PlugInserted,
    PlugMoving,
    Unknown,
};

float lastRawPPValue = 0;
Timer PPTimer = Timer(0.1f);
PPStatus GetProximityPilotStatus() // Assuming VCCVoltage
{
    if(PPTimer.HasTriggered())
    {
        lastRawPPValue = proximityPilotSensor.GetRawValue() / VCCVoltage;
    }
    
    if(FloatAbout(lastRawPPValue, 0, 0.5f))
        return PPStatus::NotConnected;
    if(FloatAbout(lastRawPPValue, 1, 0.05f))
        return PPStatus::NoInletResistor;
    if(FloatAbout(lastRawPPValue, 0.891f, 0.03f)) // 2700 / (2700 + 330)
        return PPStatus::PlugNotInserted;
    if(FloatAbout(lastRawPPValue, 0.905f, 0.03f)) // (2700 + 150 + 330) / (2700 + 150 + 330 + 330)
        return PPStatus::PlugInserted;
    if(FloatAbout(lastRawPPValue, 0.896f, 0.03f)) // (2700 + 150) / (2700 + 150 + 330)
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

struct DummyQueryResponse
{
    unsigned int queryID;
    unsigned int responseID;
    byte responseData[8];
    bool needsToRespond;

    DummyQueryResponse(){}
    DummyQueryResponse(unsigned int qID, unsigned int rID)
    {
        queryID = qID;
        responseID = rID;
    }
    DummyQueryResponse(unsigned int qID, unsigned int rID, byte rData[8])
    {
        queryID = qID;
        responseID = rID;
        memcpy(&responseData, &rData, 8);
    }
    DummyQueryResponse(unsigned int qID, unsigned int rID, unsigned long long rData)
    {
        queryID = qID;
        responseID = rID;
        memcpy(&responseData, &rData, 8);
    }
};
std::vector<DummyQueryResponse> dummyQueries;

enum MsgID
{
    // Inverter IDs
    CmdHeartBeat = 0x50B,
    CmdGearSelection = 0x11A,
    CmdTorque = 0x1D4,
    RcvInverterState = 0x1DA,
    RcvTempF = 0x55A,

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
    case 0x50B:
    case 0x11A:
    case 0x1D4:
    case 0x1DA:
    case 0x55A:
    case 0x1DC:
    case 0x1DB:
    case 0x55B:
    case 0x59E:
    case 0x5BC:
        return true;
    }

    return false;
}

void NissanCRC(byte *data)
{
    byte polynomial = 0x85;

    // We want to process 8 bytes with the 8th byte being zero
    data[7] = 0;
    byte crc = 0;
    for (int b = 0; b < 8; b++)
    {
        for (int i = 7; i >= 0; i--)
        {
            byte bit = ((data[b] & (1 << i)) > 0) ? 1 : 0;
            if (crc >= 0x80)
                crc = (byte)(((crc << 1) + bit) ^ polynomial);
            else
                crc = (byte)((crc << 1) + bit);
        }
    }
    data[7] = crc;
}

ushort Checksum(byte *data)
{
    ushort checksum = 0;
    for (int b = 0; b < 7; b++) // 7 CAN data bytes
    {
        byte wholeByte = data[b];
        checksum += wholeByte >> 4;   // XXXX XXXX -> 0000 XXXX
        checksum += wholeByte & 0x0F; // XXXX XXXX & 0000 AAAA -> 0000 XXXX
    }

    return checksum;
}

void CAN_ChecksumNibble(byte *data, byte add, byte mask)
{
    data[7] = (Checksum(data) + add) & mask;
}

ContactorTest contactorTest;

void SetContactor(int pinID, bool state)
{
    switch (pinID)
    {
    case PIN_PRECHARGE:
        digitalWrite(PIN_PRECHARGE, state || contactorTest == ContactorTest::Precharge ? HIGH : LOW);
        break;

    case PIN_POS_CONTACTOR:
        digitalWrite(PIN_POS_CONTACTOR, state || contactorTest == ContactorTest::Positive ? HIGH : LOW);
        break;

    case PIN_NEG_CONTACTOR:
        digitalWrite(PIN_NEG_CONTACTOR, state || contactorTest == ContactorTest::Negative ? HIGH : LOW);
        break;

    case PIN_VACUUM_PUMP:
        digitalWrite(PIN_VACUUM_PUMP, state || contactorTest == ContactorTest::Vacuum ? HIGH : LOW);
        break;

    case PIN_WATER_PUMP:
        digitalWrite(PIN_WATER_PUMP, state || contactorTest == ContactorTest::Water ? HIGH : LOW);
        break;

    case PIN_DCDC_ENABLE:
        digitalWrite(PIN_DCDC_ENABLE, state || contactorTest == ContactorTest::DCDC ? HIGH : LOW);
        break;

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

bool initialized = false;
void Initialize()
{
    if (initialized)
        return;
    initialized = true;

    pinMode(DEBUG_LED, OUTPUT);

    pinMode(PIN_IGNITION, INPUT_PULLUP);
    pinMode(PIN_DRIVE_MODE, INPUT_PULLUP);

    pinMode(PIN_PRECHARGE, OUTPUT);
    pinMode(PIN_POS_CONTACTOR, OUTPUT);
    pinMode(PIN_NEG_CONTACTOR, OUTPUT);
    pinMode(PIN_VACUUM_PUMP, OUTPUT);
    pinMode(PIN_WATER_PUMP, OUTPUT);
    pinMode(PIN_DCDC_ENABLE, OUTPUT);

    SetContactor(PIN_PRECHARGE, false);
    SetContactor(PIN_POS_CONTACTOR, false);
    SetContactor(PIN_NEG_CONTACTOR, false);
    SetContactor(PIN_VACUUM_PUMP, false);
    SetContactor(PIN_WATER_PUMP, false);

    throttleManager.AddThrottle(Throttle(APIN_Throttle1));
    throttleManager.AddThrottle(Throttle(APIN_Throttle2));

    dummyQueries.push_back(DummyQueryResponse(0x797, 0x79A, 0x04621103A3000000)); // 12V battery, [5]A3 = 13.04V / 0.08
    dummyQueries.push_back(DummyQueryResponse(0x745, 0x765)); // BCM body control module
    dummyQueries.push_back(DummyQueryResponse(0x740, 0x760)); // ABS
    dummyQueries.push_back(DummyQueryResponse(0x79B, 0x7BB)); // BMS/LBC
    dummyQueries.push_back(DummyQueryResponse(0x743, 0x763)); // M&A (Meter)
    dummyQueries.push_back(DummyQueryResponse(0x744, 0x764)); // HVAC
    dummyQueries.push_back(DummyQueryResponse(0x70E, 0x70F)); // Brake
    dummyQueries.push_back(DummyQueryResponse(0x73F, 0x761)); // VSP Vehicle Sound for Pedestrians
    dummyQueries.push_back(DummyQueryResponse(0x742, 0x762)); // EPS Electric Power Steering system
    dummyQueries.push_back(DummyQueryResponse(0x746, 0x783)); // TCU Telematics control unit
    dummyQueries.push_back(DummyQueryResponse(0x747, 0x767)); // Multi AV
    dummyQueries.push_back(DummyQueryResponse(0x74D, 0x76D)); // IPDM E/R
    dummyQueries.push_back(DummyQueryResponse(0x752, 0x772)); // Airbag
    dummyQueries.push_back(DummyQueryResponse(0x79D, 0x7BD)); // Shift
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
    inverterStatus.inverterVoltage = 0;
    inverterStatus.error_state = false;
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

    if (ignitionOn && driveModeFilter.GetData())
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

    // float voltageDifference = abs(inverterStatus.inverterVoltage - inverterStatus.batteryVoltage);

    if (abs(inverterStatus.inverterVoltage - lastInverterVoltage) > 5)
    {
        lastInverterVoltage = inverterStatus.inverterVoltage;
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

void HVChargeControl()
{
    bool canCharge = prechargeComplete; //&& inverterStatus.inverterVoltage < MAX_CHARGE_VOLTAGE;

    CmdChargeStatus chargeStatus = ChargerStatusPlugInserted() ? (canCharge ? CmdChargeStatus::Charge : CmdChargeStatus::Connect) : CmdChargeStatus::Off;

    unsigned char chargeCurrent = MAX_CHARGE_CURRENT;

    /*short maxMinVoltDifference = MAX_CHARGE_VOLTAGE - LOWEST_VOLTAGE;
    short slowChargeThreshold = LOWEST_VOLTAGE + maxMinVoltDifference * 0.8;

    if(inverterStatus.inverterVoltage > slowChargeThreshold)
        chargeCurrent *= 0.5;*/

    SetChargeStatus(chargeStatus, MAX_CHARGE_VOLTAGE, chargeCurrent);
}

Timer lowChargeTimer = Timer(30);
Timer lowChargeCooldownTimer = Timer(5);
bool lowCharging = false;
bool ShouldChargeDCDC()
{
    bool canCharge = prechargeComplete && inverterStatus.inverterVoltage > LOWEST_VOLTAGE;

    if(!canCharge)
        return false;

    if(ignitionOn)
        return true;

    if(!ChargerStatusPlugInserted())
        return false;

    if(lowCharging)
    {
        if(lowChargeTimer.HasTriggered())
        {
            PrintSerialMessage("DC/DC stopped");
            lowCharging = false;
        }
    }
    else if(lowChargeCooldownTimer.HasTriggered() && GetDCDCData()->supplyVoltage < 12.5f)
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

    if(inverterStatus.stats.inverter_temperature > 30)
        return true;
    if(inverterStatus.stats.motor_temperature > 30)
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

void SendHeartBeat()
{
    // Statistics from 2016 capture:
    //     10 00000000000000
    //     21 000002c0000000
    //    122 000000c0000000
    //    513 000006c0000000

    outFrame[0] = 0x00;
    outFrame[1] = 0x00;
    outFrame[2] = 0x06;
    outFrame[3] = 0xc0;
    outFrame[4] = 0x00;
    outFrame[5] = 0x00;
    outFrame[6] = 0x00;

    can->Transmit((int)MsgID::CmdHeartBeat, 7, outFrame);
}

void Msgs10ms()
{
    if (!can_status)
        return;

    static byte counter_11a_d6 = 0;
    static byte counter_1d4 = 0;
    static byte counter_1db = 0;

    // Send VCM gear selection signal (gets rid of P3197)
    // Data taken from a gen1 inFrame where the car is starting to
    // move at about 10% throttle: 4E400055 0000017D

    // All possible gen1 values: 00 01 0D 11 1D 2D 2E 3D 3E 4D 4E
    // MSB nibble: Selected gear (gen1/LeafLogs)
    //   0: some kind of non-gear before driving
    //   1: some kind of non-gear after driving
    //   2: R
    //   3: N
    //   4: D
    // LSB nibble: ? (LeafLogs)
    //   0: sometimes at startup, not always; never when the
    //      inverted is powered on (0.06%)
    //   1: this is the usual value (55% of the time in LeafLogs)
    //   D: seems to occur for ~90ms when changing gears (0.2%)
    //   E: this also is a usual value, but never occurs with the
    //      non-gears 0 and 1 (44% of the time in LeafLogs)

    outFrame[0] = WantsToCharge() ? 0x01 : 0x4E;
    // 0x40 when car is ON, 0x80 when OFF, 0x50 when ECO
    outFrame[1] = WantsToCharge() ? 0x80 : 0x40;

    // Usually 0x00, sometimes 0x80 (LeafLogs), 0x04 seen by canmsgs
    outFrame[2] = 0x00;

    // Weird value at D3:4 that goes along with the counter
    // NOTE: Not actually needed, you can just send constant AA C0
    const static byte weird_d34_values[4][2] = {
        {0xaa, 0xc0},
        {0x55, 0x00},
        {0x55, 0x40},
        {0xaa, 0x80},
    };
    outFrame[3] = weird_d34_values[counter_11a_d6][0];
    outFrame[4] = weird_d34_values[counter_11a_d6][1];

    // Always 0x00 (LeafLogs, canmsgs)
    outFrame[5] = 0x00;

    // A 2-bit counter
    outFrame[6] = counter_11a_d6;

    counter_11a_d6++;
    if (counter_11a_d6 >= 4)
        counter_11a_d6 = 0;

    // Extra CRC
    NissanCRC(outFrame);

    can->Transmit(MsgID::CmdGearSelection, 8, outFrame);

    // Send target motor torque signal
    // Data taken from a gen1 inFrame where the car is starting to
    // move at about 10% throttle: F70700E0C74430D4

    // Usually F7, but can have values between 9A...F7 (gen1); 2016: 6E
    outFrame[0] = gen2Codes ? 0x6E : 0xF7;
    // Usually 07, but can have values between 07...70 (gen1); 2016: 6E
    outFrame[1] = gen2Codes ? 0x6E : 0x07;

    // Requested torque (signed 12-bit value + always 0x0 in low nibble)
    if (final_torque_request >= -2048 && final_torque_request <= 2047)
    {
        outFrame[2] = ((final_torque_request < 0) ? 0x80 : 0) | ((final_torque_request >> 4) & 0x7f);
        outFrame[3] = (final_torque_request << 4) & 0xf0;
    }
    else
    {
        outFrame[2] = 0x00;
        outFrame[3] = 0x00;
    }

    // MSB nibble: Runs through the sequence 0, 4, 8, C
    // LSB nibble: Precharge report (precedes actual precharge
    //             control)
    //   0: Discharging (5%)
    //   2: Precharge not started (1.4%)
    //   3: Precharging (0.4%)
    //   5: Starting discharge (3x10ms) (2.0%)
    //   7: Precharged (93%)
    outFrame[4] = 0x07 | (counter_1d4 << 6);

    counter_1d4++;
    if (counter_1d4 >= 4)
        counter_1d4 = 0;

    // MSB nibble:
    //   0: 35-40ms at startup when gear is 0, then at shutdown 40ms
    //      after the car has been shut off (6% total)
    //   4: Otherwise (94%)
    // LSB nibble:
    //   0: ~100ms when changing gear, along with 11A D0 b3:0 value
    //      D (0.3%)
    //   2: Reverse gear related (13%)
    //   4: Forward gear related (21%)
    //   6: Occurs always when gear 11A D0 is 01 or 11 (66%)
    // outFrame[5] = 0x44;
    // outFrame[5] = 0x46;

    // 2016 drive cycle: 06, 46, precharge, 44, drive, 46, discharge, 06
    // 0x46 requires ~25 torque to start
    // outFrame[5] = 0x46;
    // 0x44 requires ~8 torque to start
    outFrame[5] = 0x44;

    // MSB nibble:
    //   In a drive cycle, this slowly changes between values (gen1):
    //     leaf_on_off.txt:
    //       5 7 3 2 0 1 3 7
    //     leaf_on_rev_off.txt:
    //       5 7 3 2 0 6
    //     leaf_on_Dx3.txt:
    //       5 7 3 2 0 2 3 2 0 2 3 2 0 2 3 7
    //     leaf_on_stat_DRDRDR.txt:
    //       0 1 3 7
    //     leaf_on_Driveincircle_off.txt:
    //       5 3 2 0 8 B 3 2 0 8 A B 3 2 0 8 A B A 8 0 2 3 7
    //     leaf_on_wotind_off.txt:
    //       3 2 0 8 A B 3 7
    //     leaf_on_wotinr_off.txt:
    //       5 7 3 2 0 8 A B 3 7
    //     leaf_ac_charge.txt:
    //       4 6 E 6
    //   Possibly some kind of control flags, try to figure out
    //   using:
    //     grep 000001D4 leaf_on_wotind_off.txt | cut -d' ' -f10 | uniq |
    //     ~/projects/leaf_tools/util/hex_to_ascii_binary.py
    //   2016:
    //     Has different values!
    // LSB nibble:
    //   0: Always (gen1)
    //   1:  (2016)

    // 2016 drive cycle:
    //   E0: to 0.15s
    //   E1: 2 messages
    //   61: to 2.06s (inverter is powered up and precharge
    //                 starts and completes during this)
    //   21: to 13.9s
    //   01: to 17.9s
    //   81: to 19.5s
    //   A1: to 26.8s
    //   21: to 31.0s
    //   01: to 33.9s
    //   81: to 48.8s
    //   A1: to 53.0s
    //   21: to 55.5s
    //   61: 2 messages
    //   60: to 55.9s
    //   E0: to end of capture (discharge starts during this)

    // 0x1d4 byte 6 seems to have an active role.
    // 0x00 when in park and brake released.
    // 0x20 when brake lightly pressed in park.
    // 0x30 when brake heavilly pressed in park.
    // 0xE0 when charging

    // Value chosen from a 2016 log
    // outFrame[6] = 0x61;

    // Value chosen from a 2016 log
    // 2016-24kWh-ev-on-drive-park-off.pcap #12101 / 15.63s
    // outFrame[6] = 0x01;
    // byte 6 brake signal
    outFrame[6] = WantsToCharge() ? 0xE0 : (gen2Codes ? 0x01 : 0x30);

    // Extra CRC
    NissanCRC(outFrame);

    can->Transmit(MsgID::CmdTorque, 8, outFrame);


    // We need to send 0x1db here with voltage measured by inverter
    int TMP_battI = inverterStatus.stats.motorPower * 2000.0f /
                      (float)inverterStatus.inverterVoltage; //(Param::Get(Param::idc)) * 2;
    short TMP_battV = inverterStatus.inverterVoltage * 4;    //(Param::Get(Param::udc)) * 4;

    outFrame[0] = TMP_battI >> 3;   // MSB current. 11 bit signed MSBit first
    outFrame[1] = (TMP_battI & 0x07) << 5; // LSB current bits 7-5. Dont need to mess with bits 0-4 for now as 0 works.
    outFrame[2] = TMP_battV >> 2;
    outFrame[3] = ((TMP_battV & 0x07) << 6) | 0x2b; // 0x2b should give no cut req, main rly on permission,normal p limit.
    outFrame[4] = 0x40;                          // SOC for dash in Leaf. fixed val.
    outFrame[5] = 0x00;
    outFrame[6] = counter_1db;

    counter_1db++;
    if (counter_1db >= 4)
        counter_1db = 0;

    NissanCRC(outFrame);
    can->Transmit(MsgID::RcvBatteryState, 8, outFrame);

    OBCMsgs10Ms(can);
}

bool OBDDataPending;
void MsgsOBD()
{
    OBDDataPending = false;
    
    for(int i = 0; i < dummyQueries.size(); i++)
    {
        if(!dummyQueries[i].needsToRespond)
            continue;

        dummyQueries[i].needsToRespond = false;
        can->Transmit(dummyQueries[i].responseID, 8, (uint8_t*)&dummyQueries[i].responseData);
    }
}

void Msgs100ms()
{
    if (!can_status)
        return;

    SendHeartBeat();
    OBCMsgs100Ms(can);
}

bool printThrottle;
void ToggleThrottlePrint()
{
    printThrottle = !printThrottle;
}

void ReadPedals()
{
    float normalizedThrottle = throttleManager.GetNormalizedThrottle();
    final_torque_request = 0;

    if (torqueRequestOverride != 0)
        final_torque_request = torqueRequestOverride;
    else if (driveMode && prechargeComplete)
    {
        if (normalizedThrottle < 0.01f && regenEnabled)
        {
            float normalizedRegen =
                (float)(inverterStatus.stats.rpm - minRegenRPM) / (float)(maxRegenRPM - minRegenRPM);
            normalizedRegen = max(0, min(1, normalizedRegen));
            final_torque_request = -regenSmoothTorque * normalizedRegen;
            regenSmoothTorque = max(0, min(regenSmoothTorque + regenTorque * deltaTime / regenSmoothTorqueTime, regenTorque));
        }
        else
        {
            final_torque_request = MaxTorque * normalizedThrottle;
            regenSmoothTorque = 0;
        }
    }
}

static float FahrenheitToCelsius(unsigned char fahrenheitRawValue)
{
    float temperature = (float)fahrenheitRawValue;
    temperature -= 32;
    temperature *= 0.555555f;
    return temperature;
}

can_frame recvFrame;
void ReadCAN()
{
    if (can == nullptr || !can->GetCanData(recvFrame))
        return;

    byte inFrame[8];
    memcpy(&inFrame, recvFrame.data, min(recvFrame.can_dlc, 8));

    if (!IsCanIDValid(recvFrame.can_id))
    {
        if(OBCHandleCAN(recvFrame.can_id, recvFrame.can_dlc, inFrame))
            return;

        for(int i = 0; i < dummyQueries.size(); i++)
        {
            if(dummyQueries[i].queryID == recvFrame.can_id)
            {
                OBDDataPending = true;
                dummyQueries[i].needsToRespond = true;
                return;
            }
        }

        PrintSerialMessage("Unknown CAN message ID: 0x" + IntToHex(recvFrame.can_id) +
                           " Bytes: " + BytesToString(recvFrame.data, recvFrame.can_dlc));
        return;
    }

    MsgID messageType = (MsgID)recvFrame.can_id;

    // Check data length
    switch (messageType)
    {
    case MsgID::RcvInverterState:
    case MsgID::RcvTempF:
        if (recvFrame.can_dlc != 8)
        {
            PrintSerialMessage("Invalid CAN data length for " + IntToHex(recvFrame.can_id) + ". Expected 8, got: ",
                               recvFrame.can_dlc);
            return;
        }
        break;

    default:
        PrintSerialMessageHEX("Received my own command?? ID: ", recvFrame.can_id);
        return;
    }

    short torque, rpm;
    byte plugVoltageMode;

    // Handle CAN message
    switch (messageType)
    {
    case MsgID::RcvInverterState:
        inverterStatus.inverterVoltage = (((ushort)inFrame[0] << 2) | (((ushort)inFrame[1]) >> 6)) / 2;

        torque = (short)((inFrame[2] & 0x07) << 8 | inFrame[3]);
        if ((inFrame[2] & 0x04) == 0x04) // indicates negative value
            torque = torque | 0xf800;    // pad leading 1s for 2s complement signed
        inverterStatus.stats.motorTorque = torque / 2;
        maxStats.motorTorque = max(maxStats.motorTorque, inverterStatus.stats.motorTorque);

        rpm = (short)(inFrame[4] << 8 | inFrame[5]);
        if ((inFrame[4] & 0x40) == 0x40) // indicates negative value
            rpm = rpm | 0x8000;          // pad leading 1s for 2s complement signed
        inverterStatus.stats.rpm = rpm / 2;
        maxStats.rpm = max(maxStats.rpm, inverterStatus.stats.rpm);

        // torque (Nm) to power (W) = 2 x pi / 60 * rpm * torque
        inverterStatus.stats.motorPower = rpm * torque / 9548.8f;
        maxStats.motorPower = max(maxStats.motorPower, inverterStatus.stats.motorPower);

        inverterStatus.error_state = (inFrame[6] & 0xb0) != 0x00;
        break;

    case MsgID::RcvTempF:
        inverterStatus.stats.motor_temperature = FahrenheitToCelsius(inFrame[1]);
        inverterStatus.stats.inverter_temperature = FahrenheitToCelsius(inFrame[2]);
        maxStats.motor_temperature = max(maxStats.motor_temperature, inverterStatus.stats.motor_temperature);
        maxStats.inverter_temperature = max(maxStats.inverter_temperature, inverterStatus.stats.inverter_temperature);
        break;

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
    if (printThrottle && throttlePrintTimer.HasTriggered())
        PrintSerialMessage(String(throttleManager.GetNormalizedThrottle()));
}

void ControlVacuum()
{
    float rawValue = vacuumSensor.GetRawValue();
    if(rawValue < 0.01f || vacuumPumpFailure || !ignitionOn || ChargerStatusPlugInserted()) // Disconnected or failed or charging mode
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
        can = new CAN(MKRCAN_MCP2515_CS_PIN, MKRCAN_MCP2515_INT_PIN);

    throttleManager.Tick();
    vacuumSensor.Tick();

    ReadPedals();
    ControlVacuum();

    CheckIgnition();
    CheckDriveMode();

    if (TimerHV.HasTriggered())
    {
        HighVoltageControl();
        HVChargeControl();
        TemperatureControl();
    }

    ReadCAN();

    if (timer_Frames10.HasTriggered())
        Msgs10ms();
    if (timer_Frames100.HasTriggered())
        Msgs100ms();
    if (OBDDataPending && timer_response_50MS.HasTriggered())
        MsgsOBD();

    PrintFailures();
    PrintDebug();
}
} // namespace VCU