
#include "VCU.h"

#include <Arduino.h>

#include "CAN.h"
#include "Contactor.h"
#include "SerialPrint.h"
#include "Throttle.h"
#include "Timer.h"
#include "mcp2515.h"

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
byte *inFrame = new byte[8];

#define HVTimerTime 0.2f

Timer timer_Frames10 = Timer(0.01f);
Timer timer_Frames100 = Timer(0.1f);
Timer TimerHV = Timer(HVTimerTime);
Timer prechargeFailureTimer = Timer(3);
Timer throttlePrintTimer = Timer(0.1f);

ThrottleManager throttleManager = ThrottleManager();

#define LOWEST_VOLTAGE 210
#define MAX_CHARGE_VOLTAGE 249
float powerLimitOut = 3; // In kw
float powerLimitIn = 0;  // In kw (from regen braking)
float chargingLimit = 0; // In kw (from plug)

float OBC_AC_Voltage = 0;
float OBCActivePower = 0;
float OBCAvailablePower = 0;

ushort ThrotVal = 0; // analog value of throttle position.

#define MAX_TORQUE 1120 // Max torque request is 1120
short MaxTorque = 650;
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
    if(value > MAX_TORQUE)
    {
        PrintSerialMessage("Torque requested is above max torque of " + ToString(MAX_TORQUE) + "!");
        return false;
    }

    MaxTorque = value;
    return true;
}

void SetBatteryDischargeLimit(float kilowatts)
{
    if (kilowatts < 0)
        kilowatts = 0;
    powerLimitOut = kilowatts;
}

bool ignitionOn;
bool driveMode;

bool can_status;
bool plugInserted;

bool gen2Codes = true;
bool ToggleGen2Codes()
{
    gen2Codes = !gen2Codes;
    return gen2Codes;
}

// For PDM failsafe testing.
bool PDM_CAN_Enabled = true;
bool TogglePDMCAN()
{
    PDM_CAN_Enabled = !PDM_CAN_Enabled;
    return PDM_CAN_Enabled;
}

InverterStatus inverterStatus;
InverterStatus GetInverterStatus()
{
    return inverterStatus;
}

enum MsgID
{
    // Inverter IDs
    CmdHeartBeat = 0x50B,
    CmdGearSelection = 0x11A,
    CmdBatteryState = 0x1DB, // Also for PDM
    CmdTorque = 0x1D4,
    RcvInverterState = 0x1DA,
    RcvTempF = 0x55A,

    // PDM
    CmdPowerLimits = 0x1DC,
    CmdDCToDC = 0x1F2,
    CmdSOC = 0x55B,
    CmdBatteryCapacity = 0x59E,
    CmdChargeStatus = 0x5BC,
    RcvPlugStatus = 0x390,
    RcvPlugInsert = 0x679,

    RcvPDMModel_AZE0_2014_2017 = 0x393,
    RcvPDMModel_ZE1_2018 = 0x1ED,
};

bool IsCanIDValid(int ID)
{
    switch (ID)
    {
    case 0x50B:
    case 0x11A:
    case 0x1DB:
    case 0x1D4:
    case 0x1DA:
    case 0x55A:
    case 0x1DC:
    case 0x1F2:
    case 0x55B:
    case 0x59E:
    case 0x5BC:
    case 0x390:
    case 0x679:
    case 0x393:
    case 0x1ED:
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

ContactorTest contactorTest;

void SetContactor(int pinID, bool state)
{
    switch(pinID)
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

        default:
        break;
    }
}

void SetContactorForTesting(int value)
{
    ContactorTest enumValue = (ContactorTest)value;

    if(enumValue != ContactorTest::None)
        SetContactor((int)contactorTest, false);

    contactorTest = enumValue;

    if(enumValue != ContactorTest::None)
        SetContactor((int)contactorTest, true);
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

    SetContactor(PIN_PRECHARGE, false);
    SetContactor(PIN_POS_CONTACTOR, false);
    SetContactor(PIN_NEG_CONTACTOR, false);

    throttleManager.AddThrottle(Throttle(APIN_Throttle1));
    throttleManager.AddThrottle(Throttle(APIN_Throttle2));
}

float prechargeToFailureTime = 0;

bool prechargeFailure = false;
bool prechargeComplete = false;

float lastInverterVoltage;
float lastInverterVoltageTime;

void ClearHVData()
{
    lastInverterVoltageTime = lastInverterVoltage = prechargeToFailureTime = 0;
    if (!ignitionOn)
        prechargeFailure = prechargeComplete = false;
    inverterStatus.inverterVoltage = 0;
}

bool IsIgnitionOn()
{
    return ignitionOn;
}

void CheckIgnition()
{
    if (!digitalRead(PIN_IGNITION)) // PIN ON : INPUT_PULLUP
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
    if (ignitionOn && !digitalRead(PIN_DRIVE_MODE)) // PIN ON : INPUT_PULLUP
    {
        if (!driveMode)
            PrintSerialMessage("Drive mode on");
        driveMode = true;
    }
    else if(!ignitionOn)
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
    if (!timer_Frames10.HasTriggered() || !can_status)
        return;

    static byte counter_11a_d6 = 0;
    static byte counter_1d4 = 0;
    static byte counter_1db = 0;
    static byte counter_329 = 0;
    static byte counter_100ms = 0;

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

    outFrame[0] = 0x4E;
    // outFrame[0] = 0x01;

    // 0x40 when car is ON, 0x80 when OFF, 0x50 when ECO
    outFrame[1] = 0x40;

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
    // outFrame[4] = 0x02 | (counter_1d4 << 6);

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

    // This value has been chosen at the end of the hardest
    // acceleration in the wide-open-throttle pull, with full-ish
    // torque still being requested, in
    //   LeafLogs/leaf_on_wotind_off.txt
    // outFrame[6] = 0x00;

    // 0x1d4 byte 6 seems to have an active role.
    // 0x00 when in park and brake released.
    // 0x20 when brake lightly pressed in park.
    // 0x30 when brake heavilly pressed in park.
    outFrame[6] = gen2Codes ? 0x01 : 0x30;

    // Value chosen from a 2016 log
    // outFrame[6] = 0x61;

    // Value chosen from a 2016 log
    // 2016-24kWh-ev-on-drive-park-off.pcap #12101 / 15.63s
    // outFrame[6] = 0x01;
    // byte 6 brake signal

    // Extra CRC
    NissanCRC(outFrame);

    /*Serial.print(F("Sending "));
    print_fancy_inFrame(inFrame);
    Serial.println();*/

    can->Transmit(MsgID::CmdTorque, 8, outFrame);

    // We need to send 0x1db here with voltage measured by inverter
    short TMP_battI =
        inverterStatus.motorPower * 2000.0f / (float)inverterStatus.inverterVoltage; //(Param::Get(Param::idc)) * 2;
    short TMP_battV = inverterStatus.inverterVoltage * 4;                            //(Param::Get(Param::udc)) * 4;

    outFrame[0] = TMP_battI >> 8;   // MSB current. 11 bit signed MSBit first
    outFrame[1] = TMP_battI & 0xE0; // LSB current bits 7-5. Dont need to mess with bits 0-4 for now as 0 works.
    outFrame[2] = TMP_battV >> 8;
    outFrame[3] = ((TMP_battV & 0xC0) | (0x2b)); // 0x2b should give no cut req, main rly on permission,normal p limit.
    outFrame[4] = 0x40;                          // SOC for dash in Leaf. fixed val.
    outFrame[5] = 0x00;
    outFrame[6] = counter_1db;

    counter_1db++;
    if (counter_1db >= 4)
        counter_1db = 0;

    NissanCRC(outFrame);
    can->Transmit(MsgID::CmdBatteryState, 8, outFrame);

    /////////////////////////////////////////////////////////////////////////////////////////////////
    // 0x1dc from lbc. Contains chg power lims and disch power lims.
    // Disch power lim in byte 0 and byte 1 bits 6-7. Just set to max for now.
    // Max charging power in bits 13-20. 10 bit unsigned scale 0.25.Byte 1 limit in kw.

    static byte counter_1dc = 0;

    ushort tmp_powerLimitOut = powerLimitOut * 4;  // X
    ushort tmp_powerLimitIn = powerLimitIn * 4;    // Y
    ushort tmp_chargingLimit = chargingLimit * 10; // Z

    outFrame[0] = tmp_powerLimitOut >> 2;                         // 00XX XXXX
    outFrame[1] = tmp_powerLimitOut << 6 | tmp_powerLimitIn >> 2; // XXYY YYYY
    outFrame[2] = tmp_powerLimitIn << 4 | tmp_chargingLimit >> 6; // YYYY 00ZZ
    outFrame[3] = tmp_chargingLimit << 2;                         // ZZZZ ZZ00
    outFrame[4] = 0x00;
    outFrame[5] = 0x00;
    outFrame[6] = counter_1dc;

    // Extra CRC in byte 7
    NissanCRC(outFrame);

    counter_1dc++;
    if (counter_1dc >= 4)
        counter_1dc = 0;

    if(PDM_CAN_Enabled)
        can->Transmit(MsgID::CmdPowerLimits, 8, outFrame);

    /////////////////////////////////////////////////////////////////////////////////////////////////
    // CAN Message 0x1F2: Charge Power and DC/DC Converter Control
    // convert power setpoint to PDM format:
    //    0x70 = 3 amps ish
    //    0x6a = 1.4A
    //    0x66 = 0.5A
    //    0x65 = 0.3A
    //    0x64 = no chg
    //    so 0x64=100. 0xA0=160. so 60 decimal steps. 1 step=100W???
    /////////////////////////////////////////////////////////////////////////////////////////////////////

    // get actual voltage and voltage setpoints
    int calcBMSpwr = chargingLimit * 1000; // BMS charge current limit but needs to be power for most AC charger types.

    static byte OBCpwr = 0;
    byte OBCpwrSP = (min(MAX_CHARGE_VOLTAGE, calcBMSpwr) / 100) + 0x64;

    if (!prechargeComplete)
    {
        // clamp min and max values
        if (OBCpwrSP > 0xA0)
            OBCpwrSP = 0xA0;
        else if (OBCpwrSP < 0x64)
            OBCpwrSP = 0x64;

        // if measured vbatt is less than setpoint got to max power from web ui
        if (inverterStatus.inverterVoltage < MAX_CHARGE_VOLTAGE)
            OBCpwr = OBCpwrSP;

        // decrement charger power if volt setpoint is reached
        if (inverterStatus.inverterVoltage >= MAX_CHARGE_VOLTAGE)
            OBCpwr--;
    }
    else
    {
        // set power to 0 if charge control is set to off or not in charge mode
        OBCpwr = 0x64;
    }

    static byte counter_1f2 = 0;

    /*
    Byte 2:
    00000b 0x0 = Quick charge
    00101b 0x5 = Normal Charge 6kw Full charge
    01000b 0x8 = Normal Charge 200V Full charge
    01011b 0xB = Normal Charge 100V Full charge
    10010b 0x12 = Normal Charge 6kw long life charge
    10101b 0x15 = Normal Charge 200V long life charge
    11000b 0x18 = Normal Charge 100V long life charge
    11111b 0x1F = Invalid value
    */

    // Commanded chg power in byte 1 and byte 0 bits 0-1. 10 bit number.
    // byte 1=0x64 and byte 0=0x00 at 0 power.
    // 0x00 chg 0ff dcdc on.
    outFrame[0] = 0x30; // msg is muxed but pdm doesn't seem to care.
    outFrame[1] = OBCpwr;
    outFrame[2] = 0x20; // 0x20 = Normal Charge
    outFrame[3] = 0xAC;
    outFrame[4] = 0x00;
    outFrame[5] = 0x3C;
    outFrame[6] = counter_1f2;
    outFrame[7] = 0x8F; // may not need checksum here?

    counter_1f2++;
    if (counter_1f2 >= 4)
        counter_1f2 = 0;

    if(PDM_CAN_Enabled)
        can->Transmit(MsgID::CmdDCToDC, 8, outFrame);

    SendHeartBeat();
}

void Msgs100ms()
{
    if (!timer_Frames100.HasTriggered() || !can_status)
        return;

    /////////////////////////////////////////////////////////////////////////////////////////////////
    // CAN Message 0x55B:

    static byte counter_55b = 0;

    outFrame[0] = 0xA4;
    outFrame[1] = 0x40;
    outFrame[2] = 0xAA;
    outFrame[3] = 0x00;
    outFrame[4] = 0xDF;
    outFrame[5] = 0xC0;
    outFrame[6] = ((0x1 << 4) | (counter_55b));
    // Extra CRC in byte 7
    NissanCRC(outFrame);

    counter_55b++;
    if (counter_55b >= 4)
        counter_55b = 0;

    if(PDM_CAN_Enabled)
        can->Transmit(MsgID::CmdSOC, 8, outFrame);

    /////////////////////////////////////////////////////////////////////////////////////////////////
    // CAN Message 0x59E:

    outFrame[0] = 0x00; // Static msg works fine here
    outFrame[1] = 0x00; // Batt capacity for chg and qc.
    outFrame[2] = 0x0c;
    outFrame[3] = 0x76;
    outFrame[4] = 0x18;
    outFrame[5] = 0x00;
    outFrame[6] = 0x00;
    outFrame[7] = 0x00;

    if(PDM_CAN_Enabled)
        can->Transmit(MsgID::CmdBatteryCapacity, 8, outFrame);

    /////////////////////////////////////////////////////////////////////////////////////////////////
    // CAN Message 0x5BC:

    // muxed msg with info for gids etc. Will try static for a test.
    outFrame[0] = 0x3D; // Static msg works fine here
    outFrame[1] = 0x80;
    outFrame[2] = 0xF0;
    outFrame[3] = 0x64;
    outFrame[4] = 0xB0;
    outFrame[5] = 0x01;
    outFrame[6] = 0x00;
    outFrame[7] = 0x32;

    if(PDM_CAN_Enabled)
        can->Transmit(MsgID::CmdChargeStatus, 8, outFrame);

    SendHeartBeat();
}

bool printThrottle;
void ToggleThrottlePrint()
{
    printThrottle = !printThrottle;
}

void ReadPedals()
{
    float normalizedThrottle = throttleManager.GetNormalizedThrottle();

    if (torqueRequestOverride != 0)
        final_torque_request = torqueRequestOverride;
    else if(driveMode)
        final_torque_request = MaxTorque * normalizedThrottle;
    else
        final_torque_request = 0;
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

    if (!IsCanIDValid(recvFrame.can_id))
    {
        PrintSerialMessage("Unknown CAN message ID: 0x" + IntToHex(recvFrame.can_id) +
                           " Bytes: " + BytesToString(recvFrame.data, recvFrame.can_dlc));
        return;
    }

    MsgID messageType = (MsgID)recvFrame.can_id;

    // Check data length
    switch (messageType)
    {
    case MsgID::RcvPDMModel_AZE0_2014_2017:
    case MsgID::RcvPDMModel_ZE1_2018:
    case MsgID::RcvPlugInsert:
    case MsgID::RcvPlugStatus:
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

    memcpy(inFrame, recvFrame.data, recvFrame.can_dlc);

    short parsed_speed, torque, rpm;
    byte OBCVoltageStatus;

    // Handle CAN message
    switch (messageType)
    {
    case MsgID::RcvInverterState:
        inverterStatus.inverterVoltage = (((ushort)inFrame[0] << 2) | (((ushort)inFrame[1]) >> 6)) / 2;

        torque = (short)((inFrame[2] & 0x07) << 8 | inFrame[3]);
        if ((inFrame[2] & 0x04) == 0x04) // indicates negative value
            torque = torque | 0xf800;    // pad leading 1s for 2s complement signed
        inverterStatus.motorTorque = torque / 2;

        rpm = (short)(inFrame[4] << 8 | inFrame[5]);
        if ((inFrame[4] & 0x40) == 0x40) // indicates negative value
            rpm = rpm | 0x8000;          // pad leading 1s for 2s complement signed
        inverterStatus.rpm = rpm / 2;

        // torque (Nm) to power (W) = 2 x pi / 60 * rpm * torque
        inverterStatus.motorPower = rpm * torque / 9548.8f;

        inverterStatus.error_state = (inFrame[6] & 0xb0) != 0x00;
        break;

    case MsgID::RcvTempF:
        inverterStatus.motor_temperature = FahrenheitToCelsius(inFrame[1]);
        inverterStatus.inverter_temperature = FahrenheitToCelsius(inFrame[2]);
        break;

    case MsgID::RcvPlugStatus:
        OBCVoltageStatus = (inFrame[3] >> 3) & 0x03; // Plug voltage
        OBCActivePower = inFrame[1] * 0.1f; // Power in 0.1kW
        OBCAvailablePower = inFrame[6] * 0.1f; // Power in 0.1kW

        if(OBCVoltageStatus == 0x1)
            OBC_AC_Voltage = 110;
        else if(OBCVoltageStatus == 0x2)
            OBC_AC_Voltage = 230;
        else
            OBC_AC_Voltage = 0;

        if (inFrame[5] & 0x0F == 0x08)
        {
            if (!plugInserted)
                PrintSerialMessage("Charging plug inserted");
            plugInserted = true;
        }
        if (inFrame[5] & 0x0F == 0x00)
        {
            if (plugInserted)
                PrintSerialMessage("Charging plug disconnected");
            plugInserted = false;
        }
        break;

    case MsgID::RcvPDMModel_AZE0_2014_2017:
        inverterStatus.PDMModelType = PDMType::AZE0_2014_2017;
        break;

    case MsgID::RcvPDMModel_ZE1_2018:
        inverterStatus.PDMModelType = PDMType::ZE1_2018;
        break;

    default:
        break;
    }
}

void PrintFailures()
{
    if (prechargeFailure && prechargeFailureTimer.HasTriggered())
        PrintSerialMessage("Precharge failure! Inverter didn't reach battery voltage in 5 seconds!");
}

void PrintDebug()
{
    if (printThrottle && throttlePrintTimer.HasTriggered())
        PrintSerialMessage(String(throttleManager.GetNormalizedThrottle()));
}

void Tick()
{
    if (can == nullptr) // Initialize later so the upload process doesn't hang up
        can = new CAN(MKRCAN_MCP2515_CS_PIN, MKRCAN_MCP2515_INT_PIN);

    throttleManager.Tick();
    ReadPedals();

    CheckIgnition();
    CheckDriveMode();

    if (TimerHV.HasTriggered())
        HighVoltageControl();

    ReadCAN();

    Msgs100ms();
    Msgs10ms();

    PrintFailures();
    PrintDebug();
}
} // namespace VCU