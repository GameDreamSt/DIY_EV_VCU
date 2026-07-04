
#include "LeafInverter.h"

#include "EVLib/SerialPrint.h"
#include "EVLib/SerialReader.h"
#include "EVLib/CAN.h"
#include "EVLib/MathUtils.h"

#include "VCU.h"

#include <cstdint>

using namespace std;

bool gen2Codes = true;

LeafInverter::Stats stats;
LeafInverter::Stats LeafInverter::GetStats() { return stats; }
void LeafInverter::ClearStats()
{
    stats = LeafInverter::Stats();
}

#define MAX_TORQUE 1120 // Max torque request is 1120 for 80kw inverter
short final_torque_request = 0;

void LeafInverter::SetTorqueNm(int value)
{
    int maxNm = MAX_TORQUE / 4;
    value = Clamp(value, -maxNm, maxNm);

    float valueRatio = value / (float)maxNm;

    final_torque_request = MAX_TORQUE * valueRatio;
}

void PrintInverterStatus()
{
    /*string str = "Inverter voltage: " + ToString(status.inverterVoltage) + " V " + 
                "\n" + stats.GetString() +
                "\nIs in error state: " + BoolToString(status.error_state);*/

    string str = stats.GetString();
    PrintSerialMessage(str);
}

void ToggleGen2Codes()
{
    gen2Codes = !gen2Codes;

    if (gen2Codes)
        PrintSerialMessage("Gen 2 CAN codes selected");
    else
        PrintSerialMessage("Gen 1 CAN codes selected");
}

void LeafInverter::Initialize()
{
    AddCommand("status", PrintInverterStatus);
    AddCommand("togglegen2", ToggleGen2Codes);
}

enum MsgID
{
    CmdHeartBeat = 0x50B,
    CmdGearSelection = 0x11A,
    CmdTorque = 0x1D4,
    RcvInverterState = 0x1DA,
    RcvTempF = 0x55A,
};

string LeafInverter::Stats::GetString()
{
    return "RPM: " + ToString(rpm) + "\nMotor torque: " + ToString(motorTorque) + " nm" +
           "\nMotor power: " + FloatToString(motorPower, 1) + " kw" +
           "\nInverter temperature: " + FloatToString(inverter_temperature, 2) + " C " +
           "\nMotor temperature: " + FloatToString(motor_temperature, 2) + " C ";
}

bool IsCanIDValid(int ID)
{
    switch (ID)
    {
    case 0x50B:
    case 0x11A:
    case 0x1D4:
    case 0x1DA:
    case 0x55A:
        return true;
    }

    return false;
}

void NissanCRC(uint8_t *data)
{
    uint8_t polynomial = 0x85;

    // We want to process 8 bytes with the 8th byte being zero
    data[7] = 0;
    uint8_t crc = 0;
    for (int b = 0; b < 8; b++)
    {
        for (int i = 7; i >= 0; i--)
        {
            uint8_t bit = ((data[b] & (1 << i)) > 0) ? 1 : 0;
            if (crc >= 0x80)
                crc = (uint8_t)(((crc << 1) + bit) ^ polynomial);
            else
                crc = (uint8_t)((crc << 1) + bit);
        }
    }
    data[7] = crc;
}

void SendHeartBeat(CAN *can)
{
    // Statistics from 2016 capture:
    //     10 00000000000000
    //     21 000002c0000000
    //    122 000000c0000000
    //    513 000006c0000000

    uint8_t outFrame[7]{};
    outFrame[0] = 0x00;
    outFrame[1] = 0x00;
    outFrame[2] = 0x06;
    outFrame[3] = 0xc0;
    outFrame[4] = 0x00;
    outFrame[5] = 0x00;
    outFrame[6] = 0x00;

    can->Transmit((int)MsgID::CmdHeartBeat, 7, outFrame);
}

void LeafInverter::Msgs10ms(CAN *can)
{
    uint8_t outFrame[8]{};
    static uint8_t counter_11a_d6 = 0;
    static uint8_t counter_1d4 = 0;
    static uint8_t counter_1db = 0;

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

    outFrame[0] = VCU::WantsToCharge() ? 0x01 : 0x4E;
    // 0x40 when car is ON, 0x80 when OFF, 0x50 when ECO
    outFrame[1] = VCU::WantsToCharge() ? 0x80 : 0x40;

    // Usually 0x00, sometimes 0x80 (LeafLogs), 0x04 seen by canmsgs
    outFrame[2] = 0x00;

    // Weird value at D3:4 that goes along with the counter
    // NOTE: Not actually needed, you can just send constant AA C0
    const static uint8_t weird_d34_values[4][2] = {
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
    outFrame[6] = VCU::WantsToCharge() ? 0xE0 : (gen2Codes ? 0x01 : 0x30);

    // Extra CRC
    NissanCRC(outFrame);

    can->Transmit(MsgID::CmdTorque, 8, outFrame);

    // We need to send 0x1db here with voltage measured by inverter
    /*int TMP_battI = stats.motorPower * 2000.0f /
                      (float)stats.inverterVoltage; //(Param::Get(Param::idc)) * 2;
    short TMP_battV = stats.inverterVoltage * 4;    //(Param::Get(Param::udc)) * 4;

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
    can->Transmit(MsgID::RcvBatteryState, 8, outFrame);*/
}

void LeafInverter::Msgs100ms(CAN *can)
{
    SendHeartBeat(can);
}


static float FahrenheitToCelsius(unsigned char fahrenheitRawValue)
{
    float temperature = (float)fahrenheitRawValue;
    temperature -= 32;
    temperature *= 0.555555f;
    return temperature;
}

bool LeafInverter::HandleCAN(unsigned int canID, int inFrameSize, unsigned char inFrame[8])
{
    if (!IsCanIDValid(canID))
    {
        return false;
    }

    MsgID messageType = (MsgID)canID;

    // Check data length
    switch (messageType)
    {
    case MsgID::RcvInverterState:
    case MsgID::RcvTempF:
        if (inFrameSize != 8)
        {
            PrintSerialMessage("Invalid CAN data length for " + IntToHex(canID) + ". Expected 8, got: ", inFrameSize);
            return false;
        }
        break;

    default:
        PrintSerialMessageHEX("Received my own command?? ID: ", canID);
        return false;
    }

    int16_t torque, rpm;

    // Handle CAN message
    switch (messageType)
    {
    case MsgID::RcvInverterState:
        stats.inverterVoltage = (((uint16_t)inFrame[0] << 2) | (((uint16_t)inFrame[1]) >> 6)) / 2;

        torque = (int16_t)((inFrame[2] & 0x07) << 8 | inFrame[3]);
        if ((inFrame[2] & 0x04) == 0x04) // Indicates negative value
            torque = torque | 0xf800;    // Pad leading 1s for 2s complement signed
        stats.motorTorque = torque / 2;

        rpm = (int16_t)(inFrame[4] << 8 | inFrame[5]);
        if ((inFrame[4] & 0x40) == 0x40) // Indicates negative value
            rpm = rpm | 0x8000;          // Pad leading 1s for 2s complement signed
        stats.rpm = rpm / 2;

        // torque (Nm) to power (W) = 2 x pi / 60 * rpm * torque
        stats.motorPower = rpm * torque / 9548.8f;

        stats.error_state = (inFrame[6] & 0xb0) != 0x00;
        break;

    case MsgID::RcvTempF:
        stats.motor_temperature = FahrenheitToCelsius(inFrame[1]);
        stats.inverter_temperature = FahrenheitToCelsius(inFrame[2]);
        break;

    default:
        break;
    }

    return true;
}