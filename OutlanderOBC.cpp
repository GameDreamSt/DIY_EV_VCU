
#include "OutlanderOBC.h"

#include "CAN.h"
#include "SerialPrint.h"

unsigned char outFrame[8];
unsigned short HVTargetVoltage;
unsigned char chargingCurrent;
CmdChargeStatus chargeStatus;

DCDC_Data dcdc_Data;
OBC_Data obc_Data;

DCDC_Data* GetDCDCData() { return &dcdc_Data; }
OBC_Data* GetOBCData() { return &obc_Data; }

short DCDC_Data::MaxTemperature()
{
    short temp = -40;
    for(int i = 0; i < 3; i++)
        if(temperature[i] > temp)
            temp = temperature[i];
    return temp;
}

String DCDC_Data::GetString()
{
    return "Battery voltage: " + FloatToString(supplyVoltage, 2) + " V" +
    "\nBattery charge current: " + FloatToString(supplyCurrent, 1) + " A" + 
    "\nIs ready: " + BoolToString(ready) +
    "\nIs charging: " + BoolToString(operating) +
    "\nIs in error state: " + BoolToString(error) + 
    "\nTemperature: " + ToString(MaxTemperature()) + " C";
}

short OBC_Data::MaxTemperature()
{
    short temp = -40;
    for(int i = 0; i < 4; i++)
        if(temperature[i] > temp)
            temp = temperature[i];
    return temp;
}

String OBC_Data::GetString()
{
    return "Traction battery voltage: " + ToString(HV_Voltage) + " V" +
    "\nTraction battery voltage2: " + ToString(HV_Voltage2) + " V" +
    "\nTraction battery charge current: " + FloatToString(HV_Current, 1) + " A" +
    "\nAC voltage detected: " + BoolToString(ACVoltageDetected) +
    "\nAC voltage: " + ToString(AC_Voltage) + " V" + 
    "\nIs Control Pilot detected: " + BoolToString(controlPilotDetected) +
    "\nControl Pilot duty cycle: " + ToString(controlPilotDutyCyclePercentage) + " %"
    "\nDC/DC status requested: " + BoolToString(DCDC_Status_Requested) + 
    "\nIs in error state: " + BoolToString(error) +
    "\nIs charging: " + BoolToString(charging) +
    "\nTemperature: " + ToString(MaxTemperature()) + " C";
}

void SetChargeStatus(CmdChargeStatus status, float OBC_HVTargetVoltage, unsigned char OBC_ChargingCurrent)
{
    chargeStatus = status;
    OBC_HVTargetVoltage = max(min(OBC_HVTargetVoltage, 410), 0);
    HVTargetVoltage = OBC_HVTargetVoltage * 10;
    chargingCurrent = min(OBC_ChargingCurrent, 12); // 12A is the maximum charging current
}

enum MsgID
{
    CmdHeartBeat = 0x285,
    CmdOBC_Control = 0x286,

    RcvOBC_DCDC_Status = 0x377,
    RcvOBC_Status1 = 0x389,
    RcvOBC_Status2 = 0x38A,
};

bool IsCanIDValid(unsigned int ID)
{
    switch (ID)
    {
    case 0x285:
    case 0x286:
    case 0x277:
    case 0x389:
    case 0x38A:
        return true;
    }

    return false;
}

void ClearCAN()
{
    unsigned long long zero = 0;
    memcpy(&outFrame, &zero, 8);
}

void OBCMsgs10Ms(CAN *can)
{
    if (chargeStatus == CmdChargeStatus::Off)
        return;

    ClearCAN();
    can->Transmit((int)MsgID::CmdHeartBeat, 8, outFrame);
}

void OBCMsgs100Ms(CAN *can)
{
    if (chargeStatus != CmdChargeStatus::Charge)
    {
        ClearCAN();
        outFrame[6] = 0xB6;
        can->Transmit((int)MsgID::CmdOBC_Control, 8, outFrame);
        return;
    }

    ClearCAN();

    outFrame[0] = HVTargetVoltage; // (Big Endian bytes e.g. 0x0E 0x74 = 3700 = 370v => 0x74 0x0E to CAN)
    outFrame[1] = HVTargetVoltage >> 8;
    outFrame[2] = chargingCurrent;
    outFrame[6] = 0xB6;
    can->Transmit((int)MsgID::CmdOBC_Control, 8, outFrame);
}

bool PrintCANDataLengthFailure(int ID, int expected, int got)
{
    PrintSerialMessage("Invalid CAN data length for " + IntToHex(ID) + ". Expected: " + ToString(expected) + ", got: ", got);
    return true;
}

bool OBCHandleCAN(unsigned int canID, int inFrameSize, unsigned char inFrame[8])
{
    if (!IsCanIDValid(canID))
    {
        return false;
    }

    MsgID messageType = (MsgID)canID;

    bool previousCPState = obc_Data.controlPilotDetected;

    // Handle CAN message
    switch (messageType)
    {
    case MsgID::RcvOBC_DCDC_Status:
        if(inFrameSize < 8)
            return PrintCANDataLengthFailure(canID, 8, inFrameSize);
        dcdc_Data.supplyVoltage = (inFrame[0] << 8) | inFrame[1];
        dcdc_Data.supplyVoltage *= 0.01f;

        dcdc_Data.supplyCurrent = (inFrame[2] << 8) | inFrame[3];
        dcdc_Data.supplyCurrent *= 0.1f;

        dcdc_Data.temperature[0] = inFrame[4] - 40;
        dcdc_Data.temperature[1] = inFrame[5] - 40;
        dcdc_Data.temperature[2] = inFrame[6] - 40;

        dcdc_Data.ready = (inFrame[7] >> 5) & 1;
        dcdc_Data.operating = (inFrame[7] >> 1) & 1;
        dcdc_Data.operating = inFrame[7] & 1;
        break;

    case MsgID::RcvOBC_Status1:
        if(inFrameSize < 6)
            return PrintCANDataLengthFailure(canID, 6, inFrameSize);
        obc_Data.HV_Voltage = inFrame[0] * 2;
        obc_Data.AC_Voltage = inFrame[1];

        obc_Data.HV_Current = inFrame[2] * 0.1f;

        obc_Data.temperature[0] = inFrame[3] - 40;
        obc_Data.temperature[1] = inFrame[4] - 40;

        obc_Data.ACVoltageDetected = (inFrame[5] >> 1) & 1;
        obc_Data.charging = (inFrame[5] >> 3) & 1;
        obc_Data.error = (inFrame[5] >> 4) & 1;
        obc_Data.DCDC_Status_Requested = (inFrame[5] >> 6) & 1;
        obc_Data.controlPilotDetected = (inFrame[5] >> 7) & 1;

        if (previousCPState != obc_Data.controlPilotDetected)
        {
            if (obc_Data.controlPilotDetected)
                PrintSerialMessage("Charging plug inserted");
            else
                PrintSerialMessage("Charging plug disconnected");
        }
        break;

    case MsgID::RcvOBC_Status2:
        if(inFrameSize < 4)
            return PrintCANDataLengthFailure(canID, 4, inFrameSize);
        obc_Data.temperature[2] = inFrame[0] - 40;
        obc_Data.temperature[3] = inFrame[1] - 40;

        obc_Data.HV_Voltage2 = inFrame[2] * 2;
        obc_Data.controlPilotDutyCyclePercentage = inFrame[3];
        break;

    default:
        break;
    }

    return true;
}