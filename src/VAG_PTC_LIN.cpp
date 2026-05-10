
#include "VAG_PTC_LIN.h"
#include "HardwareSerial.h"
#include "EVLib/LIN_Manager.h"

#include "EVLib/SerialPrint.h"
#include "EVLib/SerialReader.h"

#include <vector>
#include <string>

using namespace std;

LIN_Manager LIN;

enum LIN_Message_ID
{
    VAG_LIN_BOOT = 0x3C,
    VAG_PTC_REQUEST = 0x26,
    VAG_PTC_RECEIVE = 0xD3,
};

bool LIN_Boot_Sent = false;
int counter;
int ptcPowerRequest = 0;

void ToggleLINDebugMode()
{
    LIN.ToggleDebugMode();
}

void SetPTCPower()
{
    auto parameters = *GetParameters();

    if(parameters.size() < 1)
    {
        PrintSerialMessage("Needs a parameter!");
        return;
    }

    ptcPowerRequest = stoi(parameters[0]);

    if(ptcPowerRequest < 0 || ptcPowerRequest > 100)
    {
        ptcPowerRequest = 0;
        PrintSerialMessage("Invalid power parameter!");
    }
    else 
    {
        PrintSerialMessage("Setting power to " + ToString(ptcPowerRequest) + "%");
    }
}

void InitializePTC()
{
    LIN = LIN_Manager(0);

    AddCommand(CommandPointer("lindebug", ToggleLINDebugMode));
    AddCommand(CommandPointer("ptcpower", SetPTCPower));
}

void TickPTC()
{
    LIN.Tick();
    Write_LIN_Data();
    ExtractData();
}

void Write_LIN_Data()
{
    if (!LIN_Boot_Sent)
    {
        LIN_Boot_Sent = true;
        uint8_t bootTx[8] = {0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00};
        PrintSerialMessage("Setting LIN boot message");
        LIN.SetRequest(LIN_Message_ID::VAG_LIN_BOOT, 7, bootTx);
        return;
    }

    uint8_t tx[8] = {0x39, 0xC4, 0x00, 0x00, 0x00, 0xAA, 0x90, 0x01};

    LIN_Message msg;
    if(LIN.TryGetData(LIN_Message_ID::VAG_LIN_BOOT, msg))
    {
        if(!msg.dataValid) // Waiting for the command to be sent
            return;

        PrintSerialMessage("LIN boot complete");
        // Boot complete
        LIN.RemoveMessage(LIN_Message_ID::VAG_LIN_BOOT);
        LIN.SetRequest(LIN_Message_ID::VAG_PTC_REQUEST, 8, tx);
        LIN.SetReceiver(LIN_Message_ID::VAG_PTC_RECEIVE, 8);
    }

    if(LIN.TryGetData(LIN_Message_ID::VAG_PTC_REQUEST, msg) && msg.dataValid)
    {
        if (counter < 100)
            counter++;

        tx[1] = counter < 7 ? 0xC4 : 0xC5;

        tx[2] = tx[3] = ptcPowerRequest;

        if (counter >= 3)
        {
            tx[6] = ptcPowerRequest > 0 ? 0x80 : 0x00;
            tx[7] = 0x00;
        }

        LIN.SetRequest(LIN_Message_ID::VAG_PTC_REQUEST, 8, tx);
    }
}

float PTC_Current;
int PTC_Voltage;
float PTC_Temp;

void ExtractData()
{
    static uint32_t lastData = 0;
    if (millis() - lastData > 500)
    {
        lastData = millis();
    }
    else
        return;

    LIN_Message msg;
    if (LIN.TryGetData(LIN_Message_ID::VAG_PTC_RECEIVE, msg) && msg.dataValid && msg.dataSize >= 8 && msg.data[1] < 0xFE) // PTC
    {
        PTC_Current = msg.data[0] * 0.5f;
        PTC_Voltage = msg.data[1] * 2;
        PTC_Temp = msg.data[5] * 0.25f + 2;

        string str = "PTC| Curent: " + FloatToString(PTC_Current, 1) + "A, Voltage: " + ToString(PTC_Voltage) + " V, Temp: " + ToString(PTC_Temp) + "C";
        PrintSerialMessage(str);
    }
}