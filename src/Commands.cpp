
#include "Commands.h"

#include "EVLib/SerialPrint.h"
#include "EVLib/SerialReader.h"
#include "EVLib/MathUtils.h"

#include "VCU.h"
#include "EVLib/CAN.h"
#include "Contactor.h"

using namespace std;

namespace commands
{
    bool shouldOutputStatus;
    void OutputStatus()
    {
        shouldOutputStatus = !shouldOutputStatus;
    }

    void PrintInverterStatus()
    {
        InverterStatus status = VCU::GetInverterStatus();
        Stats stats = status.stats;

        string str = "Inverter voltage: " + ToString(status.inverterVoltage) + " V " + 
                    "\n" + stats.GetString() +
                    "\nIs in error state: " + BoolToString(status.error_state);

        PrintSerialMessage(str);
    }

    void PrintInverterMaxStats()
    {
        Stats stats = VCU::GetMaxRecordedStats();
        string str = "Max recorded stats:\n" +
                    stats.GetString();
        PrintSerialMessage(str);
    }

    void ClearMaxStats()
    {
        VCU::ClearMaxRecordedStats();
    }

    void PrintOBCStatus()
    {
        PrintSerialMessage(VCU::GetOBCStatus());
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
        VCU::SetFinalTorqueRequest(request);
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
        if(VCU::SetMaxTorqueRequest(request))
            PrintSerialMessage("Torque set to " + ToString(request));
    }

    void SetRegenTorque()
    {
        auto parameters = *GetParameters();

        if (parameters.size() == 0)
        {
            PrintSerialMessage("Not enough parameters!");
            return;
        }

        short request = stoi(parameters[0]);
        if(VCU::SetRegenTorque(request))
            PrintSerialMessage("Regen torque set to " + ToString(request));
    }

    void SetMinMaxRegenRPMRange()
    {
        auto parameters = *GetParameters();

        if (parameters.size() <= 1)
        {
            PrintSerialMessage("Not enough parameters!");
            return;
        }

        short lowRPM = stoi(parameters[0]);
        short highRPM = stoi(parameters[1]);
        if(VCU::SetRegenRPMRange(lowRPM, highRPM))
            PrintSerialMessage("Regen RPM range set to " + ToString(lowRPM) + "-" + ToString(highRPM));
    }

    void PrintCan()
    {
        CAN::printReceive = !CAN::printReceive;
    }

    void GetThrottle()
    {
        VCU::ToggleThrottlePrint();
    }

    void GetThrottleDetailed()
    {
        VCU::ToggleThrottlePrintDetailed();
    }

    bool printVacuumSensorData;
    void PrintVacuumSensorData()
    {
        printVacuumSensorData = !printVacuumSensorData;
    }

    void ToggleGen2()
    {
        if (VCU::ToggleGen2Codes())
            PrintSerialMessage("Gen 2 CAN codes selected");
        else
            PrintSerialMessage("Gen 1 CAN codes selected");
    }

    void SetContactor()
    {
        auto parameters = *GetParameters();

        if (parameters.size() == 0)
        {
            PrintSerialMessage("Not enough parameters!");
            return;
        }

        string state = parameters[0];
        ToLower(state);

        ContactorTest test = ContactorTest::None;
        if(state == "+" || state == "positive")
            test = ContactorTest::Positive;
        else if(state == "-" || state == "negative")
            test = ContactorTest::Negative;
        else if(state == "p" || state == "precharge")
            test = ContactorTest::Precharge;
        else if(state == "v" || state == "vacuum")
            test = ContactorTest::Vacuum;
        else if(state == "w" || state == "water")
            test = ContactorTest::Water;
        else if(state == "d" || state == "dc" || state == "dcdc")
            test = ContactorTest::DCDC;
        else if(state == "c" || state == "charger")
            test = ContactorTest::Charger;

        VCU::SetContactorForTesting((int)test);
    }

    void SendCAN()
    {
        auto parameters = *GetParameters();

        if (parameters.size() == 0)
        {
            PrintSerialMessage("Not enough parameters!");
            return;
        }

        unsigned int ID = stoi(parameters[0]);

        int size = Min(parameters.size() - 1, 8);
        unsigned char frame[8];
        for(int i = 0; i < size; i++)
        {
            frame[i] = stoi(parameters[i + 1]);
        }

        PrintSerialMessage("Sending custom CAN MSG ID: 0x" + IntToHex(ID) + " Bytes: " + BytesToString(frame, 8));
        VCU::SendCustomCanMessage(ID, frame);
    }

    void TogglePPDebug()
    {
        VCU::ToggleDebugPP();
    }

    void InitializeCommands()
    {
        AddCommand(CommandPointer("status", OutputStatus));
        AddCommand(CommandPointer("maxstats", PrintInverterMaxStats));
        AddCommand(CommandPointer("clearmaxstats", ClearMaxStats));
        AddCommand(CommandPointer("obcstatus", PrintOBCStatus));
        AddCommand(CommandPointer("printcan", PrintCan));
        AddCommand(CommandPointer("torque", SetTorque));
        AddCommand(CommandPointer("maxtorque", SetMaxTorque));
        AddCommand(CommandPointer("regentorque", SetRegenTorque));
        AddCommand(CommandPointer("regenrpm", SetMinMaxRegenRPMRange));
        AddCommand(CommandPointer("throttleout", GetThrottle));
        AddCommand(CommandPointer("throttleoutdetailed", GetThrottleDetailed));
        AddCommand(CommandPointer("vacuum", PrintVacuumSensorData));
        AddCommand(CommandPointer("togglegen2", ToggleGen2));
        AddCommand(CommandPointer("testcontactor", SetContactor));
        AddCommand(CommandPointer("sendcan", SendCAN));
        AddCommand(CommandPointer("chargerpp", TogglePPDebug));
    }
}