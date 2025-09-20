
#include "SerialReader.h"
#include "SerialPrint.h"
#include "Contactor.h"
#include "Throttle.h"
#include "Timer.h"
#include "VCU.h"
#include "CAN.h"

#include <vector>

typedef void (*cmdPtr)();

struct CommandPointer
{
  public:
    String name;
    cmdPtr ptr;

    CommandPointer(String Name, cmdPtr Ptr)
    {
        name = Name;
        ptr = Ptr;
    }
};

std::vector<CommandPointer> commandPointers;

String command;
std::vector<String> parameters;

Timer logOutputTimer = Timer(1);

bool IsASCII(char in)
{
    return in >= ' ' && in <= '~';
}

char ASCIIToLower(char in)
{
    if (in <= 'Z' && in >= 'A')
        return in - ('Z' - 'z');
    return in;
}

String RemoveNonASCII(String str)
{
    String newStr = str;
    for (int i = newStr.length() - 1; i >= 0; i--)
        if (!IsASCII(newStr[i]))
            newStr.remove(i);
    return newStr;
}

void ToLower(String &str)
{
    for (int i = 0; i < str.length(); i++)
        str[i] = ASCIIToLower(str[i]);
}

void FindCommandAndParameters(String fullString)
{
    int startIndex = 0;
    command = "";
    parameters.clear();

    for (int i = 0; i < fullString.length(); i++)
        if (fullString[i] == ' ')
        {
            command = fullString.substring(0, i);
            startIndex = i + 1;
            break;
        }

    if (command == "")
    {
        command = fullString;
        return;
    }

    for (int i = startIndex; i < fullString.length(); i++)
        if (fullString[i] == ' ')
        {
            parameters.push_back(fullString.substring(startIndex, i));
            startIndex = i + 1;
        }

    if (startIndex < fullString.length())
        parameters.push_back(fullString.substring(startIndex, fullString.length()));
}

void ProcessCommand()
{
    for (int i = 0; i < commandPointers.size(); i++)
    {
        if (commandPointers[i].name == command)
        {
            commandPointers[i].ptr();
            return;
        }
    }

    PrintSerialMessage("Command {" + command + "} not found");
}

bool shouldOutputStatus;
void OutputStatus()
{
    shouldOutputStatus = !shouldOutputStatus;
}

void PrintInverterStatus()
{
    InverterStatus status = VCU::GetInverterStatus();
    Stats stats = status.stats;

    String str = "Inverter voltage: " + ToString(status.inverterVoltage) + " V " + 
                 "\n" + stats.GetString() +
                 "\nIs in error state: " + BoolToString(status.error_state);

    PrintSerialMessage(str);
}

void PrintInverterMaxStats()
{
    Stats stats = VCU::GetMaxRecordedStats();
    String str = "Max recorded stats:\n" +
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
    if (parameters.size() == 0)
    {
        PrintSerialMessage("Not enough parameters!");
        return;
    }

    short request = parameters[0].toInt();
    VCU::SetFinalTorqueRequest(request);
}

void SetMaxTorque()
{
    if (parameters.size() == 0)
    {
        PrintSerialMessage("Not enough parameters!");
        return;
    }

    short request = parameters[0].toInt();
    if(VCU::SetMaxTorqueRequest(request))
        PrintSerialMessage("Torque set to " + ToString(request));
}

void SetRegenTorque()
{
    if (parameters.size() == 0)
    {
        PrintSerialMessage("Not enough parameters!");
        return;
    }

    short request = parameters[0].toInt();
    if(VCU::SetRegenTorque(request))
        PrintSerialMessage("Regen torque set to " + ToString(request));
}

void SetMinMaxRegenRPMRange()
{
    if (parameters.size() <= 1)
    {
        PrintSerialMessage("Not enough parameters!");
        return;
    }

    short lowRPM = parameters[0].toInt();
    short highRPM = parameters[1].toInt();
    if(VCU::SetRegenRPMRange(lowRPM, highRPM))
        PrintSerialMessage("Regen RPM range set to " + ToString(lowRPM) + "-" + ToString(highRPM));
}

void OutputHelp()
{
    String outputStr = "Available commands:\n";
    for (int i = 0; i < commandPointers.size(); i++)
        outputStr += commandPointers[i].name + "\n";

    outputStr.remove(outputStr.length() - 1);
    PrintSerialMessage(outputStr);
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
    Throttle::printDetailedLog = !Throttle::printDetailedLog;
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
    if (parameters.size() == 0)
    {
        PrintSerialMessage("Not enough parameters!");
        return;
    }

    String state = parameters[0];
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


    VCU::SetContactorForTesting((int)test);
}

void SendCAN()
{
    if (parameters.size() == 0)
    {
        PrintSerialMessage("Not enough parameters!");
        return;
    }

    unsigned int ID = parameters[0].toInt();

    int size = Min(parameters.size() - 1, 8);
    unsigned char frame[8];
    for(int i = 0; i < size; i++)
    {
        frame[i] = parameters[i + 1].toInt();
    }

    PrintSerialMessage("Sending custom CAN MSG ID: 0x" + IntToHex(ID) + " Bytes: " + BytesToString(frame, 8));
    VCU::SendCustomCanMessage(ID, frame);
}

void InitializeSerialReader()
{
    commandPointers.push_back(CommandPointer("help", OutputHelp));
    commandPointers.push_back(CommandPointer("status", OutputStatus));
    commandPointers.push_back(CommandPointer("maxstats", PrintInverterMaxStats));
    commandPointers.push_back(CommandPointer("clearmaxstats", ClearMaxStats));
    commandPointers.push_back(CommandPointer("obcstatus", PrintOBCStatus));
    commandPointers.push_back(CommandPointer("printcan", PrintCan));
    commandPointers.push_back(CommandPointer("torque", SetTorque));
    commandPointers.push_back(CommandPointer("maxtorque", SetMaxTorque));
    commandPointers.push_back(CommandPointer("regentorque", SetRegenTorque));
    commandPointers.push_back(CommandPointer("regenrpm", SetMinMaxRegenRPMRange));
    commandPointers.push_back(CommandPointer("throttleout", GetThrottle));
    commandPointers.push_back(CommandPointer("throttleoutdetailed", GetThrottleDetailed));
    commandPointers.push_back(CommandPointer("vacuum", PrintVacuumSensorData));
    commandPointers.push_back(CommandPointer("togglegen2", ToggleGen2));
    commandPointers.push_back(CommandPointer("testcontactor", SetContactor));
    commandPointers.push_back(CommandPointer("sendcan", SendCAN));
}

#define INPUT_BUFFER_SIZE 64
char serialReadBuffer[INPUT_BUFFER_SIZE];
void ProcessString()
{
    String tempString = serialReadBuffer;
    tempString = RemoveNonASCII(tempString);
    ToLower(tempString);
    FindCommandAndParameters(tempString);

    String outputString = "Received command: {" + command + "} ";
    for (int i = 0; i < parameters.size(); i++)
        outputString += parameters[i] + " ";
    PrintSerialMessage(outputString);

    ProcessCommand();
}

bool serialTail;
int idx = 0;
void TickSerialReader()
{
    while (Serial.available())
    {
        char inChar = (char)Serial.read();

        if (inChar == '\n' || idx >= INPUT_BUFFER_SIZE - 1)
        {
            if (idx > INPUT_BUFFER_SIZE - 1)
            {
                idx = INPUT_BUFFER_SIZE - 1;
                PrintSerialMessage("Input message is longer than " + ToString(INPUT_BUFFER_SIZE) + "!");
            }
            serialTail = true;
            serialReadBuffer[idx] = '\n';
        }

        if (serialTail)
        {
            serialTail = false;
            idx = 0;
            ProcessString();
        }
        else
        {
            serialReadBuffer[idx] = inChar;
            idx++;
        }
    }

    if (logOutputTimer.HasTriggered())
    {
        if (shouldOutputStatus)
            PrintInverterStatus();

        if(printVacuumSensorData)
            VCU::GetVacuumSensor()->PrintDebugValues();
    }
}