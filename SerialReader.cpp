
#include "SerialReader.h"
#include "SerialPrint.h"
#include "Contactor.h"
#include "Throttle.h"
#include "Timer.h"
#include "VCU.h"
#include "CAN.h"

#include <Arduino.h>

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
            break;
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

String PDMModelTypeToString(PDMType type)
{
    switch (type)
    {
    default:
    case PDMType::ZE0_2011_2013:
        return "ZE0 2011-2013 (or not connected...)";
    case PDMType::AZE0_2014_2017:
        return "AZE0 2014-2017";
    case PDMType::ZE1_2018:
        return "ZE1 2018+";
    }
}

void PrintInverterStatus()
{
    InverterStatus status = VCU::GetInverterStatus();

    String str = "EV status:\nPDM model mode: " + PDMModelTypeToString(status.PDMModelType) +
                 "\nInverter voltage: " + ToString(status.inverterVoltage) + "V " + "\nRPM: " + ToString(status.rpm) +
                 "\nMotor torque: " + ToString(status.motorTorque) + "\nMotor power: " + ToString(status.motorPower) +
                 "\nInverter temperature: " + FloatToString(status.inverter_temperature, 2) + "C " +
                 "\nMotor temperature: " + FloatToString(status.motor_temperature, 2) + "C " +
                 "\nIs in error state: " + BoolToString(status.error_state);

    PrintSerialMessage(str);
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

void DischargeKw()
{
    if (parameters.size() == 0)
    {
        PrintSerialMessage("Not enough parameters!");
        return;
    }

    VCU::SetBatteryDischargeLimit(parameters[0].toFloat());
}

void GetThrottle()
{
    VCU::ToggleThrottlePrint();
}

void GetThrottleDetailed()
{
    Throttle::printDetailedLog = !Throttle::printDetailedLog;
}

void ToggleGen2()
{
    if (VCU::ToggleGen2Codes())
        PrintSerialMessage("Gen 2 CAN codes selected");
    else
        PrintSerialMessage("Gen 1 CAN codes selected");
}

void TogglePDMCAN()
{
    if(VCU::TogglePDMCAN())
        PrintSerialMessage("PDM CAN enabled");
    else
        PrintSerialMessage("PDM CAN disabled");
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

    VCU::SetContactorForTesting((int)test);
}

void InitializeSerialReader()
{
    commandPointers.push_back(CommandPointer("help", OutputHelp));
    commandPointers.push_back(CommandPointer("status", OutputStatus));
    commandPointers.push_back(CommandPointer("printcan", PrintCan));
    commandPointers.push_back(CommandPointer("torque", SetTorque));
    commandPointers.push_back(CommandPointer("maxtorque", SetMaxTorque));
    commandPointers.push_back(CommandPointer("dischargekw", DischargeKw));
    commandPointers.push_back(CommandPointer("throttleout", GetThrottle));
    commandPointers.push_back(CommandPointer("throttleoutdetailed", GetThrottleDetailed));
    commandPointers.push_back(CommandPointer("togglegen2", ToggleGen2));
    commandPointers.push_back(CommandPointer("togglepdmcan", TogglePDMCAN));
    commandPointers.push_back(CommandPointer("testcontactor", SetContactor));
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
    }
}