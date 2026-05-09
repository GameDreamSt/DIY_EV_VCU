
#include "SerialReader.h"
#include "SerialPrint.h"

#include <Arduino.h>

using namespace std;

vector<CommandPointer> commandPointers;

string command;
vector<string> parameters;

vector<string>* GetParameters() { return &parameters; }

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

string RemoveNonASCII(string str)
{
    string newStr = str;
    for (int i = newStr.length() - 1; i >= 0; i--)
        if (!IsASCII(newStr[i]))
            newStr.erase(newStr.begin() + i);
    return newStr;
}

void ToLower(string &str)
{
    for (int i = 0; i < str.length(); i++)
        str[i] = ASCIIToLower(str[i]);
}

void FindCommandAndParameters(string fullString)
{
    int startIndex = 0;
    command = "";
    parameters.clear();

    for (int i = 0; i < fullString.length(); i++)
        if (fullString[i] == ' ')
        {
            command = fullString.substr(0, i);
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
            parameters.push_back(fullString.substr(startIndex, i));
            startIndex = i + 1;
        }

    if (startIndex < fullString.length())
        parameters.push_back(fullString.substr(startIndex, fullString.length()));
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

void OutputHelp()
{
    string outputStr = "Available commands:\n";
    for (int i = 0; i < commandPointers.size(); i++)
        outputStr += commandPointers[i].name + "\n";

    outputStr.erase(outputStr.length() - 1);
    PrintSerialMessage(outputStr);
}

void AddCommand(CommandPointer command)
{
    commandPointers.push_back(command);
}

void InitializeSerialReader()
{
    commandPointers.push_back(CommandPointer("help", OutputHelp));
}

#define INPUT_BUFFER_SIZE 64
char serialReadBuffer[INPUT_BUFFER_SIZE];
void ProcessString(int length)
{
    string tempString = string(serialReadBuffer, length);
    
    tempString = RemoveNonASCII(tempString);
    ToLower(tempString);
    FindCommandAndParameters(tempString);

    string outputString = "Received command: {" + command + "} ";
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
            ProcessString(idx);
            idx = 0;
        }
        else
        {
            serialReadBuffer[idx] = inChar;
            idx++;
        }
    }
}