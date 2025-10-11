
#include "SerialPrint.h"
#include "Timer.h"
#include <math.h>
#include <Arduino.h>

#define MAX_MESSAGES 8

using namespace std;

string pendingMsgs[MAX_MESSAGES]; // TODO: calculate the hash for pending pendingMsgs and do not queue the same ones
Timer maxMsgCountTimer = Timer(1);
bool triggerMaxMsgCount;

void PrintSerialMessage(string outputMsg)
{
    for (int i = 0; i < MAX_MESSAGES; i++)
    {
        if (pendingMsgs[i].length() == 0)
        {
            pendingMsgs[i] = outputMsg + "\n";
            return;
        }
    }

    triggerMaxMsgCount = true;
}

void PrintSerialMessageHEX(string message, int hexData)
{
    PrintSerialMessage(message + IntToHex(hexData));
}

void PrintSerialMessage(string message, int intData)
{
    PrintSerialMessage(message + to_string(intData));
}

string FloatToString(float var, int precision)
{
    float multiplier = pow(10, precision);
    string wholeNum = to_string((long)var);

    var = var - (float)((long)var);
    var *= multiplier;
    string fraction = to_string((long)var);
    return wholeNum + "." + fraction;
}

string ToString(int var)
{
    return to_string(var);
}

string BoolToString(bool var)
{
    return var ? "True" : "False";
}

string IntToHex(int n)
{
    if (n == 0)
        return "0";

    // ans string to store hexadecimal number
    string temp = "";

    while (n != 0)
    {
        // remainder variable to store remainder
        int rem = 0;

        // ch variable to store each character
        char ch;
        // storing remainder in rem variable.
        rem = n % 16;

        // check if temp < 10
        if (rem < 10)
            ch = rem + 48;
        else
            ch = rem + 55;

        // updating the ans string with the character variable
        temp += ch;
        n = n / 16;
    }

    string ans = "";
    for (int i = temp.length() - 1; i >= 0; i--)
        ans += temp[i];

    return ans;
}

string BytesToString(std::vector<unsigned char> data)
{
    if (data.size() == 0)
        return "NO_DATA";

    string str = "";

    for (int i = 0; i < data.size() - 1; i++)
        str += IntToHex(data[i]) + " ";
    str += IntToHex(data[data.size() - 1]);

    return str;
}

string BytesToString(unsigned char *data, int length)
{
    if (length <= 0)
        return "";

    std::vector<unsigned char> dataVector(data, data + length);
    return BytesToString(dataVector);
}

int currentMessageIndex = -1;
int bytesWritten = 0;

void TickSerialWriter()
{
    if (triggerMaxMsgCount && maxMsgCountTimer.HasTriggered())
    {
        triggerMaxMsgCount = false;
        Serial.print("Max msg count reached! Increase the baud rate or reduce msg count!\n");
        return;
    }

    int bytesAvailable = Serial.availableForWrite();
    if (bytesAvailable <= 0)
        return;

    if (currentMessageIndex == -1)
    {
        for (int i = 0; i < MAX_MESSAGES; i++)
            if (pendingMsgs[i].length() != 0)
            {
                currentMessageIndex = i;
                bytesWritten = 0;
                break;
            }
    }

    if (currentMessageIndex == -1)
        return;

    int stringLength = pendingMsgs[currentMessageIndex].length();
    int bytesToWrite = stringLength - bytesWritten;
    char *ptr = &pendingMsgs[currentMessageIndex][bytesWritten];
    if (bytesToWrite > bytesAvailable)
        bytesToWrite = bytesAvailable;

    bytesWritten += Serial.write(ptr, bytesToWrite);

    if (bytesWritten >= stringLength)
    {
        pendingMsgs[currentMessageIndex] = "";
        currentMessageIndex = -1;
    }
}
