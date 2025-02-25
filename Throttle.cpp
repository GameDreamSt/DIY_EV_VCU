
#include "Pinout.h"
#include "SerialPrint.h"
#include "Throttle.h"
#include <Arduino.h>

#define READ_AVG 50
int floatSize = sizeof(float);

bool Throttle::printDetailedLog = false;

Throttle::Throttle(int analogToDigitalPin)
{
    analogPin = analogToDigitalPin;
    inputResolution = pow(2, ANALOG_RES);
}

float Throttle::ReadAnalog()
{
    return (float)analogRead(analogPin) / inputResolution;
}

float Throttle::GetAverageAnalog()
{
    float voltage_temp_average = 0;
    for (int i = 0; i < READ_AVG; i++)
        voltage_temp_average += ReadAnalog();

    return voltage_temp_average / (float)READ_AVG;
}

void Throttle::Tick()
{
    currentThrottle = GetAverageAnalog();

    if (printDetailedLog)
        PrintSerialMessage("T" + String(analogPin) + ": " + String(currentThrottle) + " " + String(lowestValue) + " " +
                           String(highestValue) + " " + String(GetNormalizedThrottle()));

    if (currentThrottle < 0.05f) // If below 5% raw value, then reset. Highly likely that the connector was disconnected, either intentionally or not.
    {
        Reset();
        return;
    }

    if (lowestValue < 0.06f) // Initialize the lowest value
        lowestValue = highestValue;

    if (currentThrottle < lowestValue + (highestValue - lowestValue) * 0.05f)
        beingCalibrated = false;
    if (currentThrottle < lowestValue)
        lowestValue = currentThrottle;
    if (currentThrottle > calibrationValue)
    {
        calibrationValue = currentThrottle * 1.05f;
        highestValue = currentThrottle;
        beingCalibrated = true;
    }
}

void Throttle::Reset()
{
    lowestValue = highestValue = calibrationValue = 0;
}

float remap(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float remap01(float x, float in_min, float in_max)
{
    return (x - in_min) / (in_max - in_min);
}

float Throttle::GetNormalizedThrottle()
{
    // Safety:
    // 1. In calibration mode the vehicle will not go if you dont release the throttle.
    // 2. There needs to be at least 25% of raw (0 to VCC) throttle travel present. Otherwise it means that it wasn't fully callibrated.
    if (beingCalibrated || abs(highestValue - lowestValue) < 0.25f)
        return 0;

    float normalizedValue = remap01(currentThrottle, lowestValue, highestValue);

    if (normalizedValue < 0.01f) // 1% deadzone
        normalizedValue = 0;
    else if (normalizedValue > 1)
        normalizedValue = 1;

    return normalizedValue;
}

std::vector<unsigned char> Throttle::Save()
{
    std::vector<unsigned char> data;

    data.resize(floatSize * 2);

    void *dataPtr = data.data();
    memcpy(dataPtr, &lowestValue, floatSize);
    dataPtr += floatSize;
    memcpy(dataPtr, &highestValue, floatSize);
    dataPtr += floatSize;

    return data;
}

void Throttle::Load(std::vector<unsigned char> data)
{
    if (data.size() < floatSize * 2)
    {
        PrintSerialMessage("Incorrect data load size for Throttle! Data: " + BytesToString(data));
        return;
    }

    void *dataPtr = data.data();
    memcpy(&lowestValue, dataPtr, floatSize);
    dataPtr += floatSize;
    memcpy(&highestValue, dataPtr, floatSize);
    dataPtr += floatSize;
    calibrationValue = highestValue * 1.05f;
}

// ****************************************************

void ThrottleManager::Tick()
{
    for (int i = 0; i < throttles.size(); i++)
        throttles[i].Tick();
}

void ThrottleManager::AddThrottle(Throttle throttle)
{
    throttles.push_back(throttle);
}

float ThrottleManager::GetNormalizedThrottle()
{
    float *values = new float[throttles.size()];
    float averageValue = 0;

    for (int i = 0; i < throttles.size(); i++)
    {
        values[i] = throttles[i].GetNormalizedThrottle();
        averageValue += values[i];
    }

    // Check throttle deviation
    for (int i = 0; i < throttles.size(); i++)
    {
        for (int j = i + 1; j < throttles.size(); j++)
        {
            float value = abs(values[i] - values[j]);
            if (value > 0.07f) // 7% derived from testing
            {
                value *= 100;
                PrintSerialMessage("Throttle deviation between T" + String(i) + " and T" + String(j) + " detected! " +
                                   String((int)value) + "%");
                delete values;
                return 0;
            }
        }
    }

    delete values;
    return averageValue / (float)throttles.size();
}
