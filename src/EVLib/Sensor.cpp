
#include "EVTime.h"
#include "Sensor.h"
#include "SerialPrint.h"
#include "Timer.h"
#include <Arduino.h>
#include <math.h>

#define READ_AVG 10
#define DEADZONE_LOW 0.05
#define DEADZONE_HIGH 0.95

int analogResolutionBits = 12;
int floatSize = sizeof(float);
float lastMessage = 0;

void SetSensorsAnalogResolution(int bits)
{
    analogResolutionBits = bits;
}

Sensor::Sensor() : Sensor(-1)
{
}

Sensor::Sensor(int analogToDigitalPin)
{
    Reset();
    lastDisconnectMessage = 0;
    disconnectDelay = 1;
    manualMode = false;
    analogPin = analogToDigitalPin;
    sensorFunction = nullptr;
    inputResolution = pow(2, analogResolutionBits);
    averageTimes = READ_AVG;
}

Sensor::Sensor(SensorFunction func) : Sensor(-1)
{
    sensorFunction = func;
}

bool Sensor::IsBeingCalibrated()
{
    return beingCalibrated;
}

void Sensor::SetValuesManually(float min, float max)
{
    lowestValue = min;
    highestValue = max;
    calibrationValue = max * 1.05f;
    beingCalibrated = false;
    manualMode = true;
}

float Sensor::ReadAnalog()
{
    return (float)analogRead(analogPin) / inputResolution;
}

float Sensor::GetAverageAnalog()
{
    if (sensorFunction != nullptr)
        return sensorFunction();

    if (analogPin < 0)
        return 0;

    float voltage_temp_average = 0;
    for (int i = 0; i < averageTimes; i++)
        voltage_temp_average += ReadAnalog();

    if (averageTimes > 0)
        return voltage_temp_average / (float)averageTimes;
    else
        return ReadAnalog();
}

void Sensor::PrintDebugValues()
{
    PrintSerialMessage(GetDebugValuesString());
}

std::string Sensor::GetDebugValuesString()
{
    return "T" + GetName() + ": " + ToString(currentThrottle) + " " + ToString(lowestValue) + " " +
           ToString(highestValue) + " " + ToString(calibrationValue) + " " + ToString(GetNormalized()) +
           (beingCalibrated ? " Being calibrated..." : " Calibration completed.");
}

void Sensor::SetName(std::string newName)
{
    name = newName;
}
std::string Sensor::GetName()
{
    if (name != "")
        return name;

    return ToString(analogPin);
}

void Sensor::Tick()
{
    currentThrottle = GetAverageAnalog();

    if (manualMode)
        return;

    if (currentThrottle < 0.02f) // If below 2% raw value, then reset. Highly likely that the connector was
                                 // disconnected, either intentionally or not.
    {
        if (GetTime() - lastDisconnectMessage > disconnectDelay)
        {
            lastDisconnectMessage = GetTime();
            disconnectDelay += 1;
            PrintSerialMessage("Sensor was disconnected! " + GetName());
        }
        Reset();
        return;
    }

    if (lowestValue < 0.02f || currentThrottle < lowestValue)
        lowestValue = currentThrottle;

    if (currentThrottle > calibrationValue || highestValue - lowestValue < 0.05f)
    {
        calibrationValue = currentThrottle * 1.05f;
        highestValue = currentThrottle;
        beingCalibrated = true;
    }

    if (lowestValue > highestValue)
        highestValue = lowestValue;

    if (currentThrottle < lowestValue + (highestValue - lowestValue) * 0.05f)
        beingCalibrated = false;
}

float Sensor::GetRawValue()
{
    return currentThrottle;
}

void Sensor::Reset()
{
    lowestValue = highestValue = calibrationValue = 0;
    beingCalibrated = true;
}

float remap(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float remap01(float x, float in_min, float in_max)
{
    return (x - in_min) / (in_max - in_min);
}

float clamp(float x, float a, float b)
{
    return fmax(a, fmin(b, x));
}

float Sensor::GetNormalized()
{
    // Safety:
    // 1. In calibration mode the vehicle will not go if you dont release the throttle.
    // 2. There needs to be at least 25% of raw (0 to VCC) throttle travel present. Otherwise it means that it wasn't
    // fully callibrated.
    if (beingCalibrated || abs(highestValue - lowestValue) < 0.1f)
        return 0;

    if (lowestValue > highestValue)
        return 0;

    float normalizedValue =
        remap01(currentThrottle, lowestValue, highestValue); // voltage remap from lowest to highest calibrated values
    normalizedValue = remap01(normalizedValue, DEADZONE_LOW, DEADZONE_HIGH); // deadzone remap
    normalizedValue = clamp(normalizedValue, 0, 1);
    // Apply throttle filters here, otherwise it's going to be linear

    return normalizedValue;
}

// ****************************************************

void ThrottleManager::Tick()
{
    for (int i = 0; i < throttles.size(); i++)
        throttles[i].Tick();
}

void ThrottleManager::AddThrottle(Sensor throttle)
{
    throttles.push_back(throttle);
}

void ThrottleManager::PrintDebugValues()
{
    for (int i = 0; i < throttles.size(); i++)
    {
        throttles[i].PrintDebugValues();
    }
}

float ThrottleManager::GetNormalized()
{
    float *values = new float[throttles.size()];
    float averageValue = 0;
    bool beingCalibrated = false;

    for (int i = 0; i < throttles.size(); i++)
    {
        values[i] = throttles[i].GetNormalized();
        averageValue += values[i];
        beingCalibrated |= throttles[i].IsBeingCalibrated();
    }

    if (beingCalibrated)
    {
        delete values;
        return 0;
    }

    // Check throttle deviation
    for (int i = 0; i < throttles.size(); i++)
    {
        for (int j = i + 1; j < throttles.size(); j++)
        {
            float value = abs(values[i] - values[j]);
            if (value > 0.2f) // derived from testing
            {
                value *= 100;
                if (GetTime() - lastMessage > 1)
                {
                    lastMessage = GetTime();
                    PrintSerialMessage("Throttle deviation between T" + ToString(i) + " and T" + ToString(j) +
                                       " detected! " + ToString((int)value) + "%");
                }
                delete values;
                return 0;
            }
        }
    }

    delete values;
    return averageValue / (float)throttles.size();
}
