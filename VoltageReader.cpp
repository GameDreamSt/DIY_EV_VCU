
#include "Pinout.h"
#include "VoltageReader.h"
#include <Arduino.h>

#define READ_AVG 200
#define BUFFER_SIZE 10

VoltageReader::VoltageReader(int analogToDigitalPin, float bigResistorOhms, float smallResistorOhms)
{
    analogReference(AR_DEFAULT);
    bigResistor = bigResistorOhms;
    smallResistor = smallResistorOhms;
    inputResolution = pow(2, ANALOG_RES);

    alanogPinID = analogToDigitalPin;
    fullBuffer = &valueBufferA;
    insertionBuffer = &valueBufferA;
}

float VoltageReader::GetVoltage()
{
    float A0Value = (float)analogRead(alanogPinID);
    float voltage_sensed = VCCVoltage * A0Value / inputResolution;
    // Serial.print("voltage_sensed:");
    // Serial.print(voltage_sensed);
    return voltage_sensed * (1 + (bigResistor / smallResistor)) - 2.6180238724f;
}

float VoltageReader::GetVoltageAverage()
{
    float voltage_temp_average = 0;
    for (int i = 0; i < READ_AVG; i++)
        voltage_temp_average += GetVoltage();

    return voltage_temp_average / (float)READ_AVG;
}

float VoltageReader::GetVoltageMedian()
{
    if (insertionBuffer->size() >= BUFFER_SIZE)
    {
        if (insertionBuffer == &valueBufferA)
        {
            fullBuffer = &valueBufferA;
            insertionBuffer = &valueBufferB;
        }
        else
        {
            fullBuffer = &valueBufferB;
            insertionBuffer = &valueBufferA;
        }

        insertionBuffer->clear();
    }

    insertionBuffer->insert(GetVoltageAverage());

    auto it = fullBuffer->begin();
    int n = fullBuffer->size() / 2;
    for (int i = 0; i < n; i++)
        it++;
    return *it;
}