
#pragma once

#include <set>

class VoltageReader
{
  private:
    float bigResistor = 5070;
    float smallResistor = 7500;
    float inputResolution = 1023;

    std::set<float> valueBufferA;
    std::set<float> valueBufferB;
    std::set<float> *fullBuffer;
    std::set<float> *insertionBuffer;

    int alanogPinID = 0;

    float GetVoltage();

  public:
    VoltageReader(int analogToDigitalPin, float bigResistorOhms, float smallResistorOhms);

    float GetVoltageAverage();
    float GetVoltageMedian();
};