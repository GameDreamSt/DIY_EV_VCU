
#include <vector>
#include <string>

void SetSensorsAnalogResolution(int bits);

class Sensor
{
  private:
    float lowestValue;
    float highestValue;
    float calibrationValue;

    int analogPin;
    float inputResolution;
    float currentThrottle;

    bool beingCalibrated;
    bool manualMode;

    float ReadAnalog();
    float GetAverageAnalog();

  public:
    Sensor(int analogToDigitalPin);

    void Tick();
    void Reset();

    float GetNormalized();
    void PrintDebugValues();
    std::string GetDebugValuesString();
    float GetRawValue();

    void SetValuesManually(float min, float max);
};

class ThrottleManager
{
  private:
    std::vector<Sensor> throttles;

  public:
    void Tick();
    void AddThrottle(Sensor throttle);
    float GetNormalized();
    void PrintDebugValues();
};