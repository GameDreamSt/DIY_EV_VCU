
#include <vector>
#include <string>

void SetSensorsAnalogResolution(int bits);
typedef float (*SensorFunction)();

class Sensor
{
  private:
    float lowestValue;
    float highestValue;
    float calibrationValue;

    int analogPin;
    SensorFunction sensorFunction;

    int averageTimes;
    float inputResolution;
    float currentThrottle;

    bool beingCalibrated;
    bool manualMode;

    float lastDisconnectMessage;
    float disconnectDelay;

    std::string name;

    float ReadAnalog();
    float GetAverageAnalog();

  public:
    Sensor();
    Sensor(int analogToDigitalPin);
    Sensor(SensorFunction func);

    void Tick();
    void Reset();

    bool IsBeingCalibrated();
    float GetNormalized();
    void PrintDebugValues();
    std::string GetDebugValuesString();
    float GetRawValue();

    void SetValuesManually(float min, float max);
    void SetName(std::string name);
    std::string GetName();
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