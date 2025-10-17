
#include <vector>
#include <string>

class Throttle
{
  private:
    float lowestValue;
    float highestValue;
    float calibrationValue;

    int analogPin;
    float inputResolution;
    float currentThrottle;

    bool beingCalibrated;

    float ReadAnalog();
    float GetAverageAnalog();

  public:
    static bool printDetailedLog;

    Throttle(int analogToDigitalPin);

    void Tick();
    void Reset();

    float GetNormalizedThrottle();
    void PrintDebugValues();
    std::string GetDebugValuesString();
    float GetRawValue();

    std::vector<unsigned char> Save();
    void Load(std::vector<unsigned char> data);
};

class ThrottleManager
{
  private:
    std::vector<Throttle> throttles;

  public:
    void Tick();
    void AddThrottle(Throttle throttle);
    float GetNormalizedThrottle();
};