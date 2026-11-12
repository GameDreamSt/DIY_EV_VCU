
#include "ADS1115.h"
#include <stdint.h>

class ADS_Sensor
{
  private:
    ADS1115::ADS1115_ADC adc;
    ADS1115::DataRate dataRate;
    ADS1115::Pga sensorGain;
    int16_t values[4];
    float voltages[4];
    int currentShot;
    bool isConnected;
    bool initialized;
    bool errorMode;
    bool isReady;
    bool wasShooting;
    float sampleTime;
    float lastTime;
    float reconnectTime;

    void PrintADSFailure(ADS1115::Status status);
    void ResetValues();
    void Reconnect();

  public:
    ADS_Sensor();
    ADS_Sensor(ADS1115::I2CAddress address, ADS1115::DataRate dataRate, ADS1115::Pga sensorGain);
    void Tick();
    void PrintDebug();
    bool HasErrors();
    float GetVoltage(int channel);
};