
#include "ADS_Sensor.h"
#include "SerialPrint.h"
#include <Wire.h>
#include <string>

using namespace std;
using namespace ADS1115;

float SamplesToSeconds(DataRate dataRate);

void ADS_Sensor::ResetValues()
{
    values[0] = values[1] = values[2] = values[3] = 0;
    voltages[0] = voltages[1] = voltages[2] = voltages[3] = 0;
}

ADS_Sensor::ADS_Sensor()
{
    ResetValues();
    initialized = false;
    errorMode = false;
    wasShooting = false;
    isReady = false;
    sampleTime = 0;
    currentShot = 0;
    reconnectTime = 16;
}

void ADS_Sensor::Reconnect()
{
    errorMode = false;

    if (!adc.isConnected())
    {
        isConnected = false;
        errorMode = true;
        ResetValues();
        PrintSerialMessage("ADS1115 is not connected!");
        return;
    }
    else
        isConnected = true;

    PrintADSFailure(adc.init());
    initialized = !errorMode;

    if (errorMode)
        return;

    adc.setDataRate(dataRate);
    adc.setPga(sensorGain);
    PrintADSFailure(adc.uploadConfig());
}

ADS_Sensor::ADS_Sensor(I2CAddress address, DataRate IN_dataRate, Pga IN_sensorGain) : ADS_Sensor()
{
    dataRate = IN_dataRate;
    sensorGain = IN_sensorGain;

    adc = ADS1115_ADC(address);
    sampleTime = ceil(SamplesToSeconds(dataRate));

    Wire.begin();

    Reconnect();
}

bool ADS_Sensor::HasErrors()
{
    return errorMode;
}

float ADS_Sensor::GetVoltage(int channel)
{
    if (channel > 3 || channel < 0)
    {
        PrintSerialMessage("ADS get voltage channel " + ToString(channel) + " is out of range!");
        return 0;
    }

    return voltages[channel];
}

void ADS_Sensor::Tick()
{
    if (errorMode)
    {
        if (lastTime > millis())
            return;

        Reconnect();

        if (errorMode)
        {
            lastTime = millis() + reconnectTime;
            reconnectTime *= 2;
        }
        else
            reconnectTime = 16;
        return;
    }

    PrintADSFailure(adc.isConversionReady(isReady));
    if (!isReady)
        return;

    if (lastTime > millis())
        return;
    lastTime = millis() + sampleTime;

    if (wasShooting)
    {
        PrintADSFailure(adc.readConversion(values[currentShot]));
        voltages[currentShot] = adc.convertToVoltage(values[currentShot]);
        currentShot = ++currentShot % 4;
    }
    else
    {
        PrintADSFailure(adc.startSingleShotConversion(currentShot));
    }
    wasShooting = !wasShooting;
}

void ADS_Sensor::PrintDebug()
{
    if (errorMode)
        return;

    PrintSerialMessage("Voltages: P0: " + FloatToString(voltages[0], 3) + "V | P1: " + FloatToString(voltages[1], 3) +
                   "V | P2: " + FloatToString(voltages[2], 3) + "V | P3: " + FloatToString(voltages[3], 3) + "V");
}

void ADS_Sensor::PrintADSFailure(ADS1115::Status status)
{
    if (status == Status::Ok)
        return;

    if (errorMode)
        return;

    errorMode = true;
    ResetValues();

    string errorString = "";

    switch (status)
    {
    case Status::DataTooLong:
        errorString = "Data too long";
        break;
    case Status::ReceivedNackOnTxAddress:
        errorString = "Received Nack On Tx Address";
        break;
    case Status::ReceivedNackOnTxData:
        errorString = "Received Nack On Tx Data";
        break;
    case Status::OtherError:
        errorString = "Other error";
        break;
    case Status::Timeout:
        errorString = "Timeout";
        break;
    case Status::FailedToRequestBytes:
        errorString = "Failed To Request Bytes";
        break;
    case Status::ChannelOutOfRange:
        errorString = "Channel Out Of Range";
        break;
    default:
        errorString = "Unknown error";
        break;
    }

    PrintSerialMessage("ADS1115 failure - " + errorString);
}

float SamplesToSeconds(DataRate dataRate)
{
    switch (dataRate)
    {
    case DataRate::SPS_8:
        return 1 / 8.0f; // 125ms
    case DataRate::SPS_16:
        return 1 / 16.0f; // 62.5ms
    case DataRate::SPS_32:
        return 1 / 32.0f; // 31.3 ms
    case DataRate::SPS_64:
        return 1 / 64.0f; // 15.62 ms
    case DataRate::SPS_128:
        return 1 / 128.0f; // 7.81 ms
    case DataRate::SPS_250:
        return 1 / 250.0f; // 4 ms
    case DataRate::SPS_475:
        return 1 / 475.0f; // 2.1 ms
    case DataRate::SPS_860:
        return 1 / 860.0f; // 1.2 ms
    }
}