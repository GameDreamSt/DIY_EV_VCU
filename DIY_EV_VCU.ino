
#include "src/EVLib/SerialReader.h"
#include "src/EVLib/SerialPrint.h"
#include "src/EVLib/EVTime.h"
#include "src/EVLib/Sensor.h"
#include "src/VCU.h"
#include "src/Commands.h"
#include "src/VAG_PTC_LIN.h"
#include "src/pinout.h"

void setup()
{
    Serial.begin(115200);
    analogReadResolution(ANALOG_RES);
    analogSetAttenuation(ADC_11db);
    SetSensorsAnalogResolution(ANALOG_RES);

    TickTime(); // Initialize delta time
    InitializeSerialReader();
    commands::InitializeCommands();
    VCU::Initialize();
    InitializePTC();
}

void loop()
{
    TickTime(); // Recalculate delta time
    VCU::Tick();
    TickPTC();
    TickSerialReader();
    TickSerialWriter();
}
