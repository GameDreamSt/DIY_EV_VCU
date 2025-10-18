
#include "SerialReader.h"
#include "SerialPrint.h"
#include "Time.h"
#include "VCU.h"

void setup()
{
    Serial.begin(115200);
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    TickTime(); // Initialize delta time
    InitializeSerialReader();
    VCU::Initialize();
}

void loop()
{
    TickTime(); // Recalculate delta time
    VCU::Tick();
    TickSerialReader();
    TickSerialWriter();
}
