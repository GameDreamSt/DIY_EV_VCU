
#include "src/EVLib/SerialReader.h"
#include "src/EVLib/SerialPrint.h"
#include "src/EVLib/EVTime.h"
#include "src/VCU.h"
#include "src/Commands.h"

void setup()
{
    Serial.begin(115200);
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

    TickTime(); // Initialize delta time
    InitializeSerialReader();
    commands::InitializeCommands();
    VCU::Initialize();
}

void loop()
{
    TickTime(); // Recalculate delta time
    VCU::Tick();
    TickSerialReader();
    TickSerialWriter();
}
