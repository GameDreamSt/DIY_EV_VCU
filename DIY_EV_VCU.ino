
#include "src/EVLib/SerialReader.h"
#include "src/EVLib/SerialPrint.h"
#include "src/EVLib/EVTime.h"
#include "src/VCU.h"
#include "src/Commands.h"
#include "src/VAG_PTC_LIN.h"

void setup()
{
    Serial.begin(115200);
    analogReadResolution(12);
    analogSetAttenuation(ADC_11db);

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
