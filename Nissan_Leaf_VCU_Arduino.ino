
#include "Pinout.h"
#include "SerialPrint.h"
#include "SerialReader.h"
#include "Time.h"
#include "Timer.h"
#include "VCU.h"
#include <SPI.h>

bool state;
Timer ledTimer = Timer(0.1f);
Timer led2Timer = Timer(0.5f);

void setup()
{
    Serial.begin(9600);
    TickTime(); // Initialize delta time
    InitializeSerialReader();
    VCU::Initialize();
    pinMode(DEBUG_LED, OUTPUT);
}

void loop()
{
    TickTime(); // Recalculate delta time
    VCU::Tick();
    TickSerialReader();
    TickSerialWriter();

    if ((VCU::IsIgnitionOn() && led2Timer.HasTriggered()) || (!VCU::IsIgnitionOn() && ledTimer.HasTriggered()))
    {
        state = !state;
        digitalWrite(DEBUG_LED, state);
    }
}
