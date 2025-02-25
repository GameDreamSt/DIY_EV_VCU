
#include "Time.h"
#include <Arduino.h>

unsigned long lastMicroSeconds = 0;
float currentDeltaTime = 0;

float GetDeltaTime()
{
    return currentDeltaTime;
}

void TickTime()
{
    unsigned long currentMicroseconds = micros();

    if (currentMicroseconds < lastMicroSeconds)
        currentDeltaTime = 0; // Looped over
    else
        currentDeltaTime = (currentMicroseconds - lastMicroSeconds) / 1000000.0f;

    lastMicroSeconds = currentMicroseconds;
}