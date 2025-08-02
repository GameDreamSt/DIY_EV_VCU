
#include "Time.h"

uint64_t lastMicroSeconds = 0;
uint64_t microsecondsFromStart = 0;
float currentDeltaTime = 0;

float GetDeltaTime()
{
    return currentDeltaTime;
}

float GetTime()
{
    return microsecondsFromStart / 1000000.0f;
}

uint64_t GetTimeMicroseconds()
{
    return microsecondsFromStart;
}

void TickTime()
{
    uint64_t currentMicroseconds = micros();
    uint64_t microsecondsDelta = currentMicroseconds - lastMicroSeconds;

    if (currentMicroseconds < lastMicroSeconds) // Looped over
    {
        currentDeltaTime = 0;
        microsecondsDelta = 0;
    }
    else
        currentDeltaTime = microsecondsDelta / 1000000.0f;

    microsecondsFromStart += microsecondsDelta;
    lastMicroSeconds = currentMicroseconds;
}