
#include "Time.h"

#include "time.h"
#include "esp_sntp.h"

uint64_t lastMicroSeconds = 0;
uint64_t microsecondsFromStart = 0;
float currentDeltaTime = 0;

struct timeval tv_now;

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
    gettimeofday(&tv_now, NULL);
    uint64_t currentMicroseconds = (uint64_t)tv_now.tv_sec * 1000000L + (uint64_t)tv_now.tv_usec;
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