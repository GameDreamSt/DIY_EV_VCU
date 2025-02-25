
#include "Time.h"
#include "Timer.h"

Timer::Timer(float fireInterval)
{
    triggerTime = fireInterval;
    currentTime = 0;
}

bool Timer::HasTriggered()
{
    currentTime += deltaTime;
    if (currentTime > triggerTime)
    {
        currentTime = 0;
        return true;
    }

    return false;
}