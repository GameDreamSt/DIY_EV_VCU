
#include <cstdint>

#define deltaTime GetDeltaTime()

extern "C" float GetDeltaTime();
extern "C" float GetTime();
extern "C" uint64_t GetTimeMicroseconds();
extern "C" void TickTime();