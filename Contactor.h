
#include "Pinout.h"

enum ContactorTest
{
    None = 0,
    Precharge = PIN_PRECHARGE,
    Negative = -1, // Always on for now!
    Positive = PIN_POS_CONTACTOR,
    Motor = PIN_INV_POWER,
};