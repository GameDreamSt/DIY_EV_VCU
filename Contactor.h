
#include "Pinout.h"

enum ContactorTest
{
    None = 0,
    Precharge = PIN_PRECHARGE,
    Negative = PIN_NEG_CONTACTOR,
    Positive = PIN_POS_CONTACTOR,
    Vacuum = PIN_VACUUM_PUMP,
    Water = PIN_WATER_PUMP,
    DCDC = PIN_DCDC_ENABLE,
};