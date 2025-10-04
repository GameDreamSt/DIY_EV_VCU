
#ifndef Max
#define Max(a,b) a > b ? a : b
#endif

#ifndef Min
#define Min(a,b) a < b ? a : b
#endif

#ifndef Clamp
#define Clamp(x, a, b) Max(a, Min(b, x))
#endif

#ifndef Saturate
#define Saturate(a) Max(0, Min(1, x))
#endif

#ifndef FloatAbout
#define FloatAbout(value, target, deviation) value > target - deviation && value < target + deviation
#endif