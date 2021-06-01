#include "toolbox.h"

float Abs1(float value)
{
if (value >= 0)
return value;
else return -value;
}

float Max1(float value, float value2)
{
if (value > value2)
return value;
else
return value2;
}

float Min1(float value, float value2)
{
if (value < value2)
return value;
else
return value2;
}

float LimitToInterval1(float value, float lowLimit, float highLimit)
{
if (value > highLimit)
value = highLimit;
else if (value < lowLimit)
value = lowLimit;

return value;
}

float RadianToDegree(float value)
{
return value / PI * 180.0;
}

float DegreeToRadian(float value)
{
    return value * PI / 180.0;
}

