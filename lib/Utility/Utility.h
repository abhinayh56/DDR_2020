#ifndef CONTROL_SYSTEM_PID_UTILITY_H
#define CONTROL_SYSTEM_PID_UTILITY_H

template <typename T>
inline T saturate(T x, T x_min, T x_max)
{
    if (x < x_min)
    {
        return x_min;
    }
    else if (x > x_max)
    {
        return x_max;
    }
    else
    {
        return x;
    }
}

#endif // CONTROL_SYSTEM_PID_UTILITY_H