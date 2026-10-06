#ifndef ENCODER_INTERFACE_H
#define ENCODER_INTERFACE_H

#include <Arduino.h>
#include "../../config/Config.h"

class Encoder_interface
{
public:
    Encoder_interface();

    void config();

    static inline void encoder_1_int2();

    static inline void encoder_1_int3();

    static inline void encoder_2_int4();

    static inline void encoder_2_int5();

    static volatile long long count1;
    static volatile long long count2;
};

#endif // ENCODER_INTERFACE_H
