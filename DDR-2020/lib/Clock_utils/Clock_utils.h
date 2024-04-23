#ifndef CLOCK_UTILS
#define CLOCK_UTILS

#include <Arduino.h>

class Clock_utils{
    public:
        Clock_utils();
        float get_t_now_millis();
        float get_t_now_micros();
        void init_once();
    private:
        float t_init_micros = 0.0;
        float t_init_millis = 0.0;
        float t_now = 0.0;
        bool start = true;
};

#endif
