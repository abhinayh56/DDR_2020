#ifndef TIMER_UTILS
#define TIMER_UTILS

#include <Arduino.h>

class Timer_utils{
    public:
        Timer_utils(double freq_=100.0);
        void init(double freq_=100.0);
        unsigned long get_t_now_micros();
        unsigned long get_t_now_millis();
        double get_timer_freq();
        unsigned long get_time_us();
        double get_dt();
        void wait();
        unsigned long get_t_last_micros();
        void set_t_last_micros();

    private:
        double freq = 400.0;
        double dt = 1.0/freq;
        unsigned long loop_time_us = (unsigned long)(1000000.0f*dt);
        unsigned long t_last_us = 0;
};

#endif
