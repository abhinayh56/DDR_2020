#ifndef TIME_UTILS
#define TIME_UTILS

#include <Arduino.h>

class Time_utils{
    public:
        Time_utils(float freq);
        float get_t_now_micros();
        float get_t_now_millis();
        float get_sample_freq();
        float get_loop_time();
        float get_dt();
        void wait();
        float get_t_last_micros();
        void set_t_last_micros();

    private:
        float SampleFrequency = 400.0;
        float dt              = 1.0/SampleFrequency;
        float loop_timer      = 1000000.0f*dt;
        float t_last_micros   = 0.0;
};

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
