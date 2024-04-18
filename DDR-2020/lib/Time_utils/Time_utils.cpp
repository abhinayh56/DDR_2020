#include "Time_utils.h"

Time_utils::Time_utils(float freq){
    SampleFrequency = freq;
    dt              = 1.0/SampleFrequency;
    loop_timer      = 1000000.0f*dt;
    t_last_micros   = micros();
}

float Time_utils::get_t_now_micros(){
    return micros();
}

float Time_utils::get_t_now_millis(){
    return micros();
}

float Time_utils::get_sample_freq(){
    return SampleFrequency;
}

float Time_utils::get_loop_time(){
    return loop_timer;
}

float Time_utils::get_dt(){
    return dt;
}

void Time_utils::wait(){
    while((micros()- t_last_micros) < loop_timer){};
    t_last_micros = micros();
}

float Time_utils::get_t_last_micros(){
    return t_last_micros;
}

void Time_utils::set_t_last_micros(){
    t_last_micros = micros();
}

// ------------------------------------------------------------------------------

Clock_utils::Clock_utils(){
}

float Clock_utils::get_t_now_millis(){
    return millis() - t_init_millis;
}

float Clock_utils::get_t_now_micros(){
    return micros() - t_init_micros;
}

void Clock_utils::init_once(){
    if(start==true){
        t_init_micros = micros();
        t_init_millis = t_init_micros/1000.0;
        start = false;
    }
}
