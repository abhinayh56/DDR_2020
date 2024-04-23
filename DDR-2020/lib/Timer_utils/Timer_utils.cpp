#include "Timer_utils.h"

Timer_utils::Timer_utils(double freq_){
    freq = freq_;
    dt = 1.0/freq;
    loop_time_us = 1000000.0f*dt;
    t_last_us = micros();
}

void Timer_utils::init(double freq_){
    freq = freq_;
    dt = 1.0/freq;
    loop_time_us = (unsigned long)(1000000.0f*dt);
    t_last_us = micros();
}
unsigned long Timer_utils::get_t_now_micros(){
    return micros();
}

unsigned long Timer_utils::get_t_now_millis(){
    return micros();
}

double Timer_utils::get_timer_freq(){
    return freq;
}

unsigned long Timer_utils::get_time_us(){
    return loop_time_us;
}

double Timer_utils::get_dt(){
    return dt;
}

void Timer_utils::wait(){
    while((micros() - t_last_us) < loop_time_us){};
    t_last_us = micros();
}

unsigned long Timer_utils::get_t_last_micros(){
    return t_last_us;
}

void Timer_utils::set_t_last_micros(){
    t_last_us = micros();
}
