#include "Clock_utils.h"

Clock_utils::Clock_utils(){
}

void Clock_utils::init(){
    if(start==true){
        t_init_us = micros();
        t_init_ms = t_init_us/1000.0;
        start = false;
    }
}

double Clock_utils::get_t_now_s(){
    return ((double)(micros() - t_init_us)) / 1000000.0;
}

double Clock_utils::get_t_now_ms(){
    return (double)(millis() - t_init_ms);
}

double Clock_utils::get_t_now_us(){
    return (double)(micros() - t_init_us);
}

void Clock_utils::reset(){
    t_init_us = micros();
    t_init_ms = t_init_us/1000.0;
    start = false;
}