#include "Clock_utils.h"

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