#ifndef TIMER_UTILS
#define TIMER_UTILS

#include <Arduino.h>

template <typename T>
class Timer_utils
{
public:
	Timer_utils(T freq_ = 100.0);
	void init(T freq_ = 100.0);
	unsigned long get_t_now_micros();
	unsigned long get_t_now_millis();
	T get_timer_freq();
	unsigned long get_time_us();
	T get_dt();
	void sleep();
	unsigned long get_t_last_micros();
	void set_t_last_micros();

private:
	T freq = 100.0;
	T dt = 1.0 / freq;
	unsigned long loop_time_us = (unsigned long)(1000000.0f * dt);
	unsigned long t_last_us = 0;
};

#include "Timer_utils.tpp"

#endif
