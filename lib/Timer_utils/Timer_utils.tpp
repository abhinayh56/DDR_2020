#include "Timer_utils.h"

template <typename T>
Timer_utils<T>::Timer_utils(T freq_)
{
	freq = freq_;
	dt = 1.0 / freq;
	loop_time_us = 1000000.0f * dt;
	t_last_us = micros();
}

template <typename T>
void Timer_utils<T>::init(T freq_)
{
	freq = freq_;
	dt = 1.0 / freq;
	loop_time_us = (unsigned long)(1000000.0f * dt);
	t_last_us = micros();
}

template <typename T>
unsigned long Timer_utils<T>::get_t_now_micros()
{
	return micros();
}

template <typename T>
unsigned long Timer_utils<T>::get_t_now_millis()
{
	return micros();
}

template <typename T>
T Timer_utils<T>::get_timer_freq()
{
	return freq;
}

template <typename T>
unsigned long Timer_utils<T>::get_time_us()
{
	return loop_time_us;
}

template <typename T>
T Timer_utils<T>::get_dt()
{
	return dt;
}

template <typename T>
void Timer_utils<T>::sleep()
{
	while ((micros() - t_last_us) < loop_time_us)
	{
	};
	t_last_us = micros();
}

template <typename T>
unsigned long Timer_utils<T>::get_t_last_micros()
{
	return t_last_us;
}

template <typename T>
void Timer_utils<T>::set_t_last_micros()
{
	t_last_us = micros();
}
