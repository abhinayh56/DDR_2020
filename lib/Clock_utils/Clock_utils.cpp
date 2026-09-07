#include "Clock_utils.h"

Clock_utils::Clock_utils()
{
}

void Clock_utils::init()
{
	unsigned long t_init_us_temp = micros();
	if (start == true)
	{
		t_init_us = t_init_us_temp;
		start = false;
	}
}

double Clock_utils::get_t_now_s()
{
	return ((double)(micros() - t_init_us)) / 1000000.0;
}

double Clock_utils::get_t_now_ms()
{
	return ((double)(micros() - t_init_us)) / 1000.0;
}

double Clock_utils::get_t_now_us()
{
	return (double)(micros() - t_init_us);
}

void Clock_utils::reset()
{
	t_init_us = micros();
	start = false;
}