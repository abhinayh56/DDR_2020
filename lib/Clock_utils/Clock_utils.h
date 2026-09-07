#ifndef CLOCK_UTILS
#define CLOCK_UTILS

#include <Arduino.h>

class Clock_utils
{
public:
	Clock_utils();
	void init();
	uint64_t get_t_now_s();
	double get_t_now_ms();
	double get_t_now_us();
	void reset();

private:
	unsigned long t_init_us = 0;
	bool start = true;
};

#endif
