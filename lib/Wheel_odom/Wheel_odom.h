#ifndef WHEEL_ODOM_H
#define WHEEL_ODOM_H

#include <math.h>
#include "Math_functions.h"

template <typename T>
class Wheel_odom
{
public:
	Wheel_odom();
	void set_param(long N_, T r_, T L_);
	void set_dt(T dt_);
	long get_PPR();
	T get_r();
	T get_L();
	T get_dt();
	void get_wheel_speed(T &wr, T &wl);
	void get_twist(T &vc, T &wc);
	void get_pose(T &xc, T &yc, T &thc);
	void update(long long nr, long long nl);

private:
	unsigned long N = 133600;
	T r = 0.0625;
	T L = 0.265;

	T dt = 0.004;
	long long N_L, N_R;
	long long N_L_pre, N_R_pre;
	T w_L, w_R;
	T v;
	T w;
	T x;
	T y;
	T th;

	Math_functions<T> math;
};

#include "Wheel_odom.tpp"

#endif
