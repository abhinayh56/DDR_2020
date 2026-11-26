#include "Wheel_odom.h"

template <typename T>
Wheel_odom<T>::Wheel_odom()
{
}

template <typename T>
void Wheel_odom<T>::set_param(long N_, T L_, T r_)
{
	N = N_;
	r = r_;
	L = L_;
}

template <typename T>
void Wheel_odom<T>::set_dt(T dt_)
{
	dt = dt_;
}

template <typename T>
long Wheel_odom<T>::get_PPR()
{
	return N;
}

template <typename T>
T Wheel_odom<T>::get_r()
{
	return r;
}

template <typename T>
T Wheel_odom<T>::get_L()
{
	return L;
}

template <typename T>
T Wheel_odom<T>::get_dt()
{
	return dt;
}

template <typename T>
void Wheel_odom<T>::get_wheel_speed(T &wl, T &wr)
{
	wr = w_R;
	wl = w_L;
}

template <typename T>
void Wheel_odom<T>::get_twist(T &vc, T &wc)
{
	vc = v;
	wc = w;
}

template <typename T>
void Wheel_odom<T>::get_pose(T &xc, T &yc, T &thc)
{
	xc = x;
	yc = y;
	thc = th;
}

template <typename T>
void Wheel_odom<T>::update(long long nl, long long nr)
{
	N_L = nl;
	N_R = nr;

	w_L = const_math_2pi * (T)(N_L - N_L_pre) / ((T)N * dt);
	w_R = const_math_2pi * (T)(N_R - N_R_pre) / ((T)N * dt);
	v = (r / 2.0) * (w_L + w_R);
	w = (r / L) * (w_R - w_L);

	T dl = (const_math_pi * r * (T)((N_R - N_R_pre) + (N_L - N_L_pre))) / ((T)N);
	T dth = (const_math_2pi * r * (T)((N_R - N_R_pre) - (N_L - N_L_pre))) / ((T)N * L);

	th += dth;
	th = math.wrap(th, -const_math_pi, const_math_pi);
	x += dl * cos(th);
	y += dl * sin(th);

	N_L_pre = N_L;
	N_R_pre = N_R;
}