#include "LPF_1.h"

template <typename T>
LPF_1<T>::LPF_1()
{
	reset();
}

template <typename T>
void LPF_1<T>::config(T fc_, T dt_)
{
	set_param(fc_, dt_);
}

template <typename T>
void LPF_1<T>::set_param(T fc_, T dt_)
{
	fc = fc_;
	dt = dt_;
	tau = 1.0 / (6.28318530717958647692528676655900576 * fc);
	alpha = dt / (dt + tau);
}

template <typename T>
T LPF_1<T>::update(T x_k)
{
	T y_k = 0.0;

	if (start == false)
	{
		start = true;
		y_k = x_k;
	}
	else
	{
		y_k = alpha * x_k + (1.0 - alpha) * y_k_1;
	}
	y_k_1 = y_k;
	return y_k;
}

template <typename T>
void LPF_1<T>::reset()
{
	y_k_1 = 0.0;
	start = false;
}

template <typename T>
void LPF_1<T>::set_fc(T fc_)
{
	set_param(fc_, dt);
}

template <typename T>
void LPF_1<T>::set_tau(T tau_)
{
	tau = tau_;
	fc = 1.0 / (6.28318530717958647692528676655900576 * tau);
	alpha = dt / (dt + tau);
}

template <typename T>
void LPF_1<T>::set_dt(T dt_)
{
	set_param(fc, dt_);
}

template <typename T>
void LPF_1<T>::set_alpha(T alpha_)
{
	alpha = alpha_;
	tau = ((1.0 - alpha) * dt) / alpha;
	fc = 1.0 / (6.28318530717958647692528676655900576 * tau);
}

template <typename T>
T LPF_1<T>::get_fc()
{
	return fc;
}

template <typename T>
T LPF_1<T>::get_tau()
{
	return tau;
}

template <typename T>
T LPF_1<T>::get_dt()
{
	return dt;
}

template <typename T>
T LPF_1<T>::get_alpha()
{
	return alpha;
}

template <typename T>
T LPF_1<T>::get_fs()
{
   return 1.0 / dt;
}
