#include "PID_P.h"

template <typename T>
PID_P<T>::PID_P()
{
    dt = 0.0;
    Kp = 0.0;
    Ki = 0.0;
    Kd = 0.0;
    fc = 0.0;
    I_max = 0.0;
    u_max = 0.0;
    d_filter = false;
    e_k_1 = 0.0;
    P = 0.0;
    I = 0.0;
    D = 0.0;
    u = 0.0;
    u_ff = 0.0;
    start = true;
}

template <typename T>
void PID_P<T>::init(T dt_, T Kp_, T Ki_, T Kd_, T I_max_, T u_max_, bool d_filter_, T fc_)
{
    set_param(dt_, Kp_, Ki_, Kd_, I_max_, u_max_, d_filter_, fc_);
    start = true;
}

template <typename T>
void PID_P<T>::set_param(T dt_, T Kp_, T Ki_, T Kd_, T I_max_, T u_max_, bool d_filter_, T fc_)
{
    dt = dt_;
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;
    I_max = I_max_;
    u_max = u_max_;
    d_filter = d_filter_;
    fc = fc_;
    lpf.set_param(fc, dt);
}

template <typename T>
T PID_P<T>::update(T x_0, T x, T u_ff_)
{
    T e_k = x_0 - x;
    u_ff = u_ff_;

    P = Kp * e_k;

    I = I + Ki * e_k * dt;
    I = saturate(I, -I_max, I_max);

    if (start == true)
    {
        D = 0.0;
        start = false;
    }
    else
    {
        D = Kd * (e_k - e_k_1) / dt;
    }

    if (d_filter == true)
    {
        D = lpf.update(D);
    }
    e_k_1 = e_k;
    u = u_ff + P + I + D;
    u = saturate<T>(u, -u_max, u_max);
    return u;
}

template <typename T>
void PID_P<T>::reset()
{
    e_k_1 = 0.0;
    P = 0.0;
    I = 0.0;
    D = 0.0;
    u = 0.0;
    u_ff = 0.0;
    start = true;
    lpf.reset();
}

template <typename T>
void PID_P<T>::merge(T u_k_1_)
{
}

template <typename T>
void PID_P<T>::set_dt(T dt_)
{
    dt = dt_;
    lpf.set_param(fc, dt);
}

template <typename T>
void PID_P<T>::set_Kp(T Kp_)
{
    Kp = Kp_;
}

template <typename T>
void PID_P<T>::set_Ki(T Ki_)
{
    Ki = Ki_;
}

template <typename T>
void PID_P<T>::set_Kd(T Kd_)
{
    Kd = Kd_;
}

template <typename T>
void PID_P<T>::set_I_max(T I_max_)
{
    I_max = I_max_;
}

template <typename T>
void PID_P<T>::set_u_max(T u_max_)
{
    u_max = u_max_;
}

template <typename T>
void PID_P<T>::set_d_filter(bool d_filter_)
{
    d_filter = d_filter_;
}

template <typename T>
void PID_P<T>::set_fc(T fc_)
{
    fc = fc_;
    lpf.set_param(fc, dt);
}

template <typename T>
void PID_P<T>::set_ff(T u_ff_)
{
    u_ff = u_ff_;
}

template <typename T>
T PID_P<T>::get_dt()
{
    return dt;
}

template <typename T>
T PID_P<T>::get_Kp()
{
    return Kp;
}

template <typename T>
T PID_P<T>::get_Ki()
{
    return Ki;
}

template <typename T>
T PID_P<T>::get_Kd()
{
    return Kd;
}

template <typename T>
T PID_P<T>::get_I_max()
{
    return I_max;
}

template <typename T>
T PID_P<T>::get_u_max()
{
    return u_max;
}

template <typename T>
bool PID_P<T>::get_d_filter()
{
    return d_filter;
}

template <typename T>
T PID_P<T>::get_fc()
{
    return fc;
}

template <typename T>
T PID_P<T>::get_ff()
{
    return u_ff;
}

template <typename T>
T PID_P<T>::get_P()
{
    return P;
}

template <typename T>
T PID_P<T>::get_I()
{
    return I;
}

template <typename T>
T PID_P<T>::get_D()
{
    return D;
}

template <typename T>
T PID_P<T>::get_u()
{
    return u;
}

template <typename T>
T PID_P<T>::get_e_k_1()
{
    return e_k_1;
}
