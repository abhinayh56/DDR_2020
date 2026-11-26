#include "Diff_drive_unicycle.h"

template <typename T>
Diff_drive_unicycle<T>::Diff_drive_unicycle()
{
}

template <typename T>
void Diff_drive_unicycle<T>::set_param(T L_, T r_)
{
	L = L_;
	r = r_;
}

template <typename T>
void Diff_drive_unicycle<T>::set_v_max(T V_max_)
{
	V_max = V_max_;
	w_lr_max = V_max / r;
	W_max = 2.0 * V_max / L;
}

template <typename T>
void Diff_drive_unicycle<T>::set_w_max(T W_max_)
{
	W_max = W_max_;
	V_max = W_max * L / 2.0;
	w_lr_max = V_max / r;
}

template <typename T>
T Diff_drive_unicycle<T>::get_v_max()
{
	return V_max;
}

template <typename T>
T Diff_drive_unicycle<T>::get_w_max()
{
	return W_max;
}

template <typename T>
T Diff_drive_unicycle<T>::get_wlr_max()
{
	return w_lr_max;
}

template <typename T>
void Diff_drive_unicycle<T>::set_r(T r_)
{
	r = r_;
}

template <typename T>
void Diff_drive_unicycle<T>::set_L(T L_)
{
	L = L_;
}

template <typename T>
T Diff_drive_unicycle<T>::get_r()
{
	return r;
}

template <typename T>
T Diff_drive_unicycle<T>::get_L()
{
	return L;
}

template <typename T>
void Diff_drive_unicycle<T>::uni2ddr(T Vc, T Wc, T &wl, T &wr)
{
	wl = (Vc - Wc * L * 0.5) / r;
	wr = (Vc + Wc * L * 0.5) / r;
}

template <typename T>
T Diff_drive_unicycle<T>::get_wr(T Vc, T Wc)
{
	return (Vc + Wc * L * 0.5) / r;
}

template <typename T>
T Diff_drive_unicycle<T>::get_wl(T Vc, T Wc)
{
	return (Vc - Wc * L * 0.5) / r;
}

template <typename T>
void Diff_drive_unicycle<T>::ddr2uni(T wl, T wr, T &Vc, T &Wc)
{
	Vc = (wr + wl) * r * 0.5;
	Wc = (wr - wl) * r / L;
}

template <typename T>
T Diff_drive_unicycle<T>::get_Vc(T wl, T wr)
{
	return (wr + wl) * r * 0.5;
}

template <typename T>
T Diff_drive_unicycle<T>::get_Wc(T wl, T wr)
{
	return (wr - wl) * r / L;
}

template <typename T>
void Diff_drive_unicycle<T>::update_domain_vw(T V_in, T W_in, T &V_out, T &W_out)
{
	T V_n = V_in;
	T W_n = W_in;

	int region = 0;

	if (W_in >= W_max)
	{
		region = 5;
	}
	else if (W_in <= (-W_max))
	{
		region = 6;
	}
	else if (W_in >= 0)
	{
		if (V_in >= 0)
		{
			if (math_fun.points_A0_line_same_side(V_in, W_in, V_max, 0.0, 0.0, W_max) == false)
			{
				region = 1;
			}
		}
		else
		{
			if (math_fun.points_A0_line_same_side(V_in, W_in, -V_max, 0.0, 0.0, W_max) == false)
			{
				region = 2;
			}
		}
	}
	else
	{
		if (V_in >= 0)
		{
			if (math_fun.points_A0_line_same_side(V_in, W_in, V_max, 0.0, 0.0, -W_max) == false)
			{
				region = 4;
			}
		}
		else
		{
			if (math_fun.points_A0_line_same_side(V_in, W_in, -V_max, 0.0, 0.0, -W_max) == false)
			{
				region = 3;
			}
		}
	}

	switch (region)
	{
	case 0:
		V_n = V_in;
		W_n = W_in;
		break;
	case 1:
		V_n = math_fun.linear_map(W_in, 0.0, W_max, V_max, 0.0);
		W_n = W_in;
		break;
	case 2:
		V_n = math_fun.linear_map(W_in, 0.0, W_max, -V_max, 0.0);
		W_n = W_in;
		break;
	case 3:
		V_n = math_fun.linear_map(W_in, 0.0, -W_max, -V_max, 0.0);
		W_n = W_in;
		break;
	case 4:
		V_n = math_fun.linear_map(W_in, 0.0, -W_max, V_max, 0.0);
		W_n = W_in;
		break;
	case 5:
		V_n = 0.0;
		W_n = W_in;
		break;
	case 6:
		V_n = 0.0;
		W_n = W_in;
		break;
	default:
		V_n = V_in;
		W_n = W_in;
		break;
	}

	V_out = V_n;
	W_out = W_n;
}