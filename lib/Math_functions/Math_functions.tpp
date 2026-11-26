#include "Math_functions.h"

template <typename T>
T Math_functions<T>::saturate(T x, T x_min, T x_max)
{
	if (x < x_min)
	{
		return x_min;
	}
	else if (x > x_max)
	{
		return x_max;
	}
	else
	{
		return x;
	}
}

template <typename T>
T Math_functions<T>::mod(T x)
{
	if (x < 0)
	{
		return -x;
	}
	else
	{
		return x;
	}
}

template <typename T>
T Math_functions<T>::sign(T x)
{
	if (x < 0)
	{
		return -1.0;
	}
	else if (x == 0)
	{
		return 0.0;
	}
	else
	{
		return 1.0;
	}
}

template <typename T>
int Math_functions<T>::great_int(T x)
{
	if (x >= 0)
	{
		return (int)x;
	}
	else
	{
		return (int)x - 1;
	}
}

template <typename T>
T Math_functions<T>::ceil(T x)
{
	if (x >= 0)
	{
		return T((int)x + 1);
	}
	else
	{
		return T((int)x);
	}
}

template <typename T>
T Math_functions<T>::floor(T x)
{
	if (x >= 0)
	{
		return T((int)x);
	}
	else
	{
		return T((int)x - 1);
	}
}

template <typename T>
T Math_functions<T>::wrap(T x, T a, T b)
{
	if (x <= a)
	{
		return x - a + b;
	}
	else if (x > b)
	{
		return x + a - b;
	}
	else
	{
		return x;
	}
}

template <typename T>
T Math_functions<T>::apply_deadband(T x, T deadband)
{
	if (mod(x) < deadband * 0.5)
	{
		return 0;
	}
	else
	{
		return x;
	}
}

template <typename T>
T Math_functions<T>::apply_deadband2(T x, T deadband)
{
	if (mod(x) <= deadband * 0.5)
	{
		return 0;
	}
	else
	{
		if (x > deadband * 0.5)
		{
			return x - deadband * 0.5;
		}
		else
		{
			return x + deadband * 0.5;
		}
	}
}

template <typename T>
T Math_functions<T>::rad_2_deg(T x)
{
	return x * 57.295779513082320876798154814105; // x*180/PI
}

template <typename T>
T Math_functions<T>::deg_2_rad(T x)
{
	return x * 0.01745329251994329576923690768489; // x*PI/180
}

template <typename T>
T Math_functions<T>::rps_2_rpm(T rps)
{
	return rps * 9.54929658551;
}

template <typename T>
T Math_functions<T>::rpm_2_rps(T rpm)
{
	// return rpm/9.54929658551;
	return rpm * 0.10471975512; // for faster execution
}

template <typename T>
T Math_functions<T>::linear_map(T x, T x1, T x2, T y1, T y2)
{
	return y1 + ((y2 - y1) / (x2 - x1)) * (x - x1);
}

template <typename T>
T Math_functions<T>::poly_map_deg1(T x, T a0, T a1)
{
	return a0 + a1 * x;
}

template <typename T>
T Math_functions<T>::poly_map_deg2(T x, T a0, T a1, T a2)
{
	return a0 + a1 * x + a2 * x * x;
}

template <typename T>
T Math_functions<T>::poly_map_deg3(T x, T a0, T a1, T a2, T a3)
{
	return a0 + a1 * x + a2 * x * x + a3 * x * x * x;
}

template <typename T>
T Math_functions<T>::poly_map_deg4(T x, T a0, T a1, T a2, T a3, T a4)
{
	return a0 + a1 * x + a2 * x * x + a3 * x * x * x + a4 * x * x * x * x;
}

template <typename T>
T Math_functions<T>::poly_map_deg5(T x, T a0, T a1, T a2, T a3, T a4, T a5)
{
	return a0 + a1 * x + a2 * x * x + a3 * x * x * x + a4 * x * x * x * x + a4 * x * x * x * x * x;
}

template <typename T>
T Math_functions<T>::poly_map_deg6(T x, T a0, T a1, T a2, T a3, T a4, T a5, T a6)
{
	return a0 + a1 * x + a2 * x * x + a3 * x * x * x + a4 * x * x * x * x + a4 * x * x * x * x * x + a4 * x * x * x * x * x * x;
}

template <typename T>
T Math_functions<T>::poly_map_degn(T x, T a[], int n)
{
	if (n >= 1)
	{
		return 0; // poly_map_degn(T x, T a[], int n-1);
	}
	else
	{
		return a[0];
	}
}

template <typename T>
bool Math_functions<T>::points_AB_line_same_side(T x, T y, T x0, T y0, T x1, T y1, T x2, T y2)
{
	T L0 = (y2 - y1) * (x0 - x1) - (x2 - x1) * (y0 - y1);
	T L = (y2 - y1) * (x - x1) - (x2 - x1) * (y - y1);
	if (L0 * L >= 0)
	{
		return true;
	}
	else
	{
		return false;
	}
}

template <typename T>
bool Math_functions<T>::points_A0_line_same_side(T x, T y, T x1, T y1, T x2, T y2)
{
	T L0 = (y2 - y1) * (-x1) - (x2 - x1) * (-y1);
	T L = (y2 - y1) * (x - x1) - (x2 - x1) * (y - y1);
	if (L0 * L >= 0)
	{
		return true;
	}
	else
	{
		return false;
	}
}

template <typename T>
Circle<T>::Circle()
{
}

template <typename T>
Circle<T>::~Circle()
{
}

template <typename T>
bool Circle<T>::inside(T cx, T cy, T r, T x0, T y0)
{
	if (sqrt((x0 - cx) * (x0 - cx) + (y0 - cy) * (y0 - cy)) <= r)
	{
		return true;
	}
	else
	{
		return false;
	}
}

template <typename T>
void Circle<T>::intersection_line(T cx, T cy, T r, T x1, T y1, T x2, T y2, T *x10, T *y10, T *x20, T *y20)
{
	T a = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
	T b = 2.0 * ((x2 - x1) * (x1 - cx) + (y2 - y1) * (y1 - cy));
	T c = (x1 - cx) * (x1 - cx) + (y1 - cy) * (y1 - cy) - r * r;

	T D = b * b - 4.0 * a * c;

	if (D > 0.0)
	{
		T lambda_1 = (-b + sqrt(D)) / (2.0 * a);
		T lambda_2 = (-b - sqrt(D)) / (2.0 * a);
		*x10 = x1 + lambda_1 * (x2 - x1);
		*y10 = y1 + lambda_1 * (y2 - y1);
		*x20 = x1 + lambda_2 * (x2 - x1);
		*y20 = y1 + lambda_2 * (y2 - y1);
	}
	else if (D == 0.0)
	{
		T lambda_1 = (-b) / (2.0 * a);
		T lambda_2 = (-b) / (2.0 * a);
		*x10 = x1 + lambda_1 * (x2 - x1);
		*y10 = y1 + lambda_1 * (y2 - y1);
		*x20 = x1 + lambda_2 * (x2 - x1);
		*y20 = y1 + lambda_2 * (y2 - y1);
	}
	else
	{
		*x10 = const_math_inf;
		*y10 = const_math_inf;
		*x20 = -const_math_inf;
		*y20 = -const_math_inf;
	}
}

template <typename T>
void Circle<T>::intersection_circle(T c1x, T c1y, T r1, T c2x, T c2y, T r2, T *x01, T *y01, T *x02, T *y02)
{
}