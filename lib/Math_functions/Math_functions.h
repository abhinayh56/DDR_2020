#ifndef BASIC_FUNCTIONS_H
#define BASIC_FUNCTIONS_H

#include "math.h"
#include "Constants.h"

template <typename T>
class Math_functions
{
public:
	T saturate(T x, T x_min, T x_max);
	T mod(T x);
	T sign(T x);
	int great_int(T x);
	T ceil(T x);
	T floor(T x);
	T wrap(T x, T a, T b);
	T apply_deadband(T x, T deadband);
	T apply_deadband2(T x, T deadband);

	T rad_2_deg(T x);
	T deg_2_rad(T x);
	T rps_2_rpm(T rps);
	T rpm_2_rps(T rpm);

	T linear_map(T x, T x1, T x2, T y1, T y2);
	T poly_map_deg1(T x, T a0, T a1);
	T poly_map_deg2(T x, T a0, T a1, T a2);
	T poly_map_deg3(T x, T a0, T a1, T a2, T a3);
	T poly_map_deg4(T x, T a0, T a1, T a2, T a3, T a4);
	T poly_map_deg5(T x, T a0, T a1, T a2, T a3, T a4, T a5);
	T poly_map_deg6(T x, T a0, T a1, T a2, T a3, T a4, T a5, T a6);
	T poly_map_degn(T x, T a[], int n);

	bool points_AB_line_same_side(T x, T y, T x0, T y0, T x1, T y1, T x2, T y2);
	bool points_A0_line_same_side(T x, T y, T x1, T y1, T x2, T y2);
};

template <typename T>
class Circle
{
public:
	Circle();
	~Circle();
	bool inside(T xc, T yc, T r, T x0, T y0);
	void intersection_line(T xc, T yc, T r, T x1, T y1, T x2, T y2, T *x10, T *y10, T *x20, T *y20);
	void intersection_circle(T c1x, T c1y, T r1, T c2x, T c2y, T r2, T *x01, T *y01, T *x02, T *y02);
};

#include "Math_functions.tpp"

#endif