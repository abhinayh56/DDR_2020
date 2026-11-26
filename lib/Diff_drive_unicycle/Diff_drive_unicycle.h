#ifndef DIFF_DRIVE_UNICYCLE
#define DIFF_DRIVE_UNICYCLE

#include "Math_functions.h"

template <typename T>
class Diff_drive_unicycle
{
public:
	Diff_drive_unicycle();
	void set_param(T r_, T L_);
	void set_v_max(T V_max);
	void set_w_max(T W_max);
	T get_v_max();
	T get_w_max();
	T get_wlr_max();
	void set_r(T r_);
	void set_L(T L_);
	T get_r();
	T get_L();
	void uni2ddr(T Vc, T Wc, T &wr, T &wl);
	T get_wr(T Vc, T Wc);
	T get_wl(T Vc, T Wc);
	void ddr2uni(T wr, T wl, T &Vc, T &Wc);
	T get_Vc(T wr, T wl);
	T get_Wc(T wr, T wl);
	void update_domain_vw(T Vc_in, T Wc_in, T &Vc_out, T &Wc_out);

private:
	Math_functions<T> math_fun;
	T r = 1.0;
	T L = 1.0;
	T w_lr_max = 0.0;
	T V_max = 0.0;
	T W_max = 0.0;
};

#include "Diff_drive_unicycle.tpp"

#endif