#ifndef WHEEL_ODOM
#define WHEEL_ODOM

#include <math.h>
#include <Math_functions.h>

#define PI 3.1415926535897932384626433832795f
#define HALF_PI 1.5707963267948966192313216916398f
#define TWO_PI 6.283185307179586476925286766559f
#define DEG_TO_RAD 0.017453292519943295769236907684886f
#define RAD_TO_DEG 57.295779513082320876798154814105f

class Wheel_odom{
    public:
        Wheel_odom();
        void set_ddr_param(long N_, double r_, double L_);
        void set_dt(double dt_);
        long get_PPR();
        double get_r();
        double get_L();
        double get_dt();
        void get_w_LR(double* wl, double* wr);
        void get_vw(double* vc, double* wc);
        void get_xyth(double* xc, double* yc, double* thc);
        void update_odom(long nl, long nr);

    private:
        unsigned long N = 133600;
        double r = 0.0625;
        double L = 0.265;

        double dt = 0.004;
        long N_L, N_R;
        long N_L_pre, N_R_pre;
        double w_L, w_R;
        double v;
        double w;
        double x;
        double y;
        double th;

        Math_functions math;
};

#endif
