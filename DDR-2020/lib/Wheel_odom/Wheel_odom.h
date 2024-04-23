#ifndef WHEEL_ODOM
#define WHEEL_ODOM

#include <math.h>
#include "Math_functions.h"
class Wheel_odom{
    public:
        Wheel_odom();
        void set_param(long N_, double r_, double L_);
        void set_dt(double dt_);
        long get_PPR();
        double get_r();
        double get_L();
        double get_dt();
        void get_w_LR(double* wl, double* wr);
        void get_vw(double* vc, double* wc);
        void get_pose(double* xc, double* yc, double* thc);
        void update(long nr, long nl);

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
