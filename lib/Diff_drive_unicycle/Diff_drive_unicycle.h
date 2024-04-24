#ifndef DIFF_DRIVE_UNICYCLE
#define DIFF_DRIVE_UNICYCLE

#include "Math_functions.h"
class Diff_drive_unicycle{
    public:
        Diff_drive_unicycle();
        void set_param(double r_, double L_);
        void set_v_max(double V_max);
        void set_w_max(double W_max);
        double get_v_max();
        double get_w_max();
        double get_wlr_max();
        void set_r(double r_);
        void set_L(double L_);
        double get_r();
        double get_L();
        void uni2ddr(double Vc, double Wc, double* wr, double* wl);
        double get_wr(double Vc, double Wc);
        double get_wl(double Vc, double Wc);
        void ddr2uni(double wr, double wl, double* Vc, double* Wc);
        double get_Vc(double wr, double wl);
        double get_Wc(double wr, double wl);
        void update_domain_vw(double Vc_in, double Wc_in, double* Vc_out, double* Wc_out);

    private:
        Math_functions math_fun;
        double r = 1.0;
        double L = 1.0;
        double w_lr_max = 0.0;
        double V_max = 0.0;
        double W_max = 0.0;
};

#endif