#ifndef DIFF_DRIVE_UNICYCLE
#define DIFF_DRIVE_UNICYCLE

#include "Math_functions.h"
class Diff_drive_unicycle{
    public:
        Diff_drive_unicycle();
        void set_param(float r_, float L_);
        void set_v_max(float V_max);
        void set_w_max(float W_max);
        float get_v_max();
        float get_w_max();
        float get_wlr_max();
        void set_r(float r_);
        void set_L(float L_);
        float get_r();
        float get_L();
        void uni2ddr(float Vc, float Wc, float* wr, float* wl);
        float get_wr(float Vc, float Wc);
        float get_wl(float Vc, float Wc);
        void ddr2uni(float wr, float wl, float* Vc, float* Wc);
        float get_Vc(float wr, float wl);
        float get_Wc(float wr, float wl);
        void update_domain_vw(float Vc_in, float Wc_in, float* Vc_out, float* Wc_out);

    private:
        Math_functions math_fun;
        float r = 1.0;
        float L = 1.0;
        float w_lr_max = 0.0;
        float V_max = 0.0;
        float W_max = 0.0;
};

#endif