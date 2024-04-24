#include "Diff_drive_unicycle.h"

Diff_drive_unicycle::Diff_drive_unicycle(){
}

void Diff_drive_unicycle::set_param(double r_, double L_){
    r = r_;
    L = L_;
}

void Diff_drive_unicycle::set_v_max(double V_max_){
    V_max = V_max_;
    w_lr_max = V_max/r;
    W_max = 2.0*V_max/L;
}

void Diff_drive_unicycle::set_w_max(double W_max_){
    W_max = W_max_;
    V_max = W_max*L/2.0;
    w_lr_max = V_max/r;
}

double Diff_drive_unicycle::get_v_max(){
    return V_max;
}

double Diff_drive_unicycle::get_w_max(){
    return W_max;
}

double Diff_drive_unicycle::get_wlr_max(){
    return w_lr_max;
}

void Diff_drive_unicycle::set_r(double r_){
    r = r_;
}

void Diff_drive_unicycle::set_L(double L_){
    L = L_;
}

double Diff_drive_unicycle::get_r(){
    return r;
}

double Diff_drive_unicycle::get_L(){
    return L;
}

void Diff_drive_unicycle::uni2ddr(double Vc_, double Wc_, double* wr, double* wl){
    double Vc = Vc_;
    double Wc = Wc_;
    update_domain_vw(Vc_, Wc_, &Vc, &Wc);
    
    *wr = (Vc + Wc*L*0.5) / r;
    *wl = (Vc - Wc*L*0.5) / r;
}

double Diff_drive_unicycle::get_wr(double Vc, double Wc){
    return (Vc + Wc*L*0.5) / r;
}

double Diff_drive_unicycle::get_wl(double Vc, double Wc){
    return (Vc - Wc*L*0.5) / r;
}

void Diff_drive_unicycle::ddr2uni(double wr, double wl, double* Vc, double* Wc){
    *Vc = (wr + wl)*r*0.5;
    *Wc = (wr - wl)*r/L;
}

double Diff_drive_unicycle::get_Vc(double wr, double wl){
    return (wr + wl)*r*0.5;
}

double Diff_drive_unicycle::get_Wc(double wr, double wl){
    return (wr - wl)*r/L;
}

void Diff_drive_unicycle::update_domain_vw(double V_in, double W_in, double* V_out, double* W_out){
    double V_n = V_in;
    double W_n = W_in;

    int region = 0;

    if(W_in>=W_max){
        region = 5;
    }
    else if(W_in <=(-W_max)){
        region = 6;
    }
    else if(W_in>=0){
        if(V_in>=0){
            if(math_fun.points_A0_line_same_side(V_in, W_in, V_max, 0.0, 0.0, W_max)==false){
                region = 1;
            }
        }
        else{
            if(math_fun.points_A0_line_same_side(V_in, W_in, -V_max, 0.0, 0.0, W_max)==false){
                region = 2;
            }
        }
    }
    else{
        if(V_in>=0){
            if(math_fun.points_A0_line_same_side(V_in, W_in, V_max, 0.0, 0.0, -W_max)==false){
                region = 4;
            }
        }
        else{
            if(math_fun.points_A0_line_same_side(V_in, W_in, -V_max, 0.0, 0.0, -W_max)==false){
                region = 3;
            }
        }
    }

    switch(region){
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

    *V_out = V_n;
    *W_out = W_n;
}