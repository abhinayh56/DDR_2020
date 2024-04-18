#include "Wheel_odom.h"

Wheel_odom::Wheel_odom(){
}

void Wheel_odom::set_ddr_param(long N_, double r_, double L_){
    N = N_;
    r = r_;
    L = L_;
}

void Wheel_odom::set_dt(double dt_){
    dt = dt_;
}

long Wheel_odom::get_PPR(){
    return N;
}

double Wheel_odom::get_r(){
    return r;
}

double Wheel_odom::get_L(){
    return L;
}

double Wheel_odom::get_dt(){
    return dt;
}

void Wheel_odom::get_w_LR(double* wl, double* wr){
    *wl = w_L;
    *wr = w_R;
}

void Wheel_odom::get_vw(double* vc, double* wc){
    *vc = v;
    *wc = w;
}

void Wheel_odom::get_xyth(double* xc, double* yc, double* thc){
    *xc = x;
    *yc = y;
    *thc = th;
}

void Wheel_odom::update_odom(long nl, long nr){
    N_L = nl;
    N_R = nr;
    w_L = TWO_PI*(double)(N_L - N_L_pre)/((double)N*dt);
    w_R = TWO_PI*(double)(N_R - N_R_pre)/((double)N*dt);
    v = (r/2.0)*(w_L+w_R);
    w = (r/L)*(w_R-w_L);

    double dl = (PI*r*(double)((N_R-N_R_pre) + (N_L-N_L_pre)))/((double)N);
    double dth = (TWO_PI*r*(double)((N_R-N_R_pre) - (N_L-N_L_pre)))/((double)N*L);

    th += dth;
    th = math.wrap(th,-PI,PI);
    x += dl*cos(th);
    y += dl*sin(th);

    N_L_pre = N_L;
    N_R_pre = N_R;
}