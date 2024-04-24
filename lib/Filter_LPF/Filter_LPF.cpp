#include "Filter_LPF.h"

LPF_filter::LPF_filter(){
	start = true;
}

void LPF_filter::set_param(float fc_, float dt_){
	fc = fc_;
	dt = dt_;

	tau = 1.0 / (const_math_2pi*fc);
	alpha = dt / (dt + tau);
}

void LPF_filter::get_param(float* fc_, float* dt_){
	*fc_ = fc;
	*dt_ = dt;
}

void LPF_filter::set_fc(float fc_){
	fc = fc_;

	tau = 1.0 / (const_math_2pi*fc);
	alpha = dt / (dt + tau);
}

void LPF_filter::set_tau(float tau_){
	tau = tau_;

	fc = 1.0 / (const_math_2pi*tau);
	alpha = dt / (dt + tau);
}

void LPF_filter::set_dt(float dt_){
	dt = dt_;

	alpha = dt / (dt + tau);
}

void LPF_filter::set_alpha(float alpha_){
	alpha = alpha_;

	tau = ((1.0 - alpha)*dt) / alpha;
	fc = 1.0 / (const_math_2pi*tau);
}

float LPF_filter::get_fc(){
	return fc;
}

float LPF_filter::get_tau(){
	return tau;
}

float LPF_filter::get_dt(){
	return dt;
}

float LPF_filter::get_alpha(){
	return alpha;
}

float LPF_filter::get_y(){
	return y;
}

void LPF_filter::cal_y(float x){
	if(start==true){
		start = false;
		y = x;
	}
	else{
		y = alpha*x + (1.0 - alpha)*y_pre;
	}
	y_pre = y;
}

void LPF_filter::reset(){
	y_pre = 0.0;
	start = true;
}