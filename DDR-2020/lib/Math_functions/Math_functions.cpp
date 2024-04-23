#include "Math_functions.h"

float Math_functions::saturate(float x, float x_min, float x_max){
  if(x<x_min){
    return x_min;
  }
  else if(x>x_max){
    return x_max;
  }
  else{
    return x;
  }
}

float Math_functions::mod(float x){
  if(x<0){
    return -x;
  }
  else{
    return x;
  }
}

float Math_functions::sign(float x){
  if(x<0){
    return -1.0;
  }
  else if(x==0){
    return 0.0;
  }
  else{
    return 1.0;
  }
}

int Math_functions::great_int(float x){
  if(x>=0){
    return (int)x;
  }
  else{
    return (int)x - 1;
  }
}

float Math_functions::ceil(float x){
  if(x>=0){
    return float((int)x + 1);
  }
  else{
    return float((int)x);
  }
}

float Math_functions::floor(float x){
  if(x>=0){
    return float((int)x);
  }
  else{
    return float((int)x - 1);
  }
}

float Math_functions::wrap(float x, float a, float b){
  if(x<=a){
    return x - a + b;
  }
  else if(x>b){
    return x + a - b;
  }
  else{
    return x;
  }
}

float Math_functions::apply_deadband(float x, float deadband){
  if(mod(x)<deadband*0.5){
    return 0;
  }
  else{
    return x;
  }
}

float Math_functions::apply_deadband2(float x, float deadband){
  if(mod(x)<=deadband*0.5){
    return 0;
  }
  else{
    if(x>deadband*0.5){
      return x - deadband*0.5;
    }
    else{
      return x + deadband*0.5;
    }
  }
}

float Math_functions::rad_2_deg(float x){
  return x*57.295779513082320876798154814105; //x*180/PI
}

float Math_functions::deg_2_rad(float x){
  return x*0.01745329251994329576923690768489; //x*PI/180
}

float Math_functions::rps_2_rpm(float rps){
    return rps*9.54929658551;
}

float Math_functions::rpm_2_rps(float rpm){
  // return rpm/9.54929658551;
    return rpm*0.10471975512; // for faster execution
}

float Math_functions::linear_map(float x, float x1, float x2, float y1, float y2){
  return y1 + ((y2-y1)/(x2-x1))*(x-x1);
}

float Math_functions::poly_map_deg1(float x, float a0, float a1){
  return a0 + a1*x;
}

float Math_functions::poly_map_deg2(float x, float a0, float a1, float a2){
  return a0 + a1*x + a2*x*x;
}

float Math_functions::poly_map_deg3(float x, float a0, float a1, float a2, float a3){
  return a0 + a1*x + a2*x*x + a3*x*x*x;
}

float Math_functions::poly_map_deg4(float x, float a0, float a1, float a2, float a3, float a4){
  return a0 + a1*x + a2*x*x + a3*x*x*x + a4*x*x*x*x;
}

float Math_functions::poly_map_deg5(float x, float a0, float a1, float a2, float a3, float a4, float a5){
  return a0 + a1*x + a2*x*x + a3*x*x*x + a4*x*x*x*x + a4*x*x*x*x*x;
}

float Math_functions::poly_map_deg6(float x, float a0, float a1, float a2, float a3, float a4, float a5, float a6){
  return a0 + a1*x + a2*x*x + a3*x*x*x + a4*x*x*x*x + a4*x*x*x*x*x + a4*x*x*x*x*x*x;
}

float Math_functions::poly_map_degn(float x, float a[], int n){
  if(n>=1){
    return 0;//poly_map_degn(float x, float a[], int n-1);
  }
  else{
    return a[0];
  }
}

// https://en.wikipedia.org/wiki/Fast_inverse_square_root
float Math_functions::fast_inv_sqrt(float number){
  long i;
  float x2, y;
  const float threehalfs = 1.5F;

  x2 = number*0.5F;
  y  = number;
  i  = *(long*)&y;
  i  = 0x5f3759df - (i>>1);
  y  = *(float*)&i;
  y  = y*(threehalfs - (x2*y*y));
	// y  = y*(threehalfs - (x2*y*y));   // 2nd iteration, this can be removed
	return y;
}

bool Math_functions::points_AB_line_same_side(float x, float y, float x0, float y0, float x1, float y1, float x2, float y2){
  float L0 = (y2-y1)*(x0-x1) - (x2-x1)*(y0-y1);
  float L  = (y2-y1)*(x -x1) - (x2-x1)*(y -y1);
  if(L0*L>=0){
    return true;
  }
  else{
    return false;
  }
}

bool Math_functions::points_A0_line_same_side(float x, float y, float x1, float y1, float x2, float y2){
  float L0 = (y2-y1)*(-x1) - (x2-x1)*(-y1);
  float L  = (y2-y1)*(x-x1) - (x2-x1)*(y-y1);
  if(L0*L>=0){
    return true;
  }
  else{
    return false;
  }
}

Circle::Circle(){
}

Circle::~Circle(){
}

bool Circle::inside(float cx, float cy, float r, float x0, float y0){
  if(sqrt((x0-cx)*(x0-cx) + (y0-cy)*(y0-cy))<=r){
    return true;
  }
  else{
    return false;
  }
}

void Circle::intersection_line(float cx, float cy, float r, float x1, float y1, float x2, float y2, float* x10, float* y10, float* x20, float* y20){
  float a = (x2-x1)*(x2-x1) + (y2-y1)*(y2-y1);
  float b = 2.0*((x2-x1)*(x1-cx) + (y2-y1)*(y1-cy));
  float c = (x1-cx)*(x1-cx) + (y1-cy)*(y1-cy) - r*r;

  float D = b*b - 4.0*a*c;

  if(D>0.0){
  float lambda_1 = (-b + sqrt(D)) / (2.0*a);
  float lambda_2 = (-b - sqrt(D)) / (2.0*a);
    *x10 = x1 + lambda_1*(x2-x1);
    *y10 = y1 + lambda_1*(y2-y1);
    *x20 = x1 + lambda_2*(x2-x1);
    *y20 = y1 + lambda_2*(y2-y1);
  }
  else if(D==0.0){
    float lambda_1 = (-b) / (2.0*a);
    float lambda_2 = (-b) / (2.0*a);
    *x10 = x1 + lambda_1*(x2-x1);
    *y10 = y1 + lambda_1*(y2-y1);
    *x20 = x1 + lambda_2*(x2-x1);
    *y20 = y1 + lambda_2*(y2-y1);
  }
  else{
    *x10 = math_inf;
    *y10 = math_inf;
    *x20 = -math_inf;
    *y20 = -math_inf;
  }
}

void Circle::intersection_circle(float c1x, float c1y, float r1, float c2x, float c2y, float r2, float* x01, float* y01, float* x02, float* y02){
}