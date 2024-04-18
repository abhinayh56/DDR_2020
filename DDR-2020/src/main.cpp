#include <Arduino.h>

unsigned long t;
float f = 15;
float dt = 1.0/f;
int loop_timer = (float)dt*1000000;
float timer = 0;

void initialize_interrupt();
void run_estimators();
void estimate_motors_rpm();
void estimate_vehicle_position();
void estimate_vehicle_twist();
void command_motor1(float pwm1);
void command_motor2(float pwm2);
float rad2deg(float _rad);

#define A1 18
#define B1 19
#define A2 2
#define B2 3

volatile long count1 = 0;
volatile long count2 = 0;

float th1 = 0;
float th_pre1 = 0;
float rpm1 = 0;

float th2 = 0;
float th_pre2 = 0;
float rpm2 = 0;

volatile float Dr = 0;
volatile float Dl = 0;
volatile float Dc = 0;
volatile float phi = 0;
volatile float x = 0;
volatile float y = 0;
const float var = 0.000157079632679489661923132169164; //(2.0*PI/1700.0)*(85.0/2.0)*0.001;
const float L = 0.2;
const float r = 0.085*0.5;
const float dphi = 0.00078539816339744830961566084582;
volatile float dphi_temp = 0;
const float pi = 3.1415926535897932384626433832795;
const float pi_2 = 6.283185307179586476925286766559;

float X;
float Y;
float PHI;

// linear velocity and angular velocity
float Vc = 0;
float Wc = 0;

// deseired twist
float Vc0 = 0;
float Wc0 = 0;

void setup() {
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);

  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);

  initialize_interrupt();
}

void loop() {
  Serial.begin(9600);
  run_estimators();

  command_motor1(0);
  command_motor2(0);
  delay(50);
}

void initialize_interrupt()
{
  //interrupt [2,3,4,5]->[19,18,2,3]
  EICRA = (1 << ISC20) | (1 << ISC30);
  EICRB = (1 << ISC40) | (1 << ISC50);
  EIMSK = (1 << INT2) | (1 << INT3) | (1 << INT4) | (1 << INT5);
  sei();
}

ISR(INT2_vect)
{
  int a1 = digitalRead(A1);
  int b1 = digitalRead(B1);
  
  if(a1==1)
  {
    if(b1==1)
    {
      count1++;
      Dr = var;
      dphi_temp = dphi;
    }
    else
    {
      count1--;
      Dr = -var;
      dphi_temp = -dphi;
    }
  }
  else
  {
    if(b1==0)
    {
      count1++;
      Dr = var;
      dphi_temp = dphi;
    }
    else
    {
      count1--;
      Dr = -var;
      dphi_temp = -dphi;
    }
  }

  //phi += Dr/L;
  phi += dphi_temp;
  if(phi>pi)
  {
    phi = phi-pi_2;
  }
  else if (phi<=-pi)
  {
    phi = phi+pi_2;
  }
  Dc = Dr*0.5;
  x += Dc*cos(phi);
  y += Dc*sin(phi);
}

ISR(INT3_vect)
{
  int a1 = digitalRead(A1);
  int b1 = digitalRead(B1);
  
  if(b1==1)
  {
    if(a1==0)
    {
      count1++;
      Dr = var;
      dphi_temp = dphi;
    }
    else
    {
      count1--;
      Dr = -var;
      dphi_temp = -dphi;
    }
  }
  else
  {
    if(a1==1)
    {
      count1++;
      Dr = var;
      dphi_temp = dphi;
    }
    else
    {
      count1--;
      Dr = -var;
      dphi_temp = -dphi;
    }
  }

  //phi += Dr/L;
  phi += dphi_temp;
  if(phi>pi)
  {
    phi = phi-pi_2;
  }
  else if (phi<=-pi)
  {
    phi = phi+pi_2;
  }
  Dc = Dr*0.5;
  x += Dc*cos(phi);
  y += Dc*sin(phi);
}

ISR(INT4_vect)
{
  int a2 = digitalRead(A2);
  int b2 = digitalRead(B2);
  
  if(a2==1)
  {
    if(b2==1)
    {
      count2++;
      Dl = var;
      dphi_temp = -dphi;
    }
    else
    {
      count2--;
      Dl = -var;
      dphi_temp = dphi;
    }
  }
  else
  {
    if(b2==0)
    {
      count2++;
      Dl = var;
      dphi_temp = -dphi;
    }
    else
    {
      count2--;
      Dl = -var;
      dphi_temp = dphi;
    }
  }

  //phi -= Dl/L;
  phi += dphi_temp;
  if(phi>pi)
  {
    phi = phi-pi_2;
  }
  else if (phi<=-pi)
  {
    phi = phi+pi_2;
  }
  Dc = Dl*0.5;
  x += Dc*cos(phi);
  y += Dc*sin(phi);
}

ISR(INT5_vect)
{
  int a2 = digitalRead(A2);
  int b2 = digitalRead(B2);
  
  if(b2==1)
  {
    if(a2==0)
    {
      count2++;
      Dl = var;
      dphi_temp = -dphi;
    }
    else
    {
      count2--;
      Dl = -var;
      dphi_temp = dphi;
    }
  }
  else
  {
    if(a2==1)
    {
      count2++;
      Dl = var;
      dphi_temp = -dphi;
    }
    else
    {
      count2--;
      Dl = -var;
      dphi_temp = dphi;
    }
  }

  //phi -= Dl/L;
  phi += dphi_temp;
  if(phi>pi)
  {
    phi = phi-pi_2;
  }
  else if (phi<=-pi)
  {
    phi = phi+pi_2;
  }
  Dc = Dl*0.5;
  x += Dc*cos(phi);
  y += Dc*sin(phi);
}

void run_estimators()
{
  estimate_motors_rpm();
  estimate_vehicle_position();
  estimate_vehicle_twist();
}

void estimate_motors_rpm()
{
  // angle of rotation of motor
  th1 = (360.0/1700.0)*(float)count1;
  th2 = (360.0/1700.0)*(float)count2;
  // Serial.print(th1); Serial.print('\t');
  // Serial.println(th2);

  // rpm of motor
  rpm1 = (th1 - th_pre1)/dt;
  th_pre1 = th1;
  rpm2 = (th2 - th_pre2)/dt;
  th_pre2 = th2;
  Serial.print(rpm1); Serial.print('\t');
  Serial.println(rpm2);
}

void estimate_vehicle_position()
{
  X = x*100.0;
  Y = y*100.0;
  PHI = rad2deg(phi);

  //Serial.print(X); Serial.print('\t');
  //Serial.print(Y); Serial.print('\t');
  //Serial.println(PHI);
}

void estimate_vehicle_twist()
{
  Vc = 0.000370882466048795*(rpm1+rpm2);
  Wc = 0.2125*(rpm1-rpm2);
  //Serial.print(Vc); Serial.print('\t');
  //Serial.println(Wc);
}

float rad2deg(float _rad)
{
  //return _rad*180/PI;
  return _rad*57.295779513082320876798154814105;
}

void command_motor1(float pwm1)
{
  if(pwm1<0)
  {
    digitalWrite(4,1);
    digitalWrite(5,0);
    analogWrite(6,(int)(-1*pwm1));
  }
  else
  {
    digitalWrite(4,0);
    digitalWrite(5,1);
    analogWrite(6,(int)pwm1);
  }
}

void command_motor2(float pwm2)
{
  if(pwm2<0)
  {
    digitalWrite(8,1);
    digitalWrite(9,0);
    analogWrite(7,(int)(-1*pwm2));
  }
  else
  {
    digitalWrite(8,0);
    digitalWrite(9,1);
    analogWrite(7,(int)pwm2);
  }
}
