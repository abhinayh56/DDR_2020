/* This is the program for Differential Drive Robot (DDR2020).
 *  It runs on Arduino Mega
 *  Abhinay Kumar
 *  IITJ Helicopter Lab
 *  18 Aug 2020
*/

// time variables
unsigned long t;
float f = 15;
float dt = 1.0/f;
int loop_timer = (float)dt*1000000;
float timer = 0;

// encoder pins A1, B1 right; A2, B2 left
#define A1 18
#define B1 19
#define A2 2
#define B2 3

// encoder counts
volatile long count1 = 0;
volatile long count2 = 0;

// motor angle of rotation
float th1 = 0;
float th_pre1 = 0;
float rpm1 = 0;

float th2 = 0;
float th_pre2 = 0;
float rpm2 = 0;

// motor rpm control
float rpm01 = 0;
float error_rpm1 = 0;
float pwm_I1 = 0;

float rpm02 = 0;
float error_rpm2 = 0;
float pwm_I2 = 0;

float Kp1 = 1.4; //[1.15,1.4,1.75]
float Ki1 = 0.5; //[0.35,0.5,0.6]
float pwm1;

float Kp2 = 1.4; //[1.15,1.4,1.75]
float Ki2 = 0.5; //[0.35,0.5,0.6]
float pwm2;

// motor angle control
float th01 = 0;
float error_th1 = 0;
float Kp_th1 = 1; //1,0.95,0.85, 0.75, 0.5, 1.1; //3.5; //3.75

float th02 = 0;
float error_th2 = 0;
float Kp_th2 = 1; //3.5; //3.75

// globlal position and orientation
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

// serial data communication
char rx_char;
String rx_str = "";

char Vc0_char[6];
char Wc0_char[6];

bool Vc0_comming = 0;
bool Wc0_comming = 0;

void setup()
{
  // start serial communication
  Serial.begin(115200);
  
  // initialie external interrupts for encoder input
  initialize_interrupt();

  // right motor control pin
  pinMode(4,OUTPUT); // direction pin
  pinMode(5,OUTPUT); // direction pin
  pinMode(6,OUTPUT);  // analog pin

  // left motor control pin
  pinMode(7,OUTPUT); // analog pin
  pinMode(8,OUTPUT); // direction pin
  pinMode(9,OUTPUT); // direction pin
  
  t = micros(); // stores time variable in microseconds
}

void loop()
{
  run_estimators();

  communicate_data();
  
  //run_planners();
  
  run_controllers();
    
  while(micros()-t<loop_timer);
  t = micros();
}
