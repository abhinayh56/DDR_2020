unsigned long t;
float f = 15;
float dt = 1/f;
int loop_timer = dt*1000000;

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

float th01 = 0;
float error_th1 = 0;
float Kp_th1 = 1; //1,0.95,0.85, 0.75, 0.5, 1.1; //3.5; //3.75

float th02 = 0;
float error_th2 = 0;
float Kp_th2 = 1; //3.5; //3.75

float timer = 0;

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

float Vc = 0;
float Wc = 0;

void setup()
{
  Serial.begin(115200);
  initialize_interrupt();

  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);
  
  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);
  
  t = micros();
}

void loop()
{
  //Kp_th1 = ((float)analogRead(0)/1023.0)*10;
  th1 = (360.0/1700.0)*(float)count1;
  th2 = (360.0/1700.0)*(float)count2;
  //Serial.print(th1); Serial.print('\t');
  //Serial.println(th2);

  rpm1 = (th1 - th_pre1)/dt;
  th_pre1 = th1;
  rpm2 = (th2 - th_pre2)/dt;
  th_pre2 = th2;
  //Serial.print(rpm1); Serial.print('\t');
  //Serial.println(rpm2);

  Vc = 0.000370882466048795*(rpm1+rpm2);
  Wc = 0.2125*(rpm1-rpm2);
  //Serial.print(Vc); Serial.print('\t');
  //Serial.println(Wc);  
  
  //calculate_trajectory();
  //control_angle();
  twist_to_rpm(0.1, (0.1/1.2)*180/PI); //m/sec, deg/sec
  control_rpm();

  //Serial.print(x*100); Serial.print('\t');
  //Serial.print(y*100); Serial.print('\t');
  //Serial.println(phi*180/PI);
  
  while(micros()-t<loop_timer);
  t = micros();
}
