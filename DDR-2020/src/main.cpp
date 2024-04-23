#include <Arduino.h>

unsigned long t;
float f = 15;
float dt = 1.0/f;
int loop_timer = (float)dt*1000000;
float timer = 0;

void initialize_interrupt();
void command_motor1(float pwm1);
void command_motor2(float pwm2);

#define A1 18
#define B1 19
#define A2 2
#define B2 3

volatile long count1 = 0;
volatile long count2 = 0;

void setup() {
  Serial.begin(9600);

  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  pinMode(6,OUTPUT);

  pinMode(7,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);

  initialize_interrupt();
}

void loop() {
  delay(10);
}


void initialize_interrupt()
{
  //interrupt [2,3,4,5]->[19,18,2,3]
  EICRA = (1 << ISC20) | (1 << ISC30);
  EICRB = (1 << ISC40) | (1 << ISC50);
  EIMSK = (1 << INT2) | (1 << INT3) | (1 << INT4) | (1 << INT5);
  sei();
}

ISR(INT2_vect){
  int a1 = digitalRead(A1);
  int b1 = digitalRead(B1);
  
  if(a1==1){
    if(b1==1){
      count1++;
    }
    else{
      count1--;
    }
  }
  else{
    if(b1==0){
      count1++;
    }
    else{
      count1--;
    }
  }
}

ISR(INT3_vect)
{
  int a1 = digitalRead(A1);
  int b1 = digitalRead(B1);
  
  if(b1==1){
    if(a1==0){
      count1++;
    }
    else{
      count1--;
    }
  }
  else{
    if(a1==1){
      count1++;
    }
    else{
      count1--;
    }
  }
}

ISR(INT4_vect){
  int a2 = digitalRead(A2);
  int b2 = digitalRead(B2);
  
  if(a2==1){
    if(b2==1){
      count2++;
    }
    else{
      count2--;
    }
  }
  else{
    if(b2==0){
      count2++;
    }
    else{
      count2--;
    }
  }
}

ISR(INT5_vect){
  int a2 = digitalRead(A2);
  int b2 = digitalRead(B2);
  
  if(b2==1){
    if(a2==0){
      count2++;
    }
    else{
      count2--;
    }
  }
  else{
    if(a2==1){
      count2++;
    }
    else{
      count2--;
    }
  }
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
