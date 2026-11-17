#include "L298N.h"

L298N::L298N()
{
}

L298N::~L298N()
{
}

void L298N::config(double voltInDriver, uint16_t pwmMax)
{
  m_voltInDriver = voltInDriver;
  m_pwmMax = pwmMax;
}

void L298N::configMotorA(uint8_t pinPWM_A, uint8_t pinINA_1, uint8_t pinINA_2, bool reverse)
{
  m_pinPWM_A = pinPWM_A;
  m_reverseMotorA = reverse;
  m_pinINA_1 = pinINA_1;
  m_pinINA_2 = pinINA_2;
}

void L298N::configMotorB(uint8_t pinPWM_B, uint8_t pinINB_1, uint8_t pinINB_2, bool reverse)
{
  m_pinPWM_B = pinPWM_B;
  m_reverseMotorB = reverse;
  m_pinINB_1 = pinINB_1;
  m_pinINB_2 = pinINB_2;
}

void L298N::init()
{
}

void L298N::initMotorA()
{
  pinMode(m_pinPWM_A, OUTPUT);
  pinMode(m_pinINA_1, OUTPUT);
  pinMode(m_pinINA_2, OUTPUT);
}

void L298N::initMotorB()
{
  pinMode(m_pinPWM_B, OUTPUT);
  pinMode(m_pinINB_1, OUTPUT);
  pinMode(m_pinINB_2, OUTPUT);
}

void L298N::disableMotorA()
{
  digitalWrite(m_pinINA_1, LOW);
  digitalWrite(m_pinINA_2, LOW);
  analogWrite(m_pinPWM_A, 0);
}

void L298N::disableMotorB()
{
  digitalWrite(m_pinINB_1, LOW);
  digitalWrite(m_pinINB_2, LOW);
  analogWrite(m_pinPWM_B, 0);
}

void L298N::enableMotorA()
{
  digitalWrite(m_pinINA_1, HIGH);
  digitalWrite(m_pinINA_2, LOW);
  analogWrite(m_pinPWM_A, 0);
}

void L298N::enableMotorB()
{
  digitalWrite(m_pinINB_1, HIGH);
  digitalWrite(m_pinINB_2, LOW);
  analogWrite(m_pinPWM_B, 0);
}

void L298N::commandMotorA(double voltInMotorA)
{
  voltInMotorA = voltInMotorA * ((m_reverseMotorA == true) ? -1 : 1);
  voltInMotorA = constrain(voltInMotorA, -m_voltInDriver, m_voltInDriver);

  if(voltInMotorA < 0)
  {
    voltInMotorA = -voltInMotorA;
    digitalWrite(m_pinINA_1, HIGH);
    digitalWrite(m_pinINA_2, LOW);
  }
  else
  {
    digitalWrite(m_pinINA_1, LOW);
    digitalWrite(m_pinINA_2, HIGH);
  }

  analogWrite(m_pinPWM_A, constrain(static_cast<int>(voltInMotorA * static_cast<double>(m_pwmMax) / m_voltInDriver), static_cast<int>(0), static_cast<int>(m_pwmMax)));
}

void L298N::commandMotorB(double voltInMotorB)
{
  voltInMotorB = voltInMotorB * ((m_reverseMotorB == true) ? -1 : 1);
  voltInMotorB = constrain(voltInMotorB, -m_voltInDriver, m_voltInDriver);

  if(voltInMotorB < 0)
  {
    voltInMotorB = -voltInMotorB;
    digitalWrite(m_pinINB_1, HIGH);
    digitalWrite(m_pinINB_2, LOW);
  }
  else
  {
    digitalWrite(m_pinINB_1, LOW);
    digitalWrite(m_pinINB_2, HIGH);
  }
  
  analogWrite(m_pinPWM_B, constrain(static_cast<int>(voltInMotorB * static_cast<double>(m_pwmMax) / m_voltInDriver), static_cast<int>(0), static_cast<int>(m_pwmMax)));
}

void L298N::eStopDrive()
{
  digitalWrite(m_pinINA_1, LOW);
  digitalWrite(m_pinINA_2, LOW);
  analogWrite(m_pinPWM_A, 0);
  digitalWrite(m_pinINB_1, LOW);
  digitalWrite(m_pinINB_2, LOW);
  analogWrite(m_pinPWM_B, 0);
}

void L298N::enableDrive()
{
  digitalWrite(m_pinINA_1, HIGH);
  digitalWrite(m_pinINA_2, LOW);
  analogWrite(m_pinPWM_A, 0);
  digitalWrite(m_pinINB_1, HIGH);
  digitalWrite(m_pinINB_2, LOW);
  analogWrite(m_pinPWM_B, 0);
}
