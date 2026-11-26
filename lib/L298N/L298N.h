#ifndef L298N_H
#define L298N_H

#include <Arduino.h>
#include <stdint.h>

template <typename T>
class L298N
{
public:
  L298N();

  ~L298N();

  void config(T voltInDriver, uint16_t pwmMax);

  void configMotorA(uint8_t pinPWM_A, uint8_t pinINA_1, uint8_t pinINA_2, bool reverse);

  void configMotorB(uint8_t pinPWM_B, uint8_t pinINB_1, uint8_t pinINB_2, bool reverse);

  void init();

  void initMotorA();

  void initMotorB();

  void disableMotorA();

  void disableMotorB();

  void enableMotorA();

  void enableMotorB();

  void commandMotorA(T voltInMotorA);

  void commandMotorB(T voltInMotorB);

  void eStopDrive();

  void enableDrive();

private:
  uint8_t m_pinPWM_A;
  uint8_t m_pinINA_1;
  uint8_t m_pinINA_2;

  uint8_t m_pinPWM_B;
  uint8_t m_pinINB_1;
  uint8_t m_pinINB_2;

  bool m_reverseMotorA;
  bool m_reverseMotorB;

  T m_voltInDriver;

  uint16_t m_pwmMax;
};

#include "L298N.tpp"

#endif  // L298N_H