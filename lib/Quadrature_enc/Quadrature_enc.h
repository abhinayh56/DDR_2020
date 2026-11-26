#ifndef QUADRATURE_ENC
#define QUADRATURE_ENC

#include <stdint.h>
#include <Arduino.h>

template <typename T>
class Quadrature_enc
{
public:
    Quadrature_enc();

    void config(uint64_t cprOutput, int pinA, int pinB, void (*isrHandler)(void), bool reverse);

    void init();

    void isrFunction();

    void update();

    int64_t getCount();

    T getAngle();

    T getAngularSpeed();

private:
    uint64_t m_cprOutput;
    uint8_t m_pinA;
    uint8_t m_pinB;
    bool m_reverse;
    void (*m_isrHandler)(void);

    volatile int64_t m_count = 0;
    volatile int a = false;
    volatile int a_prev = false;
    volatile int b = false;
    volatile int b_prev = false;

    int64_t m_count_temp = 0;

    T m_angle = 0;
    T m_anglePrev = 0;
    T m_angularSpeed = 0;
    uint64_t m_time = 0;
    uint64_t m_timePrev = 0;
};

#include "Quadrature_enc.tpp"

#endif // QUADRATURE_ENC
