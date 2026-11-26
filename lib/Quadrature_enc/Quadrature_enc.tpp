#include "Quadrature_enc.h"

template <typename T>
Quadrature_enc<T>::Quadrature_enc()
{
}

template <typename T>
void Quadrature_enc<T>::config(uint64_t cprOutput, int pinA, int pinB, void (*isrHandler)(void), bool reverse)
{
    m_cprOutput = cprOutput;
    m_pinA = pinA;
    m_pinB = pinB;
    m_reverse = reverse;
    m_isrHandler = isrHandler;
}

template <typename T>
void Quadrature_enc<T>::init()
{

    pinMode(m_pinA, INPUT);
    pinMode(m_pinB, INPUT);

    a_prev = digitalRead(m_pinA);
    b_prev = digitalRead(m_pinB);

    m_time = micros();
    m_timePrev = m_time;

    attachInterrupt(digitalPinToInterrupt(m_pinA), m_isrHandler, CHANGE);
    attachInterrupt(digitalPinToInterrupt(m_pinB), m_isrHandler, CHANGE);
}

template <typename T>
void Quadrature_enc<T>::isrFunction()
{
    a = digitalRead(m_pinA);
    b = digitalRead(m_pinB);

    if (a != a_prev)
    {
        // interrupt due to A
        if (a != b)
        {
            m_count++;
        }
        else
        {
            m_count--;
        }
    }
    else
    {
        // interrupt due to B
        if (a == b)
        {
            m_count++;
        }
        else
        {
            m_count--;
        }
    }

    a_prev = a;
    b_prev = b;
}

template <typename T>
void Quadrature_enc<T>::update()
{
    noInterrupts();
    m_count_temp = m_count;
    interrupts();

    m_angle = static_cast<T>(m_count_temp) / static_cast<T>(m_cprOutput) * 2.0 * M_PI;
    m_time = micros();

    uint64_t dt = m_time - m_timePrev;
    if (dt > 0)
    {
        m_angularSpeed = (m_angle - m_anglePrev) * 1e6 / (static_cast<T>(dt));
    }

    m_anglePrev = m_angle;
    m_timePrev = m_time;
}

template <typename T>
int64_t Quadrature_enc<T>::getCount()
{
    return m_count_temp * ((m_reverse == true) ? -1 : 1);
}

template <typename T>
T Quadrature_enc<T>::getAngle()
{
    return m_angle * ((m_reverse == true) ? -1 : 1);
}

template <typename T>
T Quadrature_enc<T>::getAngularSpeed()
{
    return m_angularSpeed * ((m_reverse == true) ? -1 : 1);
}
