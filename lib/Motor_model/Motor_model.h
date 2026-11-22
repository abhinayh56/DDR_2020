#ifndef MOTOR_MODEL_H
#define MOTOR_MODEL_H

#include <stdint.h>
#include <Arduino.h>

template <typename T>
class Motor_model
{
public:
    Motor_model();

    ~Motor_model();

    void assign_data(T (*data_wv_)[2], uint8_t num_data_);

    T getV(T w);

private:
    T (*data_wv)[2];
    uint8_t num_data = 0;
};

#include "Motor_model.tpp"

#endif // MOTOR_MODEL_H