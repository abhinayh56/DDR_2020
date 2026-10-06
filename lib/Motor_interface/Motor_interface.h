#ifndef MOTOR_INTERFACE_H
#define MOTOR_INTERFACE_H

#include <Arduino.h>
#include "../../config/Config.h"

class Motor_interface
{
public:
    Motor_interface();

    void config();

    void command_pwm_1(float pwm_1);

    void command_pwm_2(float pwm_2);

    void command_pwm(float pwm_1, float pwm_2);

    void command_voltage(float v_1, float v_2);
};

#endif // MOTOR_INTERFACE_H