#include "Motor_interface.h"

Motor_interface::Motor_interface()
{
}

void Motor_interface::config()
{
    pinMode(MOTOR_1_PIN_A, OUTPUT);
    pinMode(MOTOR_1_PIN_B, OUTPUT);
    pinMode(MOTOR_1_PIN_PWM, OUTPUT);

    pinMode(MOTOR_2_PIN_A, OUTPUT);
    pinMode(MOTOR_2_PIN_B, OUTPUT);
    pinMode(MOTOR_2_PIN_PWM, OUTPUT);
}

void Motor_interface::command_pwm_1(float pwm_1)
{
    if (pwm_1 < 0)
    {
        digitalWrite(MOTOR_1_PIN_A, 0);
        digitalWrite(MOTOR_1_PIN_B, 1);
        analogWrite(MOTOR_1_PIN_PWM, (int)(-pwm_1));
    }
    else
    {
        digitalWrite(MOTOR_1_PIN_A, 1);
        digitalWrite(MOTOR_1_PIN_B, 0);
        analogWrite(MOTOR_1_PIN_PWM, (int)pwm_1);
    }
}

void Motor_interface::command_pwm_2(float pwm_2)
{
    if (pwm_2 < 0)
    {
        digitalWrite(MOTOR_2_PIN_A, 0);
        digitalWrite(MOTOR_2_PIN_B, 1);
        analogWrite(MOTOR_2_PIN_PWM, (int)(-pwm_2));
    }
    else
    {
        digitalWrite(MOTOR_2_PIN_A, 1);
        digitalWrite(MOTOR_2_PIN_B, 0);
        analogWrite(MOTOR_2_PIN_PWM, (int)pwm_2);
    }
}

void Motor_interface::command_pwm(float pwm_1, float pwm_2)
{
    command_pwm_1(pwm_1);
    command_pwm_2(pwm_2);
}

void Motor_interface::command_voltage(float v_1, float v_2)
{
    float pwm_1 = (PWM_MAX / V_BAT_MAX) * v_1;
    float pwm_2 = (PWM_MAX / V_BAT_MAX) * v_2;

    command_pwm(pwm_1, pwm_2);
}
