#ifndef CONFIG_ROBOT_PARAM
#define CONFIG_ROBOT_PARAM

#define MAIN_LOOP_FREQ 100.0

#define MOTOR_1_PIN_A   8
#define MOTOR_1_PIN_B   9
#define MOTOR_1_PIN_PWM 7

#define MOTOR_2_PIN_A   4
#define MOTOR_2_PIN_B   5
#define MOTOR_2_PIN_PWM 6

#define ENC_1_PIN_A 18
#define ENC_1_PIN_B 19
#define ENC_2_PIN_A 2
#define ENC_2_PIN_B 3

#define ENC_1_CPR 1700
#define ENC_2_CPR 1700
#define WHEEL_R 0.0425
#define WHEEL_L 0.2

#define V_C_MAX 1.0
#define W_C_MAX 1.0
#define V_BAT_MAX 12.0
#define PWM_MAX 255.0

#define Kp_R    0.0
#define Ki_R    0.0
#define Kd_R    0.0
#define Kff_R   0.0
#define dt_R    1.0/MAIN_LOOP_FREQ
#define I_max_R V_BAT_MAX*0.95
#define u_max_R V_BAT_MAX*0.95
#define fc_R    MAIN_LOOP_FREQ*0.5

#define Kp_L    0.0
#define Ki_L    0.0
#define Kd_L    0.0
#define Kff_L   0.0
#define dt_L    1.0/MAIN_LOOP_FREQ
#define I_max_L V_BAT_MAX*0.95
#define u_max_L V_BAT_MAX*0.95
#define fc_L    MAIN_LOOP_FREQ*0.5

#endif