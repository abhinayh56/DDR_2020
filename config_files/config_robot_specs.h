#ifndef CONFIG_ROBOT_SPECS
#define CONFIG_ROBOT_SPECS

#define WHEEL_R 0.0425
#define WHEEL_L 0.2

#define V_C_MAX 1.0
#define W_C_MAX 1.0
#define V_BAT_MAX 12.0
#define SAFETY_FACTOR 0.95

#define ENC_PPR 1700
#define MOT_GEAR_RATIO 1
#define MOT_SHAFT_CPR (ENC_PPR*MOT_GEAR_RATIO)
#define PWM_MAX 255.0

#endif