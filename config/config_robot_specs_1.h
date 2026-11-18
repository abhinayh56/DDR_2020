#ifndef CONFIG_ROBOT_SPECS_1_H
#define CONFIG_ROBOT_SPECS_1_H

#define WHEEL_R        0.0425
#define WHEEL_L        0.198 // 0.2

#define V_C_MAX        0.185 // @ 250 deg / sec wheel speed
#define W_C_MAX        1.850 // @ 250 deg / sec wheel speed
#define V_BAT_MAX      12.0
#define SAFETY_FACTOR  0.99

#define ENC_PPR        3
#define MOT_GEAR_RATIO 141
#define MOT_SHAFT_CPR  (4 * ENC_PPR * MOT_GEAR_RATIO) // (PPR*2)*2*N // 1692
#define PWM_MAX        255.0

#endif // CONFIG_ROBOT_SPECS_1_H