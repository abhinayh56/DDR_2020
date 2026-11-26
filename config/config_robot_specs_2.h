#ifndef CONFIG_ROBOT_SPECS_2_H
#define CONFIG_ROBOT_SPECS_2_H

#include <stdint.h>

#define WHEEL_R        0.0425
#define WHEEL_L        0.199 // 0.2

#define V_C_MAX        0.296 // @ 400 deg / sec wheel speed
#define W_C_MAX        2.960 // @ 400 deg / sec wheel speed
#define V_BAT_MAX      12.0
#define SAFETY_FACTOR  0.99

#define ENC_PPR        3
#define MOT_GEAR_RATIO 46.666667
#define MOT_SHAFT_CPR  static_cast<uint64_t>(4.0 * static_cast<float>(ENC_PPR) * static_cast<float>(MOT_GEAR_RATIO)) // (PPR*2)*2*N // 560
#define PWM_MAX        255.0

#endif // CONFIG_ROBOT_SPECS_2_H