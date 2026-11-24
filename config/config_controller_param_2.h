#ifndef CONFIG_CONTROLLER_2_H
#define CONFIG_CONTROLLER_2_H

#define dt_LPF_1_SPEED_L (1.0 / MAIN_LOOP_FREQ)
#define fc_LPF_1_SPEED_L 3.5

#define dt_LPF_1_SPEED_R (1.0 / MAIN_LOOP_FREQ)
#define fc_LPF_1_SPEED_R 3.5

#define dt_ODOM    (1.0 / MAIN_LOOP_FREQ)

#define Kp_L       1.65 // 1.65
#define Ki_L       12.0 // 12.0
#define Kd_L       0.0
#define dt_L       (1.0 / MAIN_LOOP_FREQ)
#define I_max_L    (0.75 * V_BAT_MAX * SAFETY_FACTOR)
#define u_max_L    (V_BAT_MAX * SAFETY_FACTOR)
#define D_FILTER_L false
#define fc_L       (MAIN_LOOP_FREQ * 0.5)

#define Kp_R       Kp_L
#define Ki_R       Ki_L
#define Kd_R       Kd_L
#define dt_R       dt_L
#define I_max_R    I_max_L
#define u_max_R    u_max_L
#define D_FILTER_R D_FILTER_L
#define fc_R       fc_L

#endif // CONFIG_CONTROLLER_2_H