#ifndef CONFIG_CONTROLLER_1_H
#define CONFIG_CONTROLLER_1_H

#define dt_LPF_1_SPEED_L 1.0/MAIN_LOOP_FREQ
#define fc_LPF_1_SPEED_L 6.0

#define dt_LPF_1_SPEED_R 1.0/MAIN_LOOP_FREQ
#define fc_LPF_1_SPEED_R 6.0

#define dt_ODOM    1.0/MAIN_LOOP_FREQ

#define Kp_L       5.0
#define Ki_L       1000 bn\.0
#define Kd_L       0.0
#define Kff_L      0.0
#define dt_L       1.0/MAIN_LOOP_FREQ
#define I_max_L    V_BAT_MAX*SAFETY_FACTOR
#define u_max_L    V_BAT_MAX*SAFETY_FACTOR
#define D_FILTER_L false
#define fc_L       50.0

#define Kp_R       0.0
#define Ki_R       0.0
#define Kd_R       0.0
#define Kff_R      0.0
#define dt_R       1.0/MAIN_LOOP_FREQ
#define I_max_R    V_BAT_MAX*SAFETY_FACTOR
#define u_max_R    V_BAT_MAX*SAFETY_FACTOR
#define D_FILTER_R false
#define fc_R       50.0

#endif // CONFIG_CONTROLLER_1_H