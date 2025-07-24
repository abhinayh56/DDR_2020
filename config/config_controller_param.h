#ifndef CONFIG_CONTROLLER
#define CONFIG_CONTROLLER

#define Kp_R       0.0
#define Ki_R       0.0
#define Kd_R       0.0
#define Kff_R      0.0
#define dt_R       1.0/MAIN_LOOP_FREQ
#define I_max_R    V_BAT_MAX*SAFETY_FACTOR
#define u_max_R    V_BAT_MAX*SAFETY_FACTOR
#define D_FILTER_R false
#define fc_R       MAIN_LOOP_FREQ*0.5

#define Kp_L       0.0
#define Ki_L       0.0
#define Kd_L       0.0
#define Kff_L      0.0
#define dt_L       1.0/MAIN_LOOP_FREQ
#define I_max_L    V_BAT_MAX*SAFETY_FACTOR
#define u_max_L    V_BAT_MAX*SAFETY_FACTOR
#define D_FILTER_L false
#define fc_L       MAIN_LOOP_FREQ*0.5

#endif