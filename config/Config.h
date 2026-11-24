#ifndef CONFIG_ROBOT_PARAM_H
#define CONFIG_ROBOT_PARAM_H

// #define DDR_2020_TYPE_1
#define DDR_2020_TYPE_2

#if defined(DDR_2020_TYPE_1)
    #include "config_time_1.h"
    #include "config_elec_connections_1.h"
    #include "config_robot_specs_1.h"
    #include "config_controller_param_1.h"
#elif defined(DDR_2020_TYPE_2)
    #include "config_time_2.h"
    #include "config_elec_connections_2.h"
    #include "config_robot_specs_2.h"
    #include "config_controller_param_2.h"
#endif

#endif // CONFIG_ROBOT_PARAM_H