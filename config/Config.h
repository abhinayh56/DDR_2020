#ifndef CONFIG_ROBOT_PARAM_H
#define CONFIG_ROBOT_PARAM_H

#define DDR_2020_TYPE_1
// #define DDR_2020_TYPE_2

#if defined(DDR_2020_TYPE_1)
    #define MAIN_LOOP_FREQ   400.0
    #define COM_UART_TX_FREQ 100.0
    #define TIMEOUT_RX_S     1.0

    #include "config_elec_connections_1.h"
    #include "config_robot_specs_1.h"
    #include "config_controller_param_1.h"
#elif defined(DDR_2020_TYPE_1)
    #define MAIN_LOOP_FREQ   400.0
    #define COM_UART_TX_FREQ 100.0
    #define TIMEOUT_RX_S     1.0
    
    #include "config_elec_connections_2.h"
    #include "config_robot_specs_2.h"
    #include "config_controller_param_2.h"
#endif

#endif // CONFIG_ROBOT_PARAM_H