#ifndef CONFIG_ROBOT_PARAM_H
#define CONFIG_ROBOT_PARAM_H

#define MAIN_LOOP_FREQ   400.0
#define COM_UART_TX_FREQ 100.0
#define TIMEOUT_RX_S     1.0

#define DDR_2020_TYPE_1
// #define DDR_2020_TYPE_2

#if defined(DDR_2020_TYPE_1)
    #include "config_1.h"
#elif defined(DDR_2020_TYPE_1)
    #include "config_2.h"
#endif

#endif // CONFIG_ROBOT_PARAM_H