/*
	DDR_2020
*/

#include <Arduino.h>
#include "Com_uart.h"
#include "Motor_interface.h"
#include "Encoder_interface.h"
#include "Wheel_odom.h"
#include "Timer_utils.h"
#include "PID_controller.h"
#include "Diff_drive_unicycle.h"
#include "../config/Config.h"

Com_uart com_uart;
Timer_utils timer(MAIN_LOOP_FREQ);
Motor_interface motor_interface;
volatile long long Encoder_interface::count1 = 0;
volatile long long Encoder_interface::count2 = 0;
Encoder_interface encoder_interface;
Wheel_odom wheel_odom;
Diff_drive_unicycle ddr_uni;
PID_controller controller_R, controller_L;

// feedback variables
long long count_r = 0; // right wheel encoder count
long long count_l = 0; // left wheel encoder count

double x = 0;	// x coordinate of the robot
double y = 0;	// y coordinate of the robot
double th = 0;	// orientation of the robot
double v = 0;	// linear velocity of the robot
double w = 0;	// angular velocity of the robot
double w_R = 0; // angular velocity of right wheel
double w_L = 0; // angular velocity of left wheel

// setpoint variables
double v_0 = 0;	  // robot linear velocity setpoint
double w_0 = 0;	  // robot  angular velocity setpoint
double w_R_0 = 0; // angular velocity setpoint right wheel
double w_L_0 = 0; // angular velocity setpoint left wheel
double v_R_0 = 0; // voltage setpoint right motor
double v_L_0 = 0; // voltage setpoint left motor
double PWM_R = 0; // pwm setpoint right motor
double PWM_L = 0; // pwm setpoint left motor

// communication variables
uint8_t drive_mode = Drive_mode::NONE; // drive mode received
double cmd_1 = 0.0;					   // command 1 received
double cmd_2 = 0.0;					   // command 2 received

void setup()
{
	com_uart.init(Serial, 115200, TIMEOUT_RX_S, MAIN_LOOP_FREQ, COM_UART_TX_FREQ);

	wheel_odom.set_param(MOT_SHAFT_CPR, WHEEL_R, WHEEL_L);
	wheel_odom.set_dt(1.0 / MAIN_LOOP_FREQ);

	ddr_uni.set_param(WHEEL_R, WHEEL_L);
	ddr_uni.set_v_max(V_C_MAX);
	ddr_uni.set_w_max(W_C_MAX);

	timer.init(MAIN_LOOP_FREQ);

	encoder_interface.config();

	motor_interface.config();
	motor_interface.command_voltage(0, 0);

	controller_R.set_param(Kp_R, Ki_R, Kd_R, Kff_R, dt_R, I_max_R, u_max_R, fc_R);
	controller_L.set_param(Kp_L, Ki_L, Kd_L, Kff_L, dt_L, I_max_L, u_max_L, fc_L);
}

void loop()
{
	// 1. =============================== Odometry ===============================
	// 1.1. Get encoder feedback
	noInterrupts();
	count_r = Encoder_interface::count1;
	count_l = Encoder_interface::count2;
	interrupts();

	// 1.2. Update odometry
	wheel_odom.update(count_r, count_l);

	// 1.3. Get odometry feedback
	wheel_odom.get_pose(x, y, th);
	wheel_odom.get_twist(v, w);
	wheel_odom.get_wheel_speed(w_R, w_L);

	// 2. ========================== Communication (RX) ==========================
	com_uart.com_rx(drive_mode, cmd_1, cmd_2);

	switch (drive_mode)
	{
	case (Drive_mode::NONE):
		w_R_0 = 0.0;
		w_L_0 = 0.0;
		break;
	case (Drive_mode::UNICYCLE):
		// receive: v_0, w_0
		v_0 = cmd_1;
		w_0 = cmd_2;
		ddr_uni.update_domain_vw(v_0, w_0, v_0, w_0);
		ddr_uni.uni2ddr(v_0, w_0, w_R_0, w_L_0);
		break;
	case (Drive_mode::DIFFERENTIAL):
		// receive: w_R_0, w_L_0
		w_R_0 = cmd_1;
		w_L_0 = cmd_2;
		break;
	default:
		w_R_0 = 0.0;
		w_L_0 = 0.0;
		break;
	}

	// 3. ======================== Wheel speed controller ========================
	v_R_0 = controller_R.update(w_R_0, w_R, D_FILTER_R);
	v_L_0 = controller_L.update(w_L_0, w_L, D_FILTER_L);

	// 4. ============================ Command motors ============================
	motor_interface.command_voltage(v_R_0, v_L_0);

	// 5. ========================== Communication (TX) ==========================
	com_uart.com_tx(drive_mode, x, y, th, v, w, w_R, w_L);

	timer.sleep();
}
