/*
	DDR_2020
*/

#include <Arduino.h>
#include "Timer_utils.h"
// #include "Com_uart.h"
#include "L298N.h"
#include "Quadrature_enc.h"
// #include "LPF_1.h"
// #include "PID_P.h"
// #include "Wheel_odom.h"
// #include "Diff_drive_unicycle.h"
#include "../config/config.h"

// Com_uart comUart;
Timer_utils timer(MAIN_LOOP_FREQ);
L298N motorDriver;
Quadrature_enc encoder_1;
Quadrature_enc encoder_2;

// LPF_1<double> lpfSpeedEncoder_1;
// LPF_1<double> lpfSpeedEncoder_2;
// PID_P<double> controllerMotor_1;
// PID_P<double> controllerMotor_2;

// double actualSpeedMotor_1_Unfilt  = 0.0;
// double actualSpeedMotor_2_Unfilt  = 0.0;
// double actualSpeedMotor_1_Filt = 0.0;
// double actualSpeedMotor_2_Filt = 0.0;

// Wheel_odom wheel_odom;
// Diff_drive_unicycle ddr_uni;

// feedback variables
// double x = 0;	// x coordinate of the robot
// double y = 0;	// y coordinate of the robot
// double th = 0;	// orientation of the robot
// double v = 0;	// linear velocity of the robot
// double w = 0;	// angular velocity of the robot
// double w_R = 0; // angular velocity of right wheel
// double w_L = 0; // angular velocity of left wheel

// setpoint variables
// double v_0 = 0;	  // robot linear velocity setpoint
// double w_0 = 0;	  // robot  angular velocity setpoint
// double w_R_0 = 0; // angular velocity setpoint right wheel
// double w_L_0 = 0; // angular velocity setpoint left wheel
double v_R_0 = 0; // voltage setpoint right motor
double v_L_0 = 0; // voltage setpoint left motor
// double PWM_R = 0; // pwm setpoint right motor
// double PWM_L = 0; // pwm setpoint left motor

// communication variables
// uint8_t drive_mode = Drive_mode::NONE; // drive mode received
// double cmd_1 = 0.0;					   // command 1 received
// double cmd_2 = 0.0;					   // command 2 received

uint64_t t_last = 0;
uint64_t t = 0;

void setup()
{
	// comUart.init(Serial, COM_UART_0_BAUDRATE, TIMEOUT_RX_S, MAIN_LOOP_FREQ, COM_UART_TX_FREQ);
	Serial.begin(115200);

	timer.init(MAIN_LOOP_FREQ);

	motorDriver.config(V_BAT_MAX, PWM_MAX);
	motorDriver.configMotorA(MOTOR_1_PIN_PWM, MOTOR_1_PIN_A, MOTOR_1_PIN_B, MOTOR_1_REVERSE);
	motorDriver.configMotorB(MOTOR_2_PIN_PWM, MOTOR_2_PIN_A, MOTOR_2_PIN_B, MOTOR_2_REVERSE);

	motorDriver.init();
	motorDriver.initMotorA();
	motorDriver.initMotorB();

	motorDriver.enableMotorA();
	motorDriver.enableMotorB();

	encoder_1.config(MOT_SHAFT_CPR, ENC_1_PIN_A, ENC_1_PIN_B, []() { encoder_1.isrFunction(); }, ENC_1_REVERSE);
	encoder_2.config(MOT_SHAFT_CPR, ENC_2_PIN_A, ENC_2_PIN_B, []() { encoder_2.isrFunction(); }, ENC_2_REVERSE);

	encoder_1.init();
	encoder_2.init();

	// lpfSpeedEncoder_1.config(fc_LPF_1_SPEED_R, dt_LPF_1_SPEED_R);
	// lpfSpeedEncoder_2.config(fc_LPF_1_SPEED_L, dt_LPF_1_SPEED_L);

	// controllerMotor_1.init(dt_R, Kp_R, Ki_R, Kd_R, I_max_R, u_max_R, false, fc_R);
	// controllerMotor_2.init(dt_L, Kp_L, Ki_L, Kd_L, I_max_L, u_max_L, false, fc_L);

	// wheel_odom.set_param(MOT_SHAFT_CPR, WHEEL_R, WHEEL_L);
	// wheel_odom.set_dt(dt_ODOM);

	// ddr_uni.set_param(WHEEL_R, WHEEL_L);
	// ddr_uni.set_v_max(V_C_MAX);
	// ddr_uni.set_w_max(W_C_MAX);
}

void loop()
{
	// 1. =============================== Odometry ===============================
	// 1.1. Update encoder feedback
	encoder_1.update();
	encoder_2.update();

	Serial.print((long)encoder_1.getCount());
	Serial.print("\t");
	Serial.print((long)encoder_2.getCount());
	Serial.print("\t");

	// actualSpeedMotor_1_Unfilt = encoder_1.getAngularSpeed();
	// actualSpeedMotor_2_Unfilt = encoder_2.getAngularSpeed();

	// actualSpeedMotor_1_Filt = lpfSpeedEncoder_1.update(actualSpeedMotor_1_Unfilt);
	// actualSpeedMotor_2_Filt = lpfSpeedEncoder_2.update(actualSpeedMotor_2_Unfilt);

	// 1.2. Update odometry
	// wheel_odom.update(encoder_1.getCount(), encoder_2.getCount());

	// 1.3. Get odometry feedback
	// wheel_odom.get_pose(x, y, th);
	// wheel_odom.get_twist(v, w);
	// wheel_odom.get_wheel_speed(w_R, w_L);

	// 2. ========================== Communication (RX) ==========================
	// 2.1. Get drive mode and command from UART
	// comUart.com_rx(drive_mode, cmd_1, cmd_2);

	// 2.2. Update setpoints based on drive mode and command
	// switch (drive_mode)
	// {
	// case (Drive_mode::NONE):
	// 	w_R_0 = 0.0;
	// 	w_L_0 = 0.0;
	// 	break;
	// case (Drive_mode::UNICYCLE):
	// 	v_0 = cmd_1;
	// 	w_0 = cmd_2;
	// 	ddr_uni.update_domain_vw(v_0, w_0, v_0, w_0);
	// 	ddr_uni.uni2ddr(v_0, w_0, w_R_0, w_L_0);
	// 	break;
	// case (Drive_mode::DIFFERENTIAL):
	// 	w_R_0 = cmd_1;
	// 	w_L_0 = cmd_2;
	// 	break;
	// default:
	// 	w_R_0 = 0.0;
	// 	w_L_0 = 0.0;
	// 	break;
	// }

	// 3. ====================== Run motor speed controller ======================
	// v_R_0 = controllerMotor_1.update(w_R_0, actualSpeedMotor_1_Filt, D_FILTER_R);
	// v_L_0 = controllerMotor_2.update(w_L_0, actualSpeedMotor_2_Filt, D_FILTER_L);
	
	// 4. ============================ Command motors ============================
	motorDriver.commandMotorA(v_R_0);
	motorDriver.commandMotorB(v_L_0);

	// 5. ========================== Communication (TX) ==========================
	// comUart.com_tx(drive_mode, x, y, th, v, w, w_R, w_L);

	timer.sleep();

	t = micros();
	Serial.println(1000000.0/double(t - t_last));
	t_last = t;
}
