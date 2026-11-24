/*
	DDR_2020
*/

#include <Arduino.h>
#include "Timer_utils.h"
#include "Com_uart.h"
#include "L298N.h"
#include "Quadrature_enc.h"
#include "LPF_1.h"
#include "PID_P.h"
#include "Wheel_odom.h"
#include "Diff_drive_unicycle.h"
#include "../config/config.h"

// Com_uart comUart;
Timer_utils timer(MAIN_LOOP_FREQ);
L298N motorDriver;
Quadrature_enc encoder_1;
Quadrature_enc encoder_2;

LPF_1<double> lpfSpeedWheel_1;
LPF_1<double> lpfSpeedWheel_2;

PID_P<double> controllerMotor_1;
PID_P<double> controllerMotor_2;

double actualSpeedWheel_1_Unfilt  = 0.0;
double actualSpeedWheel_2_Unfilt  = 0.0;
double actualSpeedWheel_1_Filt = 0.0;
double actualSpeedWheel_2_Filt = 0.0;

Wheel_odom wheel_odom;
Diff_drive_unicycle ddr_uni;

// feedback variables
double actualPositionX_C = 0;	    // x coordinate of the robot
double actualPositionY_C = 0;	    // y coordinate of the robot
double actualAngle_C = 0;	        // orientation of the robot
double actualLinearVelocity_C = 0;  // linear velocity of the robot
double actualAngularVelocity_C = 0; // angular velocity of the robot
double actualSpeedWheel_2 = 0;      // angular velocity of right wheel
double actualSpeedWheel_1 = 0;      // angular velocity of left wheel

// setpoint variables
double targetLinearVelocity_C = 0;      // robot linear velocity setpoint
double targetAngularVelocity_C = 0;     // robot  angular velocity setpoint
double targetLinearVelocity_C_new = 0;  // robot linear velocity setpoint
double targetAngularVelocity_C_new = 0; // robot  angular velocity setpoint
double targetSpeedWheel_1 = 0;          // angular velocity setpoint left wheel
double targetSpeedWheel_2 = 0;          // angular velocity setpoint right wheel
double targetVoltageMotor_1 = 0;        // voltage setpoint left motor
double targetVoltageMotor_2 = 0;        // voltage setpoint right motor

// communication variables
uint8_t drive_mode = Drive_mode::NONE;  // drive mode received
double cmd_1 = 0.0;					    // command 1 received
double cmd_2 = 0.0;					    // command 2 received

void setup()
{
	// comUart.init(Serial, COM_UART_0_BAUDRATE, TIMEOUT_RX_S, MAIN_LOOP_FREQ, COM_UART_TX_FREQ, DDR_ID);
	Serial.begin(COM_UART_0_BAUDRATE);
	
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

	lpfSpeedWheel_1.config(fc_LPF_1_SPEED_L, dt_LPF_1_SPEED_L);
	lpfSpeedWheel_2.config(fc_LPF_1_SPEED_R, dt_LPF_1_SPEED_R);

	controllerMotor_1.init(dt_L, Kp_L, Ki_L, Kd_L, I_max_L, u_max_L, D_FILTER_L, fc_L);
	controllerMotor_2.init(dt_R, Kp_R, Ki_R, Kd_R, I_max_R, u_max_R, D_FILTER_R, fc_R);

	wheel_odom.set_param(MOT_SHAFT_CPR, WHEEL_L, WHEEL_R);
	wheel_odom.set_dt(dt_ODOM);

	ddr_uni.set_param(WHEEL_L, WHEEL_R);
	ddr_uni.set_v_max(V_C_MAX);
	ddr_uni.set_w_max(W_C_MAX);
}

void loop()
{
	// 1. =============================== Odometry ===============================
	// 1.1. Update encoder feedback
	encoder_1.update();
	encoder_2.update();

	actualSpeedWheel_1_Unfilt = encoder_1.getAngularSpeed();
	actualSpeedWheel_2_Unfilt = encoder_2.getAngularSpeed();

	actualSpeedWheel_1_Filt = lpfSpeedWheel_1.update(actualSpeedWheel_1_Unfilt);
	actualSpeedWheel_2_Filt = lpfSpeedWheel_2.update(actualSpeedWheel_2_Unfilt);

	Serial.print(actualSpeedWheel_1_Unfilt * 180.0 / M_PI);
	Serial.print(",");
	Serial.print(actualSpeedWheel_2_Unfilt * 180.0 / M_PI);
	Serial.print(",");
	Serial.print(actualSpeedWheel_1_Filt * 180.0 / M_PI);
	Serial.print(",");
	Serial.println(actualSpeedWheel_2_Filt * 180.0 / M_PI);

	// 1.2. Update odometry
	wheel_odom.update(encoder_1.getCount(), encoder_2.getCount());

	// 1.3. Get odometry feedback
	wheel_odom.get_pose(actualPositionX_C, actualPositionY_C, actualAngle_C);
	wheel_odom.get_twist(actualLinearVelocity_C, actualAngularVelocity_C);
	wheel_odom.get_wheel_speed(actualSpeedWheel_1, actualSpeedWheel_2);

	// 2. ========================== Communication (RX) ==========================
	// 2.1. Get drive mode and command from UART
	// comUart.com_rx(drive_mode, cmd_1, cmd_2);

	// 2.2. Update setpoints based on drive mode and command
	switch (drive_mode)
	{
	case (Drive_mode::NONE):
		targetSpeedWheel_1 = 0.0;
		targetSpeedWheel_2 = 0.0;
		break;
	case (Drive_mode::UNICYCLE):
		targetLinearVelocity_C = cmd_1;
		targetAngularVelocity_C = cmd_2;
		ddr_uni.update_domain_vw(targetLinearVelocity_C, targetAngularVelocity_C, targetLinearVelocity_C_new, targetAngularVelocity_C_new);
		ddr_uni.uni2ddr(targetLinearVelocity_C_new, targetAngularVelocity_C_new, targetSpeedWheel_1, targetSpeedWheel_2);
		break;
	case (Drive_mode::DIFFERENTIAL):
		targetSpeedWheel_1 = cmd_1;
		targetSpeedWheel_2 = cmd_2;
		break;
	default:
		targetSpeedWheel_1 = 0.0;
		targetSpeedWheel_2 = 0.0;
		break;
	}

	// 3. ====================== Run motor speed controller ======================
	// targetVoltageMotor_1 = controllerMotor_1.update(targetSpeedWheel_1, actualSpeedWheel_1_Filt, 0);
	// targetVoltageMotor_2 = controllerMotor_2.update(targetSpeedWheel_2, actualSpeedWheel_2_Filt, 0);
	
	// targetVoltageMotor_1 = 0;
	// targetVoltageMotor_2 = 0;

	// 4. ============================ Command motors ============================
	while (Serial.available()>0)
	{
		targetVoltageMotor_1 = Serial.readStringUntil('\n').toDouble();
		targetVoltageMotor_2 = targetVoltageMotor_1;
	}
	motorDriver.commandMotorA(targetVoltageMotor_1);
	motorDriver.commandMotorB(targetVoltageMotor_2);

	// 5. ========================== Communication (TX) ==========================
	// comUart.com_tx(drive_mode, actualPositionX_C, actualPositionY_C, actualAngle_C, actualLinearVelocity_C, actualAngularVelocity_C, actualSpeedWheel_1, actualSpeedWheel_2);

	timer.sleep();
}
