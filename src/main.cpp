/*
	DDR_2020
*/

#include <Arduino.h>
#include "Timer_utils.h"
// #include "Com_uart.h"
#include "L298N.h"
#include "Quadrature_enc.h"
#include "LPF_1.h"
#include "Motor_model.h"
#include "PID_P.h"
#include "Wheel_odom.h"
// #include "Diff_drive_unicycle.h"
#include "../config/config.h"

// Com_uart comUart;
Timer_utils timer(MAIN_LOOP_FREQ);
L298N motorDriver;
Quadrature_enc encoder_1;
Quadrature_enc encoder_2;

LPF_1<double> lpfSpeedWheel_1;
LPF_1<double> lpfSpeedWheel_2;

// Motor_model<double> motorModel_1;
// Motor_model<double> motorModel_2;

PID_P<double> controllerMotor_1;
PID_P<double> controllerMotor_2;

uint8_t num_data_motor_1 = 49;
uint8_t num_data_motor_2 = 49;
double data_wv_motor_1[][2] = 
{
	{-388,-12},
	{-365,-11.5},
	{-356,-11},
	{-356,-10.5},
	{-351,-10},
	{-345,-9.5},
	{-340,-9},
	{-334,-8.5},
	{-326,-8},
	{-317,-7.5},
	{-306,-7},
	{-295,-6.5},
	{-280,-6},
	{-262,-5.5},
	{-243,-5},
	{-215,-4.5},
	{-185,-4},
	{-146,-3.5},
	{-99,-3},
	{-54,-2.5},
	{0,-2},
	{0,-1.5},
	{0,-1},
	{0,-0.5},
	{0,0},
	{0,0.5},
	{0,1},
	{0,1.5},
	{0,2},
	{51,2.5},
	{94,3},
	{139,3.5},
	{176,4},
	{205,4.5},
	{231,5},
	{251,5.5},
	{270,6},
	{285,6.5},
	{296,7},
	{308,7.5},
	{317,8},
	{324,8.5},
	{331,9},
	{336,9.5},
	{342,10},
	{347,10.5},
	{347,11},
	{355,11.5},
	{380,12}
};
double data_wv_motor_2[][2] = 
{
	{-374,-12},
	{-347,-11.5},
	{-338,-11},
	{-339,-10.5},
	{-334,-10},
	{-329,-9.5},
	{-323,-9},
	{-317,-8.5},
	{-311,-8},
	{-302,-7.5},
	{-292,-7},
	{-279,-6.5},
	{-263,-6},
	{-243,-5.5},
	{-220,-5},
	{-192,-4.5},
	{-160,-4},
	{-116,-3.5},
	{-63,-3},
	{-25,-2.5},
	{0,-2},
	{0,-1.5},
	{0,-1},
	{0,-0.5},
	{0,0},
	{0,0.5},
	{0,1},
	{0,1.5},
	{0,2},
	{35,2.5},
	{87,3},
	{139,3.5},
	{177,4},
	{206,4.5},
	{236,5},
	{257,5.5},
	{276,6},
	{292,6.5},
	{301,7},
	{314,7.5},
	{324,8},
	{332,8.5},
	{339,9},
	{344,9.5},
	{350,10},
	{356,10.5},
	{357,11},
	{365,11.5},
	{391,12}
};

double actualSpeedWheel_1_Unfilt  = 0.0;
double actualSpeedWheel_2_Unfilt  = 0.0;
double actualSpeedWheel_1_Filt = 0.0;
double actualSpeedWheel_2_Filt = 0.0;

Wheel_odom wheel_odom;
// Diff_drive_unicycle ddr_uni;

// feedback variables
double actualPositionX_C = 0;	    // x coordinate of the robot
double actualPositionY_C = 0;	    // y coordinate of the robot
double actualAngle_C = 0;	        // orientation of the robot
double actualLinearVelocity_C = 0;  // linear velocity of the robot
double actualAngularVelocity_C = 0; // angular velocity of the robot
double actualSpeedWheel_2 = 0;      // angular velocity of right wheel
double actualSpeedWheel_1 = 0;      // angular velocity of left wheel

// setpoint variables
// double targetLinearVelocity_C = 0;  // robot linear velocity setpoint
// double targetAngularVelocity_C = 0; // robot  angular velocity setpoint
double targetSpeedWheel_1 = 0;      // angular velocity setpoint left wheel
double targetSpeedWheel_2 = 0;      // angular velocity setpoint right wheel
double targetVoltageMotor_1 = 0;    // voltage setpoint left motor
double targetVoltageMotor_2 = 0;    // voltage setpoint right motor
double voltageFFMotor_1 = 0;        // voltage feed forward left motor
double voltageFFMotor_2 = 0;        // voltage feed forward right motor

// communication variables
// uint8_t drive_mode = Drive_mode::NONE;  // drive mode received
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

	lpfSpeedWheel_1.config(fc_LPF_1_SPEED_L, dt_LPF_1_SPEED_L);
	lpfSpeedWheel_2.config(fc_LPF_1_SPEED_R, dt_LPF_1_SPEED_R);

	// motorModel_1.assign_data(data_wv_motor_1, num_data_motor_1);
	// motorModel_2.assign_data(data_wv_motor_2, num_data_motor_2);

	controllerMotor_1.init(dt_L, Kp_L, Ki_L, Kd_L, I_max_L, u_max_L, D_FILTER_L, fc_L);
	controllerMotor_2.init(dt_R, Kp_R, Ki_R, Kd_R, I_max_R, u_max_R, D_FILTER_R, fc_R);

	wheel_odom.set_param(MOT_SHAFT_CPR, WHEEL_L, WHEEL_R);
	wheel_odom.set_dt(dt_ODOM);

	// ddr_uni.set_param(WHEEL_L, WHEEL_R);
	// ddr_uni.set_v_max(V_C_MAX);
	// ddr_uni.set_w_max(W_C_MAX);
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

	// Serial.print((double)encoder_1.getCount());
	// Serial.print(",");
	// Serial.print((double)encoder_2.getCount());
	// Serial.print(",");

	// Serial.print(actualSpeedWheel_1_Unfilt * 180.0 / M_PI);
	// Serial.print(",");
	// Serial.print(actualSpeedWheel_2_Unfilt * 180.0 / M_PI);
	// Serial.print(",");
	// Serial.print(actualSpeedWheel_1_Filt * 180.0 / M_PI);
	// Serial.print(",");
	// Serial.print(actualSpeedWheel_2_Filt * 180.0 / M_PI);
	// Serial.print(",");

	// 1.2. Update odometry
	wheel_odom.update(encoder_1.getCount(), encoder_2.getCount());

	// 1.3. Get odometry feedback
	wheel_odom.get_pose(actualPositionX_C, actualPositionY_C, actualAngle_C);
	wheel_odom.get_twist(actualLinearVelocity_C, actualAngularVelocity_C);
	wheel_odom.get_wheel_speed(actualSpeedWheel_1, actualSpeedWheel_2);

	// Serial.print(actualPositionX_C * 1000.0);
	// Serial.print(",");
	// Serial.print(actualPositionY_C * 1000.0);
	// Serial.print(",");
	// Serial.print(actualAngle_C * 180.0 / M_PI);
	// Serial.print(",");

	// 2. ========================== Communication (RX) ==========================
	// 2.1. Get drive mode and command from UART
	// comUart.com_rx(drive_mode, cmd_1, cmd_2);

	// 2.2. Update setpoints based on drive mode and command
	// switch (drive_mode)
	// {
	// case (Drive_mode::NONE):
	// 	targetSpeedWheel_2 = 0.0;
	// 	targetSpeedWheel_1 = 0.0;
	// 	break;
	// case (Drive_mode::UNICYCLE):
	// 	targetLinearVelocity_C = cmd_1;
	// 	targetAngularVelocity_C = cmd_2;
	// 	ddr_uni.update_domain_vw(targetLinearVelocity_C, targetAngularVelocity_C, targetLinearVelocity_C, targetAngularVelocity_C);
	// 	ddr_uni.uni2ddr(targetLinearVelocity_C, targetAngularVelocity_C, targetSpeedWheel_2, targetSpeedWheel_1);
	// 	break;
	// case (Drive_mode::DIFFERENTIAL):
	// 	targetSpeedWheel_2 = cmd_1;
	// 	targetSpeedWheel_1 = cmd_2;
	// 	break;
	// default:
	// 	targetSpeedWheel_2 = 0.0;
	// 	targetSpeedWheel_1 = 0.0;
	// 	break;
	// }

	// 3. ====================== Run motor speed controller ======================
	while(Serial.available()>0)
	{
		targetSpeedWheel_1 = Serial.readStringUntil('\n').toDouble() * M_PI / 180.0;
		targetSpeedWheel_2 = targetSpeedWheel_1;
	}

	Serial.print(targetSpeedWheel_1 * 180.0 / M_PI);
	Serial.print(",");
	Serial.print(targetSpeedWheel_2 * 180.0 / M_PI);
	Serial.print(",");
	Serial.print(actualSpeedWheel_1_Filt * 180.0 / M_PI);
	Serial.print(",");
	Serial.print(actualSpeedWheel_2_Filt * 180.0 / M_PI);
	Serial.print(",");

	voltageFFMotor_1 = 0; // motorModel_1.getV(targetSpeedWheel_1 * 180.0 / M_PI);
	voltageFFMotor_2 = 0; // motorModel_2.getV(targetSpeedWheel_2 * 180.0 / M_PI);

	Serial.print(voltageFFMotor_1);
	Serial.print(",");
	Serial.print(voltageFFMotor_2);
	Serial.print(",");

	voltageFFMotor_1 = saturate<double>(voltageFFMotor_1, -V_BAT_MAX, V_BAT_MAX);
	voltageFFMotor_2 = saturate<double>(voltageFFMotor_2, -V_BAT_MAX, V_BAT_MAX);

	targetVoltageMotor_1 = controllerMotor_1.update(targetSpeedWheel_1, actualSpeedWheel_1_Filt, voltageFFMotor_1);
	targetVoltageMotor_2 = controllerMotor_2.update(targetSpeedWheel_2, actualSpeedWheel_2_Filt, voltageFFMotor_2);
	
	// 4. ============================ Command motors ============================
	motorDriver.commandMotorA(targetVoltageMotor_1);
	motorDriver.commandMotorB(targetVoltageMotor_2);

	Serial.print(targetVoltageMotor_1);
	Serial.print(",");
	Serial.print(targetVoltageMotor_2);
	Serial.print(",");
	// Serial.print(actualSpeedWheel_1_Filt * 180.0 / M_PI);
	// Serial.print(",");
	// Serial.print(actualSpeedWheel_2_Filt * 180.0 / M_PI);
	// Serial.print(",");

	// 5. ========================== Communication (TX) ==========================
	// comUart.com_tx(drive_mode, actualPositionX_C, actualPositionY_C, actualAngle_C, actualLinearVelocity_C, actualAngularVelocity_C, actualSpeedWheel_2, actualSpeedWheel_1);

	timer.sleep();

	t = micros();
	Serial.println(0);
	t_last = t;
}
