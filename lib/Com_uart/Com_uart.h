#ifndef COM_UART_H
#define COM_UART_H

#include <Arduino.h>

#pragma pack(push, 1)
struct Rx_packet
{
	uint8_t start = 0x21;
	uint8_t drive_mode = 0x00;
	double cmd_1 = 0x00;
	double cmd_2 = 0x00;
	uint8_t end = 0x23;
};
#pragma pack(pop)

#pragma pack(push, 1)
struct Tx_packet
{
	uint8_t start = 0x21;
	uint8_t id = 0x00;
	unsigned long t_millis = 0x00;
	uint8_t drive_mode = 0x00;
	double x = 0x00;
	double y = 0x00;
	double th = 0x00;
	double v = 0x00;
	double w = 0x00;
	double w_R = 0x00;
	double w_L = 0x00;
	uint8_t end = 0x23;
};
#pragma pack(pop)

class Com_uart
{
public:
	Com_uart();
	void init(HardwareSerial &serial, unsigned long baud, double timeout_rx_s, double main_loop_freq, double com_uart_tx_freq);
	void com_rx(uint8_t &drive_mode, double &cmd_1, double &cmd_2);
	void com_tx(const uint8_t drive_mode, const double x, const double y, const double th, const double v, const double w, const double w_R, const double w_L);

private:
	HardwareSerial *_serial;
	struct Rx_packet rx_pkt;
	struct Tx_packet tx_pkt;

	uint8_t rx_buff[11];
	uint8_t tx_buff[36];

	unsigned long com_uart_tx_counter = 1;
	double main_loop_freq = 100.0;
	double com_uart_tx_freq = 1.0;
	unsigned long com_uart_tx_counter_max = 1;
	uint8_t rx_buff_index = 0;

	double last_t_rx_us = 0.0;
	unsigned long timeout_rx_us = 0.0;
};

#endif
