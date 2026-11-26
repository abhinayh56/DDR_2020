#ifndef COM_UART_H
#define COM_UART_H

#include <Arduino.h>

enum Drive_mode
{
	NONE = 0x00,
	UNICYCLE = 0x01,
	DIFFERENTIAL = 0x02
};

#pragma pack(push, 1)
struct Rx_packet
{
	uint8_t start = 0x21;
	uint8_t drive_mode = 0x00;
	float cmd_1 = 0x00;
	float cmd_2 = 0x00;
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
	float x = 0x00;
	float y = 0x00;
	float th = 0x00;
	float v = 0x00;
	float w = 0x00;
	float w_R = 0x00;
	float w_L = 0x00;
	uint8_t end = 0x23;
};
#pragma pack(pop)

class Com_uart
{
public:
	Com_uart();
	void init(HardwareSerial &serial, unsigned long baud, float timeout_rx_s, float main_loop_freq, float com_uart_tx_freq, uint8_t ddr_id);
	void com_rx(uint8_t &drive_mode, float &cmd_1, float &cmd_2);
	void com_tx(const uint8_t drive_mode, const float x, const float y, const float th, const float v, const float w, const float w_R, const float w_L);

private:
	HardwareSerial *_serial;
	struct Rx_packet rx_pkt;
	struct Tx_packet tx_pkt;

	uint8_t rx_buff[11];
	uint8_t tx_buff[36];

	unsigned long com_uart_tx_counter = 1;
	float main_loop_freq = 100.0;
	float com_uart_tx_freq = 1.0;
	unsigned long com_uart_tx_counter_max = 1;
	uint8_t rx_buff_index = 0;

	float last_t_rx_us = 0.0;
	unsigned long timeout_rx_us = 0.0;
};

#endif
