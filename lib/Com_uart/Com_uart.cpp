#include "Com_uart.h"

Com_uart::Com_uart()
{
}

void Com_uart::init(HardwareSerial &serial, unsigned long baud)
{
    _serial = &serial;
    _serial->begin(baud);
    tx_pkt.drive_mode = 0x00;
    tx_pkt.id = 0x05;
}

void Com_uart::com_rx(uint8_t &drive_mode, double &cmd_1, double &cmd_2)
{
    // start_byte, drive_mode, data_1, data_2, end_byte
    while (_serial->available())
    {
        uint8_t data = _serial->read();
        if (data == 0x21)
        {
            rx_buff_index = 0;
        }
        rx_buff[rx_buff_index] = data;
        rx_buff_index++;

        if (rx_buff_index > 10)
        {
            rx_buff_index = 10;
        }
        if (data == 0x23)
        {
            if (rx_buff[0] == 0x21)
            {
                memcpy(&rx_pkt, rx_buff, 11);
            }
        }
    }

    drive_mode = rx_pkt.drive_mode;
    cmd_1 = rx_pkt.cmd_1;
    cmd_2 = rx_pkt.cmd_2;
}

void Com_uart::com_tx(const uint8_t drive_mode, const double x, const double y, const double th, const double v, const double w)
{
    pkt_comm_counter++;
    if (pkt_comm_counter >= 11)
    {
        pkt_comm_counter = 1;
        tx_pkt.t_millis = millis();
        tx_pkt.drive_mode = drive_mode;
        tx_pkt.x = x;
        tx_pkt.y = y;
        tx_pkt.th = th;
        tx_pkt.v = v;
        tx_pkt.w = w;
        memcpy(tx_buff, &tx_pkt, 28);
        _serial->write(tx_buff, 28);
    }
}