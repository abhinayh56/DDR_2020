import serial
import type_conversion as tc

port = '/dev/ttyACM0'
baudrate = 115200
timeout = 1.0

rx_pkt = []

try:
	ser = serial.Serial(port, baudrate, timeout=timeout)
	print(f"INFO   : Serial port {port} opened")
except serial.SerialException as exception_e:
	print(f"ERROR  : Failed to open serial port {port}: {exception_e}")
	exit()
	
ser.close()

def decode_rx_pkt(rx_pkt_):
    start, id, t_millis, drive_mode, x, y, th, v, w, end = 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
    start      = rx_pkt_[0]
    id         = rx_pkt_[1]
    t_millis   = tc.byte_4_2_UDINT(rx_pkt_[2:6])
    drive_mode = rx_pkt_[6]
    x          = tc.byte_4_2_FLOAT(rx_pkt_[7:11])
    y          = tc.byte_4_2_FLOAT(rx_pkt_[11:15])
    th         = tc.byte_4_2_FLOAT(rx_pkt_[15:19])
    v          = tc.byte_4_2_FLOAT(rx_pkt_[19:23])
    w          = tc.byte_4_2_FLOAT(rx_pkt_[23:27])
    end        = rx_pkt_[27]
    return start, id, t_millis/1000.0, drive_mode, x, y, th, v, w, end	

ser.open()

while True:
    data_byte_arr = ser.read_all()
    for i in range(0,len(data_byte_arr)):
        data_byte = data_byte_arr[i]
        if(data_byte==0x21):
            rx_pkt = []
        rx_pkt.append(data_byte)
        if(data_byte==0x23):
            if(len(rx_pkt)==28):
                # decode_rx_pkt(rx_pkt)
                print(decode_rx_pkt(rx_pkt))