import serial
import type_conversion as tc

ser = serial.Serial()
ser.baudrate = 115200
ser.timeout = 0
ser.port = 'COM1'

def decode_rx_pkt(rx_pkt):
    start      = tc.byte_1_2_BYTE(rx_pkt[0])
    id         = tc.byte_1_2_BYTE(rx_pkt[1])
    t_millis   = tc.byte_4_2_UDINT(rx_pkt[2:6])
    drive_mode = tc.byte_1_2_BYTE(rx_pkt[6])
    x          = tc.byte_4_2_FLOAT(rx_pkt[7:11])
    y          = tc.byte_4_2_FLOAT(rx_pkt[11:15])
    th         = tc.byte_4_2_FLOAT(rx_pkt[15:19])
    v          = tc.byte_4_2_FLOAT(rx_pkt[19:23])
    w          = tc.byte_4_2_FLOAT(rx_pkt[23:27])
    end        = tc.byte_1_2_BYTE(rx_pkt[27])
    return start, id, t_millis, drive_mode, x, y, th, v, w, end

data_arr = bytearray()

ser.open()

while True:
    # read data
    while(ser.in_waiting>0):
        data = ser.read(size=1)
        print(data)
        if(data==0x21):
            data_arr = bytearray()
        data_arr.append(data)
        if(data==0x23):
            if(data_arr[0]==0x21):
                # decode received packet
                start, id, t_millis, drive_mode, x, y, th, v, w, end = decode_rx_pkt(data_arr)
                print("start, id, t_millis, drive_mode, x, y, th, v, w, end: ", start, id, t_millis, drive_mode, x, y, th, v, w, end)
    
    # write data
