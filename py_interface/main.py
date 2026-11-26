print('GCS DDR2020')

import time
import pygame
import serial
import type_conversion as tc
import serial.tools.list_ports

port = ""

print("===")
print("Available serial ports:")
port_list = []
for i, port in enumerate(serial.tools.list_ports.comports()):
    print("    ", i + 1 , ". ", port.device, "-", port.description)
    port_list.append(port.device)

user_input = int(input("Select the serial port to connect to the DDR2018 robot. (Enter the number corresponding to the port): "))
port_name = port_list[user_input - 1]

ser = serial.Serial()
ser.baudrate = 115200
# ser.port = '/dev/rfcomm3'
ser.port = port_name
ser.timeout = 1.0

ser.close()
time.sleep(1)

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
    w_R        = tc.byte_4_2_FLOAT(rx_pkt_[27:31])
    w_L        = tc.byte_4_2_FLOAT(rx_pkt_[31:35])
    end        = rx_pkt_[27]
    return start, id, t_millis/1000.0, drive_mode, x, y, th, v, w, w_L, w_R, end

ser.open()

print("===")

pygame.init()
screen = pygame.display.set_mode((400, 300))
pygame.display.set_caption("Keyboard Input")
running = True

def limit(n, min_val, max_val):
    return max(min_val, min(n, max_val))

def send_command(drive_mode_, cmd_1_, cmd_2_):

    cmd_1 = tc.REAL_2_byte_4(cmd_1_)
    cmd_2 = tc.REAL_2_byte_4(cmd_2_)

    cmd_1_0 = cmd_1[0]
    cmd_1_1 = cmd_1[1]
    cmd_1_2 = cmd_1[2]
    cmd_1_3 = cmd_1[3]

    cmd_2_0 = cmd_2[0]
    cmd_2_1 = cmd_2[1]
    cmd_2_2 = cmd_2[2]
    cmd_2_3 = cmd_2[3]

    tx_pkt = [0x21, drive_mode_, cmd_1_0, cmd_1_1, cmd_1_2, cmd_1_3, cmd_2_0, cmd_2_1, cmd_2_2, cmd_2_3, 0x23]

    tx_pkt = bytearray(tx_pkt)

    try:
        ser.write(tx_pkt)
        print('---> Tx : ', "2", ", ", drive_mode_, ", ", cmd_1_, ', ', cmd_2_)
    except:
        print('---> Tx : ', 'XX_1, XX_2, XX_3')

rx_pkt = []


rx_pkt = bytearray()

def receive_command():
    global rx_pkt
    # Read available bytes without blocking
    data = ser.read(ser.in_waiting)

    for data_byte in data:
        # Detect packet start
        if data_byte == 0x21:
            rx_pkt = bytearray()
            rx_pkt.append(data_byte)
        elif len(rx_pkt) > 0:
            rx_pkt.append(data_byte)
            # Detect packet end
            if data_byte == 0x23:
                if len(rx_pkt) == 36:
                    try:
                        start, id, t, drive_mode, x, y, th, v, w, w_L, w_R, end = decode_rx_pkt(rx_pkt)
                        print("<--- Rx : ", id, ", ", drive_mode, ", ", v, ", ", w, ", ", t)
                    except Exception as e:
                        print("RX Decode Error:", e)
                        print('<--- Rx : ', 'XX_1, XX_2, XX_3, XX_4, XX_5, XX_6, XX_7, XX_8', "RX Error!")

                # Reset packet buffer
                rx_pkt = bytearray()

            # Prevent buffer overflow
            elif len(rx_pkt) > 36:
                rx_pkt = bytearray()

t0 = time.time()

v, w = 0.0, 0.0

while running:
    # Process pygame events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Get current keyboard state
    keys = pygame.key.get_pressed()

    dv = 0.01
    dw = 0.1

    if keys[pygame.K_UP]:
        print("UP")
        v = v + dv

    if keys[pygame.K_DOWN]:
        print("DOWN")
        v = v - dv

    if keys[pygame.K_LEFT]:
        print("LEFT")
        w = w + dw

    if keys[pygame.K_RIGHT]:
        print("RIGHT")
        w = w - dw

    v_max = 0.185
    w_max = 1.85

    # v_max = 0.296
    # w_max = 2.96

    v = limit(v, -v_max, v_max)
    w = limit(w, -w_max, w_max)

    # ESC to quit
    if keys[pygame.K_ESCAPE]:
        running = False

    if(not running):
        send_command(0, 0.0, 0.0)
        time.sleep(0.25)
        break

    send_command(1, v, w)
    receive_command()

    pygame.time.Clock().tick(25)
    time.sleep(0.001)

pygame.quit()
ser.close()
