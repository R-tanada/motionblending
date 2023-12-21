import serial
import time

data = 0
received_data = 1

ser = serial.Serial("/dev/cu.usbmodem1301", 115200, timeout = 0.1)
time.sleep(2)

try:
    while True:
        data += 0.5
        if data == 129:
            data = 0

        ser.write(bytes([int(data)]))
        # received_data = ser.readline().decode("utf-8")
        print(received_data)

        time.sleep(0.005)

except KeyboardInterrupt:
    ser.close()
    print('closed socket')