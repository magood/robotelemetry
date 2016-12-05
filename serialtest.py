import serial

ser = serial.Serial('/dev/serial0', 9600, timeout=4)
lines = []
for i in range(1000):
    lines.append(ser.readline())

with open('gps_test_output.txt', 'w') as f:
    for line in lines:
        f.write(line)
