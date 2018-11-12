import serial
ser = serial.Serial('COM3',9600)

import time
file = open('jointAngle_test.csv')
line = file.readline()

ser.write(line.encode('latin-1'))
time.sleep(5)
file.close
