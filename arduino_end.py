import serial
import time
arduino_slave = serial.Serial("/dev/ttyACM0", 9600)
arduino_slave.write('-10')
