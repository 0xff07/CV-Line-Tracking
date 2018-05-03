import serial
import time
arduino_slave = serial.Serial("/dev/ttyACM0", 9600)
print "Connecting to Ardino slave..."
time.sleep(3)
print "ESC Calibrating..."
arduino_slave.write('-1')
time.sleep(5)
arduino_slave.write('-60')
time.sleep(1)
arduino_slave.write('-36')
