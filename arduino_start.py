import serial
import time
# 10.7V
arduino_slave = serial.Serial("/dev/ttyACM0", 9600)
print "Connecting to Ardino slave..."
time.sleep(3)
print "ESC Calibrating..."
arduino_slave.write('-1')
time.sleep(5)
arduino_slave.write('-80')
time.sleep(1.2)
arduino_slave.write('-41')
