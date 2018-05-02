import serial
import time
arduino_slave = serial.Serial("/dev/ttyACM0", 9600)
time.sleep(3)
print "y..."
arduino_slave.write('y')
time.sleep(12)
arduino_slave.write('s')
print "s..."
time.sleep(3)
arduino_slave.write('24')
print "24..."
time.sleep(10)
print "o..."
arduino_slave.write('o')
