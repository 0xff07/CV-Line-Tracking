import time
import pigpio 

class ARSCHLOCH:
    def __init__(self):
        self.STEER = 19
        self.FAN_ANG = 13
        self.ESC = 6
        self.SONAR_ECHO = 27
        self.SONAR_TRIG = 17
        self.S_MID = 16
        self.S_AMP = 5
        self.SERVO_PIN = [self.STEER, self.FAN_ANG]
        self.ESC_PIN = [self.ESC]
        self.SENSOR_PIN_OUT = [self.SONAR_ECHO]
        self.SENSOR_PIN_IN = [self.SONAR_TRIG]
        self.PWM_TEST_SEQ = [self.S_MID, self.S_MID + self.S_AMP, self.S_MID, self.S_MID - self.S_AMP]
        self.pi = pigpio.pi()

    def turn_on(self):
        for i in self.SERVO_PIN + self.ESC_PIN + self.SENSOR_PIN_OUT:
            self.pi.set_mode(i, pigpio.OUTPUT)
            self.pi.set_PWM_frequency(i, 50)
            self.pi.set_PWM_dutycycle(i, self.S_MID)
        for i in self.SENSOR_PIN_IN:
            self.pi.set_mode(i, pigpio.INPUT)
            self.pi.set_PWM_frequency(i, 50)

    def turn_off(self):
        print "exit routing start..."
        print "Halt BLDC..."
        for i in self.ESC_PIN:
            self.pi.set_PWM_dutycycle(i, 0)
        print "Halt Servos..."
        for i in self.SERVO_PIN:
            self.pi.set_PWM_dutycycle(i, self.S_MID)
        time.sleep(1)
        for i in self.SERVO_PIN:
            self.pi.set_PWM_dutycycle(i, 0)
        print "Clean Sensor Pin..."
        for i in self.SENSOR_PIN_IN:
            self.pi.set_PWM_dutycycle(i, 0)
        self.pi.stop()

    def accelerate(self, speed = 1):
        if speed < 0:
            duty = 10
        elif speed > 30:
            duty = 30
        else:
            duty = speed + 10
        self.pi.set_PWM_dutycycle(self.ESC, duty)

    def steer(self, theta):
        if theta < 35:
            theta = 35
        elif theta > 145:
            theta = 145
        else:
            duty = int(round((35. - theta)/11 + 21))
            self.pi.set_PWM_dutycycle(self.STEER, duty)

    def test_servo(self):
        for i in self.PWM_TEST_SEQ:
            for j in self.SERVO_PIN:
                self.pi.set_PWM_dutycycle(j, i)
            time.sleep(1)

    def callibrate_ESC(self):
        print "calibration high ..."
        self.pi.set_PWM_dutycycle(self.ESC, 30)
        time.sleep(2)
        print "calibration low ..."
        self.pi.set_PWM_dutycycle(self.ESC, 10)
        time.sleep(2)
        self.pi.set_PWM_dutycycle(self.ESC, 10)
        print "Finish calibration !"

if __name__ == "__main__":
    import atexit
    arschloch = ARSCHLOCH()
    arschloch.turn_on()

    atexit.register(arschloch.turn_off)
    try:
        arschloch.callibrate_ESC()
        arschloch.accelerate(1)
        time.sleep(5)
        while 1:
            for i in arschloch.SERVO_PIN:
                arschloch.test_servo()
    except:
        exit()
