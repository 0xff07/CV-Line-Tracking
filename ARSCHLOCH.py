import time
import pigpio 
import sonar_ranger
import numpy as np

class ARSCHLOCH:
    def __init__(self):
        self.STEER = 19
        self.FAN_ANG = 13
        self.ESC = 6
        self.SONAR_ECHO = 17
        self.SONAR_TRIG = 27
        self.S_MID = 16
        self.S_AMP = 11
        self.MAX_DUTY = self.S_MID + self.S_AMP
        self.MIN_DUTY = self.S_MID - self.S_AMP
        self.ESC_MINDUTY = 10
        self.ESC_MAXDUTY = 40
        self.SERVO_PIN = [self.STEER, self.FAN_ANG]
        self.ESC_PIN = [self.ESC]
        self.SENSOR_PIN_OUT = []
        self.SENSOR_PIN_IN = []
        self.PWM_TEST_SEQ = [self.S_MID, self.S_MID + self.S_AMP, self.S_MID, self.S_MID - self.S_AMP]
        self.pi = pigpio.pi()
        self.ranger = sonar_ranger.ranger(self.pi, self.SONAR_TRIG, self.SONAR_ECHO)

    def turn_on(self):
        for i in self.SERVO_PIN + self.SENSOR_PIN_OUT:
            self.pi.set_mode(i, pigpio.OUTPUT)
            self.pi.set_PWM_frequency(i, 50)
            self.pi.set_PWM_dutycycle(i, self.S_MID)
        for i in self.ESC_PIN:
            self.pi.set_mode(i, pigpio.OUTPUT)
            self.pi.set_PWM_frequency(i, 50)
            self.pi.set_PWM_dutycycle(i, 0)
        for i in self.SENSOR_PIN_IN:
            self.pi.set_mode(i, pigpio.INPUT)
            self.pi.set_PWM_frequency(i, 50)

    def turn_off(self):
        print("exit routing start...")
        print("Halt BLDC...")
        for i in self.ESC_PIN:
            self.pi.set_PWM_dutycycle(i, 0)
        print("Halt Servos...")
        for i in self.SERVO_PIN:
            self.pi.set_PWM_dutycycle(i, self.S_MID)
        time.sleep(1)
        for i in self.SERVO_PIN:
            self.pi.set_PWM_dutycycle(i, 0)
        print("Stop sonar ranger ...")
        self.ranger.cancel()
        print("Clean Sensor pins...")
        for i in self.SENSOR_PIN_IN:
            self.pi.set_PWM_dutycycle(i, 0)
        self.pi.stop()


    def set_fan(self, theta):
        if theta < 0:
            theta = 0
        elif theta > 180:
            theta = 180
        duty = np.interp(theta, [0, 180], [self.MAX_DUTY, self.MIN_DUTY]) 
        self.pi.set_PWM_dutycycle(self.FAN_ANG, int(duty))



    def accelerate(self, speed = 1):
        if speed < 0:
            duty = self.ESC_MINDUTY
        elif speed > self.ESC_MAXDUTY - self.ESC_MINDUTY:
            duty = self.ESC_MAXDUTY
        else:
            duty = speed + self.ESC_MINDUTY
        self.pi.set_PWM_dutycycle(self.ESC, duty)

    def steer(self, theta):
        if theta < 0:
            theta = 0
        elif theta > 180:
            theta = 180
        theta = (np.interp(theta, [0, 180], [35, 145]))
        duty = np.interp(theta, [0, 180], [self.MAX_DUTY, self.MIN_DUTY])
        self.pi.set_PWM_dutycycle(self.STEER, int(duty))

    def get_distance(self):
        return self.ranger.read()

    def test_servo(self):
        for i in self.PWM_TEST_SEQ:
            for j in self.SERVO_PIN:
                self.pi.set_PWM_dutycycle(j, i)
            time.sleep(1)

    def callibrate_ESC(self):
        print("calibration high ...")
        self.pi.set_PWM_dutycycle(self.ESC, self.ESC_MAXDUTY)
        time.sleep(4)
        print("calibration low ...")
        self.pi.set_PWM_dutycycle(self.ESC, self.ESC_MINDUTY)
        time.sleep(2)
        self.pi.set_PWM_dutycycle(self.ESC, self.ESC_MINDUTY)
        print("Finish calibration !")

if __name__ == "__main__":
    import atexit
    arschloch = ARSCHLOCH()
    arschloch.turn_on()

    atexit.register(arschloch.turn_off)
    try:
        #arschloch.callibrate_ESC()
        #arschloch.accelerate(4)
        #arschloch.turn_off()
        #arschloch.set_fan(172)
        #while(1):
            #res = arschloch.get_distance()
            #print(res)
        #time.sleep(5)
        while 1:
            for i in arschloch.SERVO_PIN:
                arschloch.test_servo()
    except:
        exit()
