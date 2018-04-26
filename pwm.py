import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
GPIO.setup(12, GPIO.OUT)
servo_pwm = GPIO.PWM(12, 50)
servo_pwm.start(2)

# 2~11.5

def deg2duty(MIN_i, MAX_i, val, MIN_f = 544, MAX_f = 2400, freq=50):
    val = float(val)
    range_i = float(MAX_i) - MIN_i
    range_f = float(MAX_f) - MIN_f
    period = 1000000./freq
    return ((val - MIN_i) / range_i * range_f + MIN_f)/period * 100

def ESC_Callibrate(ESC_Pin):
    print "Callibrating high ..."
    ESC_Pin.ChangeDutyCycle(12.5)
    time.sleep(5)
    print "Callibrating low ..."
    ESC_Pin.ChangeDutyCycle(2.5)
    time.sleep(5)
    print "Finish callibrating !"

def end_routine(PWM_PIN):
    PWM_PIN.stop()
    GPIO.cleanup()

if __name__ == "__main__":
    try:
        #ESC_Callibrate(servo_pwm)
        while True:
            #duty = deg2duty(-90, 90, 90);
            #servo_pwm.ChangeDutyCycle(duty)
            #time.sleep(10)
            servo_pwm.ChangeDutyCycle(6.5)
            time.sleep(1)
            servo_pwm.ChangeDutyCycle(11)
            time.sleep(1)
            servo_pwm.ChangeDutyCycle(6.5)
            time.sleep(1)
            servo_pwm.ChangeDutyCycle(2)
            time.sleep(1)




    except KeyboardInterrupt:
        servo_pwm.ChangeDutyCycle(6.5)
        servo_pwm.stop()
        GPIO.cleanup()
