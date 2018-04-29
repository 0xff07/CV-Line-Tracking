import RPi.GPIO as GPIO
import time

PWM_PIN = [12]

def PWM_Obj(PWM_PIN):
    GPIO.setmode(GPIO.BOARD)
    obj = []
    for i in PWM_PIN:
        GPIO.setup(i, GPIO.OUT)
        obj.append(GPIO.PWM(i, 50))
    for pwm_out in obj:
        pwm_out.start(2)
    return obj

def deg2duty(MIN_i, MAX_i, val, MIN_f = 544, MAX_f = 2400, freq=50):
    val = float(val)
    range_i = float(MAX_i) - MIN_i
    range_f = float(MAX_f) - MIN_f
    period = 1000000./freq
    return ((val - MIN_i) / range_i * range_f + MIN_f)/period * 100

def ESC_Callibrate(ESC, DUTY_MAX=11, DUTY_MIN=3):
    print "Callibrating high ..."
    ESC.ChangeDutyCycle(DUTY_MAX)
    time.sleep(2)
    print "Callibrating low ..."
    ESC.ChangeDutyCycle(DUTY_MIN)
    time.sleep(2)
    print "Finish callibrating !"

def end_routine(PWM_objs):
    for i in PWM_objs:
        i.stop()
    GPIO.cleanup()


if __name__ == "__main__":
    try:
        pwm_obj = PWM_Obj(PWM_PIN)
        #ESC_Callibrate(pwm_obj[0])
        for i in range(2, 12):
            print str(i)
            pwm_obj[0].ChangeDutyCycle(i)
            time.sleep(2)
        end_routine(pwm_obj)
    except KeyboardInterrupt or RuntimeWarning:
        end_routine(pwm_obj)
        GPIO.cleanup()
