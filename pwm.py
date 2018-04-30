import RPi.GPIO as GPIO
import time


def PWM_init(PWM_PIN):
    GPIO.setmode(GPIO.BOARD)
    for i in PWM_PIN.keys():
        GPIO.setup(PWM_PIN[i], GPIO.OUT)
    pwm_table = {key:GPIO.PWM(PWM_PIN[i], 50) for key in PWM_PIN}
    for i in pwm_table:
        pwm_table[i].start(2.7)
    return pwm_table

def ESC_Callibrate(ESC, DUTY_MAX=12, DUTY_MIN=2.7):
    print "Callibrating high ..."
    ESC.ChangeDutyCycle(DUTY_MAX)
    time.sleep(2)
    print "Callibrating low ..."
    ESC.ChangeDutyCycle(DUTY_MIN)
    time.sleep(2)
    print "Finish callibrating !"

def end_routine(PWM_table):
    for key in PWM_table:
        PWM_table[key].stop()
    GPIO.cleanup()


if __name__ == "__main__":
    pwm_table = PWM_init({"ESC":12})
    ESC_Callibrate(pwm_table["ESC"])
    for i in range(2, 12):
        print str(i)
        pwm_table["ESC"].ChangeDutyCycle(i)
        time.sleep(2)
    end_routine(pwm_table)
