from ARSCHLOCH import *
import atexit
from evdev import InputDevice, categorize, ecodes

thrust = 0
brake = 0
steer = 90

if __name__ == "__main__":

    arschloch = ARSCHLOCH()
    arschloch.turn_on()
    arschloch.callibrate_ESC()
    atexit.register(arschloch.turn_off)
    gamepad = InputDevice('/dev/input/event0')
    print(gamepad)
    arschloch.accelerate(int(thrust))
    arschloch.set_fan(int(brake))
    arschloch.steer(int(steer))

    for event in gamepad.read_loop():
        if event.code == 315 and event.value == 1:
            print("Calibrate ESC")
            print(event.value)
            arschloch.callibrate_ESC()
        if event.code == 5:
            #print("THRUST")
            #print(event.value)
            thrust = event.value / 15
        elif event.code == 2:
            #print("BRAKE")
            #print(event.value)
            brake = np.interp(event.value, [0, 255], [0, 180])
        elif event.code == 0 and not event.value == 0:
            #print("STEER")
            #print(event.value)
            if event.value < 3000 and event.value > -3000:
                steer = 90
            else:
                steer = np.interp(event.value, [-32767, 32767], [180, 0])
 
        print([thrust, brake, steer])
        arschloch.accelerate(int(thrust))
        arschloch.set_fan(int(brake))
        arschloch.steer(int(steer))
        #time.sleep(0.05)

