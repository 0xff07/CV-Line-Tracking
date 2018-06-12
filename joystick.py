from ARSCHLOCH import *
import atexit
from evdev import InputDevice, categorize, ecodes

if __name__ == "__main__":

    arschloch = ARSCHLOCH()
    arschloch.turn_on()
    atexit.register(arschloch.turn_off)
    gamepad = InputDevice('/dev/input/event12')
    print(gamepad)

    for event in gamepad.read_loop():
        if event.code == 5:
            print("THRUST")
            print(event.value)
            arschloch.accelerate(event.value % 22)
        elif event.code == 2:
            print("BRAKE")
            print(event.value)
            angle = np.interp(event.value, [0, 255], [170, 10])
            arschloch.set_fan(angle)
        elif event.code == 0:
            print("STEER")
            print(event.value)
            angle = np.interp(event.value, [-32767, 32767], [180, 0])
            arschloch.steer(angle)
 

