from gpiozero import AngularServo
import sys
import time

time.sleep(5)
pin = int(sys.argv[1])
servo = AngularServo(pin, min_pulse_width=0.0006, max_pulse_width=0.0023)
servo.angle = 90
print(f"DEPLOYED (pin {pin})")
time.sleep(2)

