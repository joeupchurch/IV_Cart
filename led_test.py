# Simple demo of of the PCA9685 PWM servo/LED controller library.
# This will move channel 0 from min to max position repeatedly.
# Author: Tony DiCola
# License: Public Domain
from __future__ import division
import time
import atexit

# Import the PCA9685 module.
import Adafruit_PCA9685

# Initialise the PCA9685 using the default address (0x40).
pwm = Adafruit_PCA9685.PCA9685()

# Set max pulse
max_pulse = int(4096*0.5)
min_pulse = int(4096*0.2)

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(100)

def exit_handler():
    pwm.set_pwm(12,0,0)
    pwm.set_pwm(13,0,0)
    pwm.set_pwm(14,0,0)
    pwm.set_pwm(15,0,0)

print('Moving servo on channel 0, press Ctrl-C to quit...')
while True:
    try:

        pwm.set_pwm(13, 0, 0)
        pwm.set_pwm(15, 0, 0)

        time.sleep(0.5)
        for pulse_up in range(0,max_pulse,500):
            print pulse_up
            # Move servo on channel O between extremes.
            pwm.set_pwm(12, 0, pulse_up)
            pwm.set_pwm(15, 0, pulse_up)
            time.sleep(0.01)
        pwm.set_pwm(12, 0, min_pulse)
        pwm.set_pwm(15, 0, min_pulse)
        time.sleep(3)
        pwm.set_pwm(12, 0, 0)
        pwm.set_pwm(15, 0, 0)
        break
        for pulse_down in range(max_pulse,min_pulse,-500):
            print pulse_down
            # Move servo on channel O between extremes.
            pwm.set_pwm(12, 0, pulse_down)
            pwm.set_pwm(15, 0, pulse_down)
            time.sleep(0.01)

        time.sleep(2)
        
        break


        time.sleep(0.5)
        for pulse_up in range(0,max_pulse,500):
            print pulse_up
            # Move servo on channel O between extremes.
            pwm.set_pwm(13, 0, pulse_up)
            pwm.set_pwm(14, 0, pulse_up)
            time.sleep(0.01)
            
        for pulse_down in range(max_pulse,min_pulse,-500):
            print pulse_down
            # Move servo on channel O between extremes.
            pwm.set_pwm(13, 0, pulse_down)
            pwm.set_pwm(14, 0, pulse_down)
            time.sleep(0.01)

        time.sleep(10)

    except KeyboardInterrupt:
        break
        
atexit.register(exit_handler)
