# Import necessary packages
import RPi.GPIO as gpio
import time
import subprocess, os
import signal
import Adafruit_PCA9685

gpio.setmode(gpio.BCM)
gpio.setwarnings(False)
gpio_switch = 17
gpio.setup(gpio_switch,gpio.IN,pull_up_down=gpio.PUD_UP)

# Set I2C locations for motors
left_forward = 12
left_backward = 13
right_forward = 14
right_backward = 15

try:
    run = 0
    while True:
        # Switch pressed, program not running
        if gpio.input(gpio_switch)==0 and run ==0:

            # Start tracking.py
            print 'Tracking Script Started'
            rpistr = "python /home/pi/Code/tracking.py"
            p = subprocess.Popen(rpistr,shell=True, preexec_fn=os.setsid)
            run=1

            time.sleep(0.25)
            # Wait for button to quit being pressed
            while gpio.input(gpio_switch)==0:
                time.sleep(0.1)

        # Switch pressed program running
        if gpio.input(gpio_switch)==1 and run ==1:

            # End tracking.py
            print 'Tracking Script Ended (User)'
            run = 0
            time.sleep(0.25)
            os.killpg(p.pid, signal.SIGTERM)
            
            # Initialize the PCA9685 using default address
            pwm = Adafruit_PCA9685.PCA9685()

            # Set pwm frequency (Hz)
            pwm.set_pwm_freq(100)

            # Set PWM for all channels to 0
            pwm.set_pwm(left_forward,0,0)
            pwm.set_pwm(right_forward,0,0)
            pwm.set_pwm(left_backward,0,0)
            pwm.set_pwm(right_backward,0,0)

            # Wait for button to quit being pressed
            while gpio.input(gpio_switch)==0:
                time.sleep(0.1)

# If ctrl+C is pressed
except KeyboardInterrupt:
    # End tracking.py
    os.killpg(p.pid, signal.SIGTERM)
    print 'Tracking Script Ended (Keyboard)'

    # Initialize the PCA9685 using default address
    pwm = Adafruit_PCA9685.PCA9685()

    # Set pwm frequency (Hz)
    pwm.set_pwm_freq(100)

    # Set PWM for all channels to 0
    pwm.set_pwm(left_forward,0,0)
    pwm.set_pwm(right_forward,0,0)
    pwm.set_pwm(left_backward,0,0)
    pwm.set_pwm(right_backward,0,0)

    # Clean up GPIO Pins
    gpio.cleanup()
