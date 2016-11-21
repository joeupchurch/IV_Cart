import RPi.GPIO as gpio
import time
gpio.setmode(gpio.BCM)
##GPIO.setwarnings(False)
gpio.setup(23, gpio.OUT)
gpio.setup(24, gpio.OUT)

ontime = 0.010
offtime = 0.005
counts = 200
counter = 0

while counter < counts:
    gpio.output(23,True)
    time.sleep(ontime)
    gpio.output(23,False)
    time.sleep(offtime)
    counter += 1

counter = 0

while counter < counts:
    gpio.output(24,True)
    time.sleep(ontime)
    gpio.output(24,False)
    time.sleep(offtime)
    counter += 1
gpio.cleanup()
    
##p = GPIO.PWM(18, 100)  # channel=12 frequency=50Hz
##p.start(.5)
##print 'running'
##time.sleep(10)
##
##try:
##    while 1:
##        print 'start'
##        for dc in range(0, 101, 5):
##            p.ChangeDutyCycle(dc)
##            time.sleep(0.1)
####        for dc in range(100, -1, -5):
####            p.ChangeDutyCycle(dc)
####            time.sleep(0.1)
##except KeyboardInterrupt:
##    pass
##p.stop()
##GPIO.cleanup()
