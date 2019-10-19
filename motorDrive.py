#Dit is een git test

import RPi.GPIO as GPIO
import time

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

#Pinning
SDI   = 17
SRCLK = 27
RCLK  = 22

def setup():
    GPIO.setup(SDI, GPIO.OUT)
    GPIO.setup(RCLK, GPIO.OUT)
    GPIO.setup(SRCLK, GPIO.OUT)
    GPIO.output(SDI, GPIO.LOW)
    GPIO.output(RCLK, GPIO.LOW)
    GPIO.output(SRCLK, GPIO.LOW)

#Clock input data into register
def hc595_in(data):
    for bit in range(0, 8): 
        GPIO.output(SDI, 0x80 & (data << bit))
        GPIO.output(SRCLK, GPIO.HIGH)
        time.sleep(0.001)
        GPIO.output(SRCLK, GPIO.LOW)
        
#Clock register data to output
def hc595_out():
    GPIO.output(RCLK, GPIO.HIGH)
    time.sleep(0.001)
    GPIO.output(RCLK, GPIO.LOW)

def destroy():
    hc595_in(all_off)
    hc595_out()
    GPIO.cleanup()
    
def main():
    setup()
    
    #All
    all_off = 0b00000000
    #Motor 1
    m1_left = 0b00000100
    m1_right = 0b00001000
    #Motor 2
    m2_left = 0b00000010
    m2_right = 0b00010000
    #Motor 3
    m3_left = 0b00100000
    m3_right = 0b10000000
    #Motor 4
    m4_left = 0b00000001
    m4_right = 0b01000000   
    
    
    while 1:   
        hc595_in(m1_left)
        hc595_out()

if __name__ == '__main__': # Program starting from here
    try:
        main()
    finally:
        destroy() 
#End