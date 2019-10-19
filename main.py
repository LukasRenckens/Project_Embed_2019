import RPi.GPIO as GPIO
import time
import smbus
import math

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

#Pinning
SDI   = 17
SRCLK = 27
RCLK  = 22

#Gyro setup
 # Power management registers
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c

bus = smbus.SMBus(1) # or bus = smbus.SMBus(1) for Revision 2 boards
address = 0x68       # This is the address value read via the i2cdetect command

# Now wake the 6050 up as it starts in sleep mode
bus.write_byte_data(address, power_mgmt_1, 0)

def gpio_setup():
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
    hc595_in(0b00000000)
    hc595_out()
    GPIO.cleanup()

def read_byte(adr):
    return bus.read_byte_data(address, adr)

def read_word(adr):
    high = bus.read_byte_data(address, adr)
    low = bus.read_byte_data(address, adr+1)
    val = (high << 8) + low
    return val

def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val

def dist(a,b):
    return math.sqrt((a*a)+(b*b))

def get_y_rotation(x,y,z):
    radians = math.atan2(x, dist(y,z))
    return -math.degrees(radians)

def get_x_rotation(x,y,z):
    radians = math.atan2(y, dist(x,z))
    return math.degrees(radians)


    
def main():
    gpio_setup()
    
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
        accel_xout = read_word_2c(0x3b)
        accel_yout = read_word_2c(0x3d)
        accel_zout = read_word_2c(0x3f)

        accel_xout_scaled = accel_xout / 16384.0
        accel_yout_scaled = accel_yout / 16384.0
        accel_zout_scaled = accel_zout / 16384.0
        
        if(get_y_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled) >= 10):
            hc595_in(m1_left)
            hc595_out()
        elif(get_y_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled) <= -10):
            hc595_in(m1_right)
            hc595_out()
        else:
            hc595_in(all_off)
            hc595_out()
        

        #print("x rotation: " , get_x_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled))
        #print("y rotation: " , get_y_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled))


if __name__ == '__main__': # Program starting from here
    try:
        main()
    except KeyboardInterrupt: 
        destroy()
        print('end')
#End
