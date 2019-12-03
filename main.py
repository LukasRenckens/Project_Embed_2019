import RPi.GPIO as GPIO
import time
import smbus
import math
from pidcontroller import PIDController

##------------------------------------------------------- GPIO -------------------------------------------------------------------
GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)

SDI   = 17
SRCLK = 27
RCLK  = 22

GPIO.setup(SDI, GPIO.OUT)
GPIO.setup(RCLK, GPIO.OUT)
GPIO.setup(SRCLK, GPIO.OUT)
GPIO.setup(21, GPIO.OUT)
GPIO.setup(20, GPIO.OUT)
GPIO.output(SDI, GPIO.LOW)
GPIO.output(RCLK, GPIO.LOW)
GPIO.output(SRCLK, GPIO.LOW)

#Pulse width modulation: the speed changes accordingly the inclination angle
PWM1 = GPIO.PWM(21, 100)
PWM2 = GPIO.PWM(20, 100)
PWM1.start(0)
PWM2.start(0)    
    
##------------------------------------------------------ Shiftreg -------------------------------------------------------------------    
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
##-------------------------------------------------- Motors ---------------------------------------------------------------------
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
#This backward function takes a velocity argument that is the PID value. Both motors drives backward
def backward(velocity):
    hc595_in(m1_right | m2_right)
    hc595_out()
    PWM1.ChangeDutyCycle(velocity)
    PWM2.ChangeDutyCycle(velocity)
#Alike the backward funtion this forward function does the same thing but moves both the motors forward.
def forward(velocity):
    hc595_in(m1_left | m2_left)
    hc595_out()
    PWM1.ChangeDutyCycle(velocity)
    PWM2.ChangeDutyCycle(velocity)
#If the PID value is 0 (the Robot is 'balanced') it uses this equilibrium function.
def equilibrium():
    hc595_in(all_off)
    hc595_out()

##---------------------------------------------------- Gyro/Accel----------------------------------------------------------------
#Gyro setup
 # Power management registers
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c

bus = smbus.SMBus(1) # or bus = smbus.SMBus(1) for Revision 2 boards
address = 0x68       # This is the address value read via the i2cdetect command

# Now wake the 6050 up as it starts in sleep mode
#bus.write_byte_data(address, power_mgmt_1, 0)

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

##----------------------------------------------------- Main ---------------------------------------------------------------------    
def main():    
    while 1:
        accel_data = sensor.get_accel_data()
        gyro_data = sensor.get_gyro_data()

        accelX = accel_data['x']
        accelY = accel_data['y']
        accelZ = accel_data['z']

        gyroX = gyro_data['x']
        gyroY = gyro_data['y']
        gyroZ = gyro_data['z']

        gyroX -= gyro_offset_x
        gyroY -= gyro_offset_y

        gyro_x_delta = (gyroX * time_diff)
        gyro_y_delta = (gyroY * time_diff)

        gyro_total_x += gyro_x_delta
        gyro_total_y += gyro_y_delta

        rotation_x = x_rotation(accelX, accelY, accelZ)
        rotation_y = y_rotation(accelX, accelY, accelZ)
        
        #Complementary Filter
        last_x = K * (last_x + gyro_x_delta) + (K1 * rotation_x)

        #setting the PID values. Here you can change the P, I and D values according to yiur needs
        PID = PIDController(P=-78.5, I=1.0, D=1.0)
        PIDx = PID.step(last_x)

        #if the PIDx data is lower than 0.0 than move appropriately backward
        if PIDx < 0.0:
            backward(-float(PIDx))
            #StepperFor(-PIDx)
        #if the PIDx data is higher than 0.0 than move appropriately forward
        elif PIDx > 0.0:
            forward(float(PIDx))
            #StepperBACK(PIDx)
        #if none of the above statements is fulfilled than do not move at all 
        else:
            equilibrium()


        print(int(last_x), 'PID: ', int(PIDx))
        sleep(0.02)
        
#         accel_xout = read_word_2c(0x3b)
#         accel_yout = read_word_2c(0x3d)
#         accel_zout = read_word_2c(0x3f)
# 
#         accel_xout_scaled = accel_xout / 16384.0
#         accel_yout_scaled = accel_yout / 16384.0
#         accel_zout_scaled = accel_zout / 16384.0
#         if(get_y_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled) <= 5):
#             hc595_in(m1_left | m2_left)
#             hc595_out()
#         elif(get_y_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled) >= -5):
#             hc595_in(m1_right | m2_right)
#             hc595_out()
#         else:
#             hc595_in(all_off)
#             hc595_out()
# 
#         print("x rotation: " , get_x_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled))
#         print("y rotation: " , get_y_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled))

##-------------------------------------------------------- End -------------------------------------------------------------------
if __name__ == '__main__': # Program starting from here
    try:
        main()
    except KeyboardInterrupt: 
        hc595_in(0b00000000)
        hc595_out()
        GPIO.cleanup()
        print('end')
#End
