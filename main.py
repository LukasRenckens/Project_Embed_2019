import RPi.GPIO as GPIO
import time
import smbus
import math
import tkinter as tk
from mpu6050 import mpu6050
from pidcontroller import PIDController

##------------------------------------------------------- Tkinter ----------------------------------------------------------------

root = tk.Tk()
frame = tk.Frame(root)
frame.grid()
root.title("Balancing robot")

def stop():
    global start
    start = False
    hc595_in(0b00000000)
    hc595_out()
    print('end')

def start():
    global start
    start = True
    main()
    
labelP = tk.Label(frame, text="P")
labelP.grid(row=0, column=0, padx=10, pady=10)

labelI = tk.Label(frame, text="I")
labelI.grid(row=1, column=0, padx=10, pady=10)

labelI = tk.Label(frame, text="D")
labelI.grid(row=2, column=0, padx=10, pady=10)

slider1 = tk.Scale(frame, from_=0.0, to=1000.0, orient=tk.HORIZONTAL, length=800, resolution=0.01)
slider1.grid(row=0, column=1, padx=10, pady=10)
slider1.set(60)

slider2 = tk.Scale(frame, from_=0.0, to=1000.0, orient=tk.HORIZONTAL, length=800, resolution=0.01)
slider2.grid(row=1, column=1, padx=10, pady=10)
slider2.set(1)

slider3 = tk.Scale(frame, from_=0.0, to=1000.0, orient=tk.HORIZONTAL, length=800, resolution=0.01)
slider3.grid(row=2, column=1, padx=10, pady=10)
slider3.set(11)

start_button = tk.Button(frame, text="Start", command=start)
start_button.grid(row=3, column=0, columnspan=2, pady=10, padx=10)
start_button.config(background="lightgreen")

stop_button = tk.Button(frame, text="Stop", command=stop)
stop_button.grid(row=4, column=0, columnspan=2, pady=10, padx=10)
stop_button.config(background="red")

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
#     velocity = 100
    hc595_in(m1_left | m2_left)
    hc595_out()
    PWM1.ChangeDutyCycle(velocity)
    PWM2.ChangeDutyCycle(velocity)
#Alike the backward funtion this forward function does the same thing but moves both the motors forward.
def forward(velocity):
#     velocity = 100
    hc595_in(m1_right | m2_right)
    hc595_out()
    PWM1.ChangeDutyCycle(velocity)
    PWM2.ChangeDutyCycle(velocity)
#If the PID value is 0 (the Robot is 'balanced') it uses this equilibrium function.
def equilibrium():
    hc595_in(all_off)
    hc595_out()

##---------------------------------------------------- Gyro/Accel----------------------------------------------------------------
#Gyro math
def distance(a, b):
    return math.sqrt((a*a) + (b*b))

def y_rotation(x, y, z):
    radians = math.atan2(x, distance(y, z))
    return -math.degrees(radians)

def x_rotation(x, y, z):
    radians = math.atan2(y, distance(x, z))
    return math.degrees(radians)

##----------------------------------------------------- Main ---------------------------------------------------------------------    
def main():
    
    sensor = mpu6050(0x68)
    #K and K1 --> Constants used with the complementary filter
    K = 0.98
    K1 = 1 - K

    time_diff = 0.02
    ITerm = 0
    
    #Calling the MPU6050 data 
    accel_data = sensor.get_accel_data()
    gyro_data = sensor.get_gyro_data()

    aTempX = accel_data['x']
    aTempY = accel_data['y']
    aTempZ = accel_data['z']

    gTempX = gyro_data['x']
    gTempY = gyro_data['y']
    gTempZ = gyro_data['z']
    
    last_x = x_rotation(aTempX, aTempY, aTempZ)
    last_y = y_rotation(aTempX, aTempY, aTempZ)

    gyro_offset_x = gTempX
    gyro_offset_y = gTempY

    gyro_total_x = (last_x) - gyro_offset_x
    gyro_total_y = (last_y) - gyro_offset_y
    
    while start:
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
        last_y = K * (last_y + gyro_y_delta) + (K1 * rotation_y)

        #setting the PID values. Here you can change the P, I and D values according to your needs
        PID = PIDController(P=slider1.get(), I=slider2.get(), D=slider3.get())
        PIDx = PID.step(last_y)

        #if the PIDx data is lower than 0.0 than move appropriately backward
        if PIDx < 0.0:
            if (PIDx < -100):
                PIDx = -100
            backward(-float(PIDx))
            #StepperFor(-PIDx)
        #if the PIDx data is higher than 0.0 than move appropriately forward
        elif PIDx > 0.0:
            if (PIDx > 100):
                PIDx = 100
            forward(float(PIDx))
            #StepperBACK(PIDx)
        #if none of the above statements is fulfilled than do not move at all 
        else:
            equilibrium()


        #update GUI
        root.update()

##-------------------------------------------------------- End -------------------------------------------------------------------
if __name__ == '__main__': # Program starting from here
    try:
        root.mainloop()
    except KeyboardInterrupt: 
        hc595_in(0b00000000)
        hc595_out()
        GPIO.cleanup()
        print('end')
#End
