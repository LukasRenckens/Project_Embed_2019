import tkinter as tk
import smbus
import math
import time

root = tk.Tk()
frame = tk.Frame(root)
frame.grid()
# root.geometry("500x500")
root.title("WHEEL-E")

# Power management registers
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c

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

bus = smbus.SMBus(1) # or bus = smbus.SMBus(1) for Revision 2 boards
address = 0x68       # This is the address value read via the i2cdetect command

# Now wake the 6050 up as it starts in sleep mode
bus.write_byte_data(address, power_mgmt_1, 0)

# print("gyro data")
# print("---------")

gyro_xout = read_word_2c(0x43)
gyro_yout = read_word_2c(0x45)
gyro_zout = read_word_2c(0x47)

# print("gyro_xout: ", gyro_xout, " scaled: ", (gyro_xout / 131))
# print("gyro_yout: ", gyro_yout, " scaled: ", (gyro_yout / 131))
# print("gyro_zout: ", gyro_zout, " scaled: ", (gyro_zout / 131))

# print()
# print("accelerometer data")
# print("------------------")

accel_xout = read_word_2c(0x3b)
accel_yout = read_word_2c(0x3d)
accel_zout = read_word_2c(0x3f)

accel_xout_scaled = accel_xout / 16384.0
accel_yout_scaled = accel_yout / 16384.0
accel_zout_scaled = accel_zout / 16384.0

# print("accel_xout: ", accel_xout, " scaled: ", accel_xout_scaled)
# print("accel_yout: ", accel_yout, " scaled: ", accel_yout_scaled)
# print("accel_zout: ", accel_zout, " scaled: ", accel_zout_scaled)

accel_xout = read_word_2c(0x3b)
accel_yout = read_word_2c(0x3d)
accel_zout = read_word_2c(0x3f)

accel_xout_scaled = accel_xout / 16384.0
accel_yout_scaled = accel_yout / 16384.0
accel_zout_scaled = accel_zout / 16384.0

label_y_rotation = tk.Label(frame, text="Y-rotation:")
label_y_rotation.grid(row=1, column=0, padx=10, pady=10)

label_y_rotation_value = tk.Label(frame, text=get_y_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled))
label_y_rotation_value.grid(row=1, column=1, padx=10, pady=10)

label_x_rotation = tk.Label(frame, text="X-rotation:")
label_x_rotation.grid(row=0, column=0, padx=10, pady=10)

label_x_rotation_value = tk.Label(frame, text=get_x_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled))
label_x_rotation_value.grid(row=0, column=1, padx=10, pady=10)

# port = tk.OptionMenu(frame, port_variable, *PORTS)
# port.grid(row=0, column=1, padx=10)

# start_button = tk.Button(frame, text="Start", command=start_measurements)
# start_button.grid(row=3, column=0, pady=10, padx=10)
# start_button.config(background="lightgreen")

# stop_button = tk.Button(frame, text="Stop", command=stop_measurements)
# stop_button.grid(row=3, column=1, pady=10, padx=10)
# stop_button.config(background="red")

# calibrate_button = tk.Button(frame, text="Calibrate", command=calibrate)
# calibrate_button.grid(row=2, column=0, columnspan=2, pady=10)
# calibrate_button.config(background="orange")

# text_box = tk.Text(frame, width = 50, height = 10)
# text_box.grid(row=3, column=0, columnspan=2, pady=10, padx=10)

# graphic_box = tk.Text(frame, width = 50, height = 18)
# graphic_box.grid(row=0, column=2, rowspan=4, pady=10, padx=10)
# graphic_box.config(background="lightgray")

root.mainloop()