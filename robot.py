#!/usr/bin/python

import smbus
import math
import time
import RPi.GPIO as GPIO

#use board pin numering
GPIO.setmode(GPIO.BOARD)

#pin definitions
in1 = 38
in2 = 40
in3 = 33
in4 = 35
enA = 36
enB = 37
halA = 11

#complementary filter constant
AA = 0.98
DT = 0.02

#PID gains
KP = 50
KI = 0
KD = 0

#pin setups
GPIO.setup(enA, GPIO.OUT) 
GPIO.setup(enB, GPIO.OUT) 
GPIO.setup(in1, GPIO.OUT) 
GPIO.setup(in2, GPIO.OUT) 
GPIO.setup(in3, GPIO.OUT) 
GPIO.setup(in4, GPIO.OUT)
GPIO.setup(halA, GPIO.IN) #may have to use pull_up_down=GPIO.PUD_DOWN or PUD_UP

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


bus = smbus.SMBus(1) # or bus = smbus.SMBus(0) for Revision 1 boards
address = 0x68       # This is the address value read via the i2cdetect command

# Now wake the 6050 up as it starts in sleep mode
bus.write_byte_data(address, power_mgmt_1, 0)

Iterm = 0
lastAngle = 0
Cangle = 0

try:
    # Loop until users quits with CTRL-C
    while True :
      #time.sleep(0.02)
      #gyro raw data
      gyro_xout = read_word_2c(0x43)
      gyro_yout = read_word_2c(0x45)
      gyro_zout = read_word_2c(0x47)
        
      #gyro rate of rotation /s
      #gyro_xout / 131
      #gyro_yout / 131
      #gyro_zout / 131
        
      #acc raw data
      accel_xout = read_word_2c(0x3b)
      accel_yout = read_word_2c(0x3d)
      accel_zout = read_word_2c(0x3f)
        
      #something something
      accel_xout_scaled = accel_xout / 16384.0
      accel_yout_scaled = accel_yout / 16384.0
      accel_zout_scaled = accel_zout / 16384.0
        
      #acc angle
      acc_x = get_x_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)
      acc_y = get_y_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)
        
      #current angle
      Cangle=AA*(Cangle+(gyro_xout/131)*DT) + (1-AA) * acc_x
        
      #PID
      Pterm = KP * Cangle
      Iterm = Iterm + KI * Cangle
      Dterm = KD * (Cangle - lastAngle)
      lastAngle = Cangle
      output = Pterm + Iterm + Dterm
      #print(output)
      if output > 2: #not using PWM at the moment, since it was not working
            GPIO.output(in1, GPIO.HIGH)
            GPIO.output(in2, GPIO.LOW)
            GPIO.output(in3, GPIO.HIGH)
            GPIO.output(in4, GPIO.LOW)
      elif output < -2:
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.HIGH)
            GPIO.output(in3, GPIO.LOW)
            GPIO.output(in4, GPIO.HIGH)
      else:
            GPIO.output(in1, GPIO.LOW)
            GPIO.output(in2, GPIO.LOW)
            GPIO.output(in3, GPIO.LOW)
            GPIO.output(in4, GPIO.LOW)

      GPIO.output(enA, GPIO.HIGH)
      GPIO.output(enB, GPIO.HIGH)
            
except KeyboardInterrupt:
    # Reset GPIO settings
    GPIO.output(enA, GPIO.LOW)
    GPIO.output(enB, GPIO.LOW)
    GPIO.cleanup()
