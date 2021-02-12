
#!/usr/bin/python
import smbus
import math
import time
from datetime import datetime

# Register
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c

def read_byte(adr):
    return bus.read_byte_data(address, adr)

def read_word(adr):
    h = bus.read_byte_data(address, adr)
    l = bus.read_byte_data(address, adr+1)
    value = (h << 8 ) + l
    return value

def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535-val)+1)
    else:
        return val

def dist(a,b):
    return math.sqrt((a*a)+(b*b))

def get_y_rotation(x,y,z):
    radians=math.atan2(x, dist(y,z))
    return -math.degrees(radians)

def get_x_rotation(x,y,z):
    radians=math.atan2(y, dist(x,z))
    return math.degrees(radians)

bus = smbus.SMBus(1) # bus = smbus.SMBus(1) for Revision 2 boards
address = 0x68 # via i2cdetect

# Now wake the 6050 up as it starts in sleep mode
bus.write_byte_data(address, power_mgmt_1,0)

while True:
    time.sleep(0.1)
    print "Gyroskop"
    print "--------"

    xout = read_word_2c(0x43)
    yout = read_word_2c(0x45)
    zout = read_word_2c(0x47)

    print "xout: ", ("%5d" % xout), "scaled: ", (xout/131)
    print "yout: ", ("%5d" % yout), "scaled: ", (yout/131)
    print "zout: ", ("%5d" % zout), "scaled: ", (zout/131)

    print
    print "accelssensor"
    print "------------"

    accel_xout = read_word_2c(0x3b)
    accel_yout = read_word_2c(0x3d)
    accel_zout = read_word_2c(0x3f)

    accel_xout_scaled = accel_xout/16384.0
    accel_yout_scaled = accel_yout/16384.0
    accel_zout_scaled = accel_zout/16384.0

    print "accel_xout: ", ("%6d" % accel_xout), "scaled: ", accel_xout_scaled
    print "accel_yout: ", ("%6d" % accel_yout), "scaled: ", accel_yout_scaled
    print "accel_zout: ", ("%6d" % accel_zout), "scaled: ", accel_zout_scaled

    print "X Rotation: " , get_x_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)
    print "Y Rotation: " , get_y_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled)
    timestamp=datetime.fromtimestamp(time.time())
    print timestamp
    time.sleep(0.5)
