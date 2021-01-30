import sensor
AHRS=sensor.serialConnect("/dev/ttyUSB0",115200)
AHRS.read()
print(AHRS.data)
gyro=sensor.Gyro()
gyro.gyro_out()
gyro.accl_out()