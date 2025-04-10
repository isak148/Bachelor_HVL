from mpu6050 import mpu6050
from time import sleep



sensor = mpu6050(0x68)

while True:
    accel_data = sensor.get_accel_data(g=True)
    gyro_data = sensor.get_gyro_data()
    temp = sensor.get_temp()

    if accel_data['z'] <=-0.15:
        print("ned")
    elif accel_data['z']>=0.15:
        print("opp")
    else:
        print("plan")
    sleep(0.2)