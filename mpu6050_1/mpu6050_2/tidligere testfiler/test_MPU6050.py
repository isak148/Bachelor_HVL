import mpu6050
import time

mpu = mpu6050.mpu6050(0x68)

while True:
    print(f"Temp: {mpu.get_temp()}")
    print()

    accel_data = mpu.get_accel_data()
    print(f"Accel X: {accel_data['x']}")
    print(f"Accel Y: {accel_data['y']}")
    print(f"Accel Z: {accel_data['z']}")
    print()

    gyro_data = mpu.get_gyro_data()
    print(f"Gyro X: {gyro_data['x']}")
    print(f"Gyro Y: {gyro_data['y']}")
    print(f"Gyro Z: {gyro_data['z']}")
    print()
    print("---------------------------------")
    time.sleep(1)   
