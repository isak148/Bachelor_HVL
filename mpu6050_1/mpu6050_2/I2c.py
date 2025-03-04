import smbus2
import time

bus = smbus2.SMBus(1)  # I2C-buss 1
address = 0x68  # MPU6050-adressen

try:
    who_am_i = bus.read_byte_data(address, 0x75)  # WHO_AM_I-registeret
    print(f"MPU6050 WHO_AM_I response: {hex(who_am_i)}")
except Exception as e:
    print(f"Feil ved I2C-lesing: {e}")

# Dette fikk sensoren til å verfall gi ut noe annet data en bare 0. 

# Reset sensor (nullstill alt)
bus.write_byte_data(address, 0x6B, 0x80)
time.sleep(0.1)

# Wake up sensofir
bus.write_byte_data(address, 0x6B, 0x00)
time.sleep(0.1)

# Les akselerometerdata
accel_x_h = bus.read_byte_data(address, 0x3B)
accel_x_l = bus.read_byte_data(address, 0x3C)
accel_x = (accel_x_h << 8) | accel_x_l

print(f"Akselerasjon X: {accel_x}")