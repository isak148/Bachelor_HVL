# -*- coding: utf-8 -*-
# Kode hentet og tilpasset fra:
# https://github.com/Hykudoru/MPU6050-Gyro-Motion-Tracking/blob/main/MPU6050-Gyro-Motion-Tracking/MPU6050.py
# Original forfatter: Hykudoru

import smbus2 as smbus # Bruker smbus2 for bedre kompatibilitet
import math
import time

# MPU6050 Register Map
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47


class mpu6050:

    def __init__(self, address=0x68, bus=1):
        self.address = address
        self.bus = smbus.SMBus(bus)
        # Wake up the MPU-6050 since it starts in sleep mode
        self.bus.write_byte_data(self.address, PWR_MGMT_1, 0x00)
        time.sleep(0.1) # Wait for sensor to stabilize
        print(f"MPU6050 initialisert på adresse {hex(address)} (Hykudoru driver)")
         # Optionally set sensitivity (example from repo not included here)
        # self.set_gyro_sensitivity(1) # Example: 0: +/-250, 1: +/-500 etc.
        # self.set_accel_sensitivity(0) # Example: 0: +/-2g, 1: +/-4g etc.

    # Read a word (two bytes) from the MPU-6050
    def read_word(self, adr):
        high = self.bus.read_byte_data(self.address, adr)
        low = self.bus.read_byte_data(self.address, adr+1)
        val = (high << 8) + low
        return val

    # Read a word (two bytes) from the MPU-6050 in 2's complement format
    def read_word_2c(self, adr):
        val = self.read_word(adr)
        if (val >= 0x8000):
            return -((65535 - val) + 1)
        else:
            return val

    # Get accelerometer data
    def get_accel_data(self, g=False):
        x = self.read_word_2c(ACCEL_XOUT_H)
        y = self.read_word_2c(ACCEL_YOUT_H)
        z = self.read_word_2c(ACCEL_ZOUT_H)

        # Default sensitivity is +/- 2g
        # Raw value range -32768 to 32767 corresponds to -2g to +2g
        # So scale factor is 32768 / 2 = 16384.0 LSB/g
        # (Adjust if sensitivity is changed)
        scale_factor = 16384.0
        if g:
            x = x / scale_factor
            y = y / scale_factor
            z = z / scale_factor

        return {'x': x, 'y': y, 'z': z}

    # Get gyroscope data
    def get_gyro_data(self):
        x = self.read_word_2c(GYRO_XOUT_H)
        y = self.read_word_2c(GYRO_YOUT_H)
        z = self.read_word_2c(GYRO_ZOUT_H)

        # Default sensitivity is +/- 250 degrees/s
        # Raw value range -32768 to 32767 corresponds to -250 to +250 deg/s
        # So scale factor is 32768 / 250 = 131.072 LSB/(deg/s)
        # (Adjust if sensitivity is changed)
        scale_factor = 131.0
        x = x / scale_factor
        y = y / scale_factor
        z = z / scale_factor

        return {'x': x, 'y': y, 'z': z}

    # --- Metoder for vinkel fra akselerometer (fra GitHub repo) ---
    # --- ADVARSEL: Disse formlene er ustabile nær 90 grader tilt ---
    @staticmethod
    def get_accel_pitch(accel_data):
        """
        Calculates pitch based on atan2(y, z).
        NOTE: This is typically used for ROLL in standard conventions.
        Returns angle in degrees.
        Unstable when z is near 0 (tilted ~90 deg roll or pitch).
        """
        return math.atan2(accel_data['y'], accel_data['z']) * 180 / math.pi

    @staticmethod
    def get_accel_roll(accel_data):
        """
        Calculates roll based on atan2(x, z).
        NOTE: This is NOT the standard pitch calculation.
        Returns angle in degrees.
        Unstable when z is near 0 (tilted ~90 deg roll or pitch).
        """
        # Original repo used atan2(x,z), let's keep that for consistency
        return math.atan2(accel_data['x'], accel_data['z']) * 180 / math.pi
        # A more standard roll (if get_accel_pitch was actually roll):
        # return math.atan2(-accel_data['x'], math.sqrt(accel_data['y']**2 + accel_data['z']**2)) * 180 / math.pi


# Eksempel på bruk (hvis filen kjøres direkte)
if __name__ == '__main__':
    try:
        sensor = mpu6050(0x68)
        print("Tester MPU6050 lesing (Hykudoru driver):")
        for _ in range(10):
            accel = sensor.get_accel_data(g=True)
            gyro = sensor.get_gyro_data()
            # Beregn vinkler KUN fra akselerometer for test
            accel_pitch_calc = sensor.get_accel_pitch(accel) # Egentlig roll
            accel_roll_calc = sensor.get_accel_roll(accel)   # Ikke standard pitch

            print(f"Accel(g): x={accel['x']:.2f}, y={accel['y']:.2f}, z={accel['z']:.2f} | "
                  f"Gyro(dps): x={gyro['x']:.2f}, y={gyro['y']:.2f}, z={gyro['z']:.2f} | "
                  f"AccelPitch(y,z): {accel_pitch_calc:.1f} | AccelRoll(x,z): {accel_roll_calc:.1f}")
            time.sleep(0.2)
    except Exception as e:
        print(f"Feil under testing av MPU6050 driver: {e}")