# -*- coding: utf-8 -*-
# Kode opprinnelig fra:
# https://github.com/Hykudoru/MPU6050-Gyro-Motion-Tracking/blob/main/MPU6050-Gyro-Motion-Tracking/MPU6050.py
# Original forfatter: Hykudoru
# MODIFISERT for å inkludere kalibrering

import smbus2 as smbus
import math
import time
from time import sleep
import numpy as np # Nødvendig for forbedret kalibrering

# MPU6050 Register Map
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
ACCEL_CONFIG = 0x1C # Lagt til for å kunne sette range
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

class mpu6050:

    # Legg til konstanter for range (fra forrige MPU6050 klasse)
    ACCEL_RANGE_2G = 0x00
    ACCEL_RANGE_4G = 0x08
    ACCEL_RANGE_8G = 0x10
    ACCEL_RANGE_16G = 0x18

    GYRO_RANGE_250DEG = 0x00
    GYRO_RANGE_500DEG = 0x08
    GYRO_RANGE_1000DEG = 0x10
    GYRO_RANGE_2000DEG = 0x18

    # Skaleringsfaktorer
    accel_scale_factors = {
        ACCEL_RANGE_2G: 16384.0,
        ACCEL_RANGE_4G: 8192.0,
        ACCEL_RANGE_8G: 4096.0,
        ACCEL_RANGE_16G: 2048.0
    }
    gyro_scale_factors = {
        GYRO_RANGE_250DEG: 131.0,
        GYRO_RANGE_500DEG: 65.5,
        GYRO_RANGE_1000DEG: 32.8,
        GYRO_RANGE_2000DEG: 16.4
    }

    def __init__(self, address=0x68, bus=1):
        self.address = address
        self.bus = smbus.SMBus(bus)

        # Wake up MPU-6050
        self.bus.write_byte_data(self.address, PWR_MGMT_1, 0x00)
        time.sleep(0.1) # Viktig pause

        # Sett standard ranges (kan endres med set_*_range)
        self.accel_range = self.ACCEL_RANGE_2G
        self.gyro_range = self.GYRO_RANGE_250DEG
        self.accel_scale = self.accel_scale_factors[self.accel_range]
        self.gyro_scale = self.gyro_scale_factors[self.gyro_range]
        self.set_accel_range(self.accel_range) # Skriv til register
        self.set_gyro_range(self.gyro_range)   # Skriv til register
        print(f"MPU6050 (Hykudoru driver) initialisert på {hex(address)}.")
        print(f"  Standard Accel Range: +/- {int(32768 / self.accel_scale)}g")
        print(f"  Standard Gyro Range: +/- {int(32768 / self.gyro_scale)} dps")


        # --- KJØR KALIBRERING ---
        self.gyro_offset = self.calibrate_gyro(samples=150)
        self.accel_offset = self.calibrate_accel(samples=150)
        # ------------------------

    def read_word(self, adr):
        high = self.bus.read_byte_data(self.address, adr)
        low = self.bus.read_byte_data(self.address, adr+1)
        val = (high << 8) + low
        return val

    def read_word_2c(self, adr):
        val = self.read_word(adr)
        if (val >= 0x8000):
            return -((65535 - val) + 1)
        else:
            return val

    def set_gyro_range(self, gyro_range):
        """Sets the gyro range (+/- 250, 500, 1000, or 2000 degrees/second)"""
        if gyro_range in self.gyro_scale_factors:
            self.bus.write_byte_data(self.address, GYRO_CONFIG, gyro_range)
            self.gyro_range = gyro_range
            self.gyro_scale = self.gyro_scale_factors[gyro_range]
            # print(f"Gyro range set to: {hex(gyro_range)}")
        else:
            print(f"Advarsel: Ugyldig gyro range verdi: {hex(gyro_range)}")

    def set_accel_range(self, accel_range):
        """Sets the accelerometer range (+/- 2, 4, 8, or 16g)"""
        if accel_range in self.accel_scale_factors:
            self.bus.write_byte_data(self.address, ACCEL_CONFIG, accel_range)
            self.accel_range = accel_range
            self.accel_scale = self.accel_scale_factors[accel_range]
            # print(f"Accel range set to: {hex(accel_range)}")
        else:
             print(f"Advarsel: Ugyldig accel range verdi: {hex(accel_range)}")

    def get_accel_data(self, g=False):
        """Reads the current acceleration values"""
        x = self.read_word_2c(ACCEL_XOUT_H)
        y = self.read_word_2c(ACCEL_YOUT_H)
        z = self.read_word_2c(ACCEL_ZOUT_H)

        if g:
            # Bruk lagret skaleringsfaktor basert på range
            x = x / self.accel_scale
            y = y / self.accel_scale
            z = z / self.accel_scale

        return {'x': x, 'y': y, 'z': z}

    def get_gyro_data(self):
        """Reads the current gyroscope values"""
        x = self.read_word_2c(GYRO_XOUT_H)
        y = self.read_word_2c(GYRO_YOUT_H)
        z = self.read_word_2c(GYRO_ZOUT_H)

        # Bruk lagret skaleringsfaktor basert på range
        x = x / self.gyro_scale
        y = y / self.gyro_scale
        z = z / self.gyro_scale

        return {'x': x, 'y': y, 'z': z}

    # --- Metoder for vinkel fra akselerometer (fra GitHub repo) ---
    @staticmethod
    def get_accel_pitch(accel_data):
        # Denne beregner egentlig ROLL (rotasjon rundt X)
        return math.atan2(accel_data['y'], accel_data['z']) * 180 / math.pi

    @staticmethod
    def get_accel_roll(accel_data):
         # Denne beregner egentlig PITCH (rotasjon rundt Y)
        return math.atan2(accel_data['x'], accel_data['z']) * 180 / math.pi

    # --- KALIBRERINGSMETODER (forbedret versjon) ---
    def calibrate_accel(self, samples=150):
        print("\nStarter kalibrering av akselerometer.")
        print("VIKTIG: Plasser sensoren HELT I RO og HORISONTALT (flatt)!")
        sleep(2.5)

        offset = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        sum_sq = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        read_errors = 0

        print("Samler akselerometerdata for kalibrering...")
        for i in range(samples):
            try:
                # Les med g=True for å få verdier rundt +/- 1g
                accel_data = self.get_accel_data(g=True)
                offset['x'] += accel_data['x']
                offset['y'] += accel_data['y']
                offset['z'] += accel_data['z']
                sum_sq['x'] += accel_data['x']**2
                sum_sq['y'] += accel_data['y']**2
                sum_sq['z'] += accel_data['z']**2
                sleep(0.02)
                if (i + 1) % 25 == 0:
                    print(f"  ...kalibrering sample {i+1}/{samples}")
            except IOError:
                 read_errors += 1
                 if read_errors > samples // 4:
                     print("FEIL: For mange I/O-feil under aksel.kalibrering.")
                     raise IOError("Kommunikasjonsproblem med MPU6050 under aksel.kalibrering")
                 print(f"Advarsel: IOError under aksel.kalibrering sample {i+1}/{samples}. Fortsetter...")
                 sleep(0.1)

        if samples == 0: return {'x': 0, 'y': 0, 'z': 0}

        offset['x'] /= samples
        offset['y'] /= samples
        offset['z'] /= samples

        std_dev_x = math.sqrt(max(0, sum_sq['x'] / samples - offset['x']**2))
        std_dev_y = math.sqrt(max(0, sum_sq['y'] / samples - offset['y']**2))
        std_dev_z = math.sqrt(max(0, sum_sq['z'] / samples - offset['z']**2))

        print(f"Aksel. Kalibrering Std Dev: x={std_dev_x:.4f}, y={std_dev_y:.4f}, z={std_dev_z:.4f}")
        threshold_g = 0.05
        if max(std_dev_x, std_dev_y, std_dev_z) > threshold_g:
            print(f"ADVARSEL: Akselerometer ustabilt (StdDev > {threshold_g}g) under kalibrering!")

        accel_offset_cal = {'x': offset['x'], 'y': offset['y']}
        g_magnitude_z = abs(offset['z'])
        if g_magnitude_z < 0.8 or g_magnitude_z > 1.2:
             print(f"ADVARSEL: Z-aksen ({offset['z']:.2f}g) ikke nær +/-1g under kalibrering. Er sensoren flat?")
             accel_offset_cal['z'] = offset['z']
        else:
            expected_g_z = np.sign(offset['z'])
            accel_offset_cal['z'] = offset['z'] - expected_g_z

        print(f"Akselerometer offset beregnet: x={accel_offset_cal['x']:.4f}, y={accel_offset_cal['y']:.4f}, z={accel_offset_cal['z']:.4f}")
        return accel_offset_cal

    def calibrate_gyro(self, samples=150):
        print("\nStarter kalibrering av gyroskop.")
        print("VIKTIG: Plasser sensoren HELT I RO!")
        sleep(2.5)

        offset = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        sum_sq = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        read_errors = 0

        print("Samler gyroskopdata for kalibrering...")
        for i in range(samples):
            try:
                gyro_data = self.get_gyro_data() # Får dps
                offset['x'] += gyro_data['x']
                offset['y'] += gyro_data['y']
                offset['z'] += gyro_data['z']
                sum_sq['x'] += gyro_data['x']**2
                sum_sq['y'] += gyro_data['y']**2
                sum_sq['z'] += gyro_data['z']**2
                sleep(0.02)
                if (i + 1) % 25 == 0:
                    print(f"  ...kalibrering sample {i+1}/{samples}")
            except IOError:
                 read_errors += 1
                 if read_errors > samples // 4:
                     print("FEIL: For mange I/O-feil under gyro.kalibrering.")
                     raise IOError("Kommunikasjonsproblem med MPU6050 under gyro.kalibrering")
                 print(f"Advarsel: IOError under gyro.kalibrering sample {i+1}/{samples}. Fortsetter...")
                 sleep(0.1)

        if samples == 0: return {'x': 0, 'y': 0, 'z': 0}

        offset['x'] /= samples
        offset['y'] /= samples
        offset['z'] /= samples

        std_dev_x = math.sqrt(max(0, sum_sq['x'] / samples - offset['x']**2))
        std_dev_y = math.sqrt(max(0, sum_sq['y'] / samples - offset['y']**2))
        std_dev_z = math.sqrt(max(0, sum_sq['z'] / samples - offset['z']**2))

        print(f"Gyro Kalibrering Std Dev: x={std_dev_x:.4f}, y={std_dev_y:.4f}, z={std_dev_z:.4f}")
        threshold_dps = 1.0
        if max(std_dev_x, std_dev_y, std_dev_z) > threshold_dps:
            print(f"ADVARSEL: Gyroskop ustabilt (StdDev > {threshold_dps} dps) under kalibrering!")

        print(f"Gyro offset beregnet: x={offset['x']:.4f}, y={offset['y']:.4f}, z={offset['z']:.4f}")
        return offset

# Testdel (valgfritt)
if __name__ == '__main__':
    print("Tester MPU6050 driver med kalibrering...")
    try:
        sensor = mpu6050(0x68) # Kalibrering kjøres her
        print("\nKalibrering ferdig. Testlesing:")
        for _ in range(5):
            accel_raw = sensor.get_accel_data(g=True)
            gyro_raw = sensor.get_gyro_data()
            # Kalibrer manuelt for testvisning
            acc_cal = {k: accel_raw[k] - sensor.accel_offset[k] for k in accel_raw}
            gyr_cal = {k: gyro_raw[k] - sensor.gyro_offset[k] for k in gyro_raw}
            print(f"AccCal: x={acc_cal['x']:.2f}, y={acc_cal['y']:.2f}, z={acc_cal['z']:.2f} | "
                  f"GyrCal: x={gyr_cal['x']:.2f}, y={gyr_cal['y']:.2f}, z={gyr_cal['z']:.2f}")
            time.sleep(0.2)
    except Exception as e:
        print(f"Feil under testing: {e}")
        import traceback
        traceback.print_exc()