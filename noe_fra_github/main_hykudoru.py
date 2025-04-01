# -*- coding: utf-8 -*-
# Kode opprinnelig fra:
# https://github.com/Hykudoru/MPU6050-Gyro-Motion-Tracking/blob/main/MPU6050-Gyro-Motion-Tracking/main.py
# Original forfatter: Hykudoru
# MODIFISERT for kalibrering, korrekte navn, og dynamisk R i Kalman

import time
import math
import sys

try:
    from mpu6050_driver_hykudoru import mpu6050
    from kalman_filter_hykudoru import KalmanFilter
except ImportError as e:
    print(f"FEIL: Kunne ikke importere nødvendige klasser: {e}")
    print("Sørg for at 'mpu6050_driver_hykudoru.py' og 'kalman_filter_hykudoru.py' ligger i samme mappe.")
    sys.exit(1)

# --- Initialisering ---
try:
    print("Initialiserer MPU6050 og starter kalibrering...")
    sensor = mpu6050(0x68)
    print("-" * 40)
    print("Kalibrering fullført.")
    print(f"  Gyro Offset: {sensor.gyro_offset}")
    print(f"  Accel Offset: {sensor.accel_offset}")
    print("-" * 40)

    # Renamed filters for clarity based on the axis they process
    kalman_filter_x = KalmanFilter() # For Roll (uses gyro_x)
    kalman_filter_y = KalmanFilter() # For Pitch (uses gyro_y)
    print("Kalman-filtre initialisert.")
    print("-" * 40)

    print("Starter loop (Hykudoru - Korrigert + Dyn R)... Trykk Ctrl+C for å avslutte.")

except Exception as e:
    print(f"Feil under initialisering: {e}")
    sys.exit(1)

# --- Hovedloop ---
while True:
    try:
        gyro_data_raw = sensor.get_gyro_data()
        accel_data_raw = sensor.get_accel_data(g=True)

        accel_cal = {
            'x': accel_data_raw['x'] - sensor.accel_offset['x'],
            'y': accel_data_raw['y'] - sensor.accel_offset['y'],
            'z': accel_data_raw['z'] - sensor.accel_offset['z']
        }
        gyro_cal = {
            'x': gyro_data_raw['x'] - sensor.gyro_offset['x'],
            'y': gyro_data_raw['y'] - sensor.gyro_offset['y'],
            'z': gyro_data_raw['z'] - sensor.gyro_offset['z']
        }

        # Beregn vinkler fra KALIBRERT akselerometer
        accel_roll_from_acc = sensor.get_accel_pitch(accel_cal) # atan2(y, z) -> ROLL
        accel_pitch_from_acc = sensor.get_accel_roll(accel_cal) # atan2(x, z) -> PITCH

        # 6. Oppdater Kalman filtrene - SEND MED accel_cal['z']
        #    Filter X (Roll) får Roll-vinkel, gyro X, og Z-akselerasjon.
        roll = kalman_filter_x.update(accel_roll_from_acc, gyro_cal['x'], accel_cal['z'])
        #    Filter Y (Pitch) får Pitch-vinkel, gyro Y, og Z-akselerasjon.
        pitch = kalman_filter_y.update(accel_pitch_from_acc, gyro_cal['y'], accel_cal['z'])

        print(f"Roll (Est): {roll:6.1f} | Pitch (Est): {pitch:6.1f}     ", end='\r')

        sleep_time = max(0, (1.0/100.0) - kalman_filter_x.dt)
        time.sleep(sleep_time)

    except IOError:
        time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nAvslutter program...")
        break
    except Exception as e:
        print(f"\nEn uventet feil oppstod: {e}")
        import traceback
        traceback.print_exc()
        break