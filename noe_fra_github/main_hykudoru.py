# -*- coding: utf-8 -*-
# Kode opprinnelig fra:
# https://github.com/Hykudoru/MPU6050-Gyro-Motion-Tracking/blob/main/MPU6050-Gyro-Motion-Tracking/main.py
# Original forfatter: Hykudoru
# MODIFISERT for å bruke kalibrering og korrekte Roll/Pitch-navn

import time
import math
import sys

# Importer klassene fra de andre filene vi lagde
try:
    # Bruker den modifiserte driveren med kalibrering
    from mpu6050_driver_hykudoru import mpu6050
    from kalman_filter_hykudoru import KalmanFilter
except ImportError as e:
    print(f"FEIL: Kunne ikke importere nødvendige klasser: {e}")
    print("Sørg for at 'mpu6050_driver_hykudoru.py' og 'kalman_filter_hykudoru.py' ligger i samme mappe.")
    sys.exit(1)

# --- Initialisering ---
try:
    # 1. Initialiser MPU6050 (kalibrering skjer i __init__)
    print("Initialiserer MPU6050 og starter kalibrering...")
    sensor = mpu6050(0x68)
    print("-" * 40)
    print("Kalibrering fullført.")
    print(f"  Gyro Offset: {sensor.gyro_offset}")
    print(f"  Accel Offset: {sensor.accel_offset}")
    print("-" * 40)


    # 2. Initialiser Kalman-filtre (uten argumenter)
    # Renamed filters for clarity based on the axis they process (using gyro input axis)
    kalman_filter_x = KalmanFilter() # For Roll (uses gyro_x)
    kalman_filter_y = KalmanFilter() # For Pitch (uses gyro_y)
    print("Kalman-filtre initialisert.")
    print("-" * 40)


    print("Starter loop for å lese sensor og kjøre Kalman filter (Hykudoru - Korrigert)...")
    print("Trykk Ctrl+C for å avslutte.")

except Exception as e:
    print(f"Feil under initialisering: {e}")
    sys.exit(1)

# --- Hovedloop ---
while True:
    try:
        # 3. Les RÅ data fra sensor
        gyro_data_raw = sensor.get_gyro_data()  # Får data i grader/sekund
        accel_data_raw = sensor.get_accel_data(g=True) # Får data i g

        # 4. ANVEND KALIBRERING
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

        # 5. Beregn vinkler KUN fra KALIBRERT akselerometer
        #    og bruk korrekte navn basert på hva formlene faktisk beregner.
        #    sensor.get_accel_pitch(data) bruker atan2(y, z) -> Beregner ROLL
        accel_roll_from_acc = sensor.get_accel_pitch(accel_cal)
        #    sensor.get_accel_roll(data) bruker atan2(x, z) -> Beregner PITCH
        accel_pitch_from_acc = sensor.get_accel_roll(accel_cal)

        # 6. Oppdater Kalman filtrene med KORREKT data-parring
        #    Filter X (Roll) får Roll-vinkel fra aksel. og gyro X rate.
        roll = kalman_filter_x.update(accel_roll_from_acc, gyro_cal['x'])
        #    Filter Y (Pitch) får Pitch-vinkel fra aksel. og gyro Y rate.
        pitch = kalman_filter_y.update(accel_pitch_from_acc, gyro_cal['y'])

        # 7. Skriv ut filtrerte vinkler med KORREKTE labels
        print(f"Roll (Est): {roll:6.1f} | Pitch (Est): {pitch:6.1f}     ", end='\r')

        # 8. Pause for å kontrollere loop-hastighet (ca. 100 Hz)
        #    Bruk dt fra et av filtrene for å justere søvntiden
        #    (begge filtrene bør ha ca. samme dt)
        sleep_time = max(0, (1.0/100.0) - kalman_filter_x.dt) # Sikter mot 100 Hz
        time.sleep(sleep_time)

    except IOError:
        # print("\nIOError: Kunne ikke lese fra MPU6050. Prøver igjen...")
        time.sleep(0.1) # Vent litt ved I/O feil
    except KeyboardInterrupt:
        print("\nAvslutter program...")
        break
    except Exception as e:
        print(f"\nEn uventet feil oppstod: {e}")
        import traceback
        traceback.print_exc()
        break