# -*- coding: utf-8 -*-
# Kode hentet og tilpasset fra:
# https://github.com/Hykudoru/MPU6050-Gyro-Motion-Tracking/blob/main/MPU6050-Gyro-Motion-Tracking/main.py
# Original forfatter: Hykudoru

import time
import math

# Importer klassene fra de andre filene vi lagde
try:
    from mpu6050_driver_hykudoru import mpu6050
    from kalman_filter_hykudoru import KalmanFilter
except ImportError as e:
    print(f"FEIL: Kunne ikke importere nødvendige klasser: {e}")
    print("Sørg for at 'mpu6050_driver_hykudoru.py' og 'kalman_filter_hykudoru.py' ligger i samme mappe.")
    exit()

# --- Initialisering ---
try:
    # Initialiser MPU6050 sensor
    sensor = mpu6050(0x68) # Bruker standard adresse

    # Initialiser to separate Kalman-filtre
    # Parametrene (R_angle, R_bias, R_measure) kan trenge tuning.
    # Merk: Navnene R/Q er byttet om i repoet ift. standard Kalman-notasjon.
    # Jeg brukte standardnotasjon (Q for prosess, R for måling) i klassedefinisjonen over.
    kalman_filter_pitch = KalmanFilter(Q_angle=0.001, Q_bias=0.003, R_measure=0.03)
    kalman_filter_roll = KalmanFilter(Q_angle=0.001, Q_bias=0.003, R_measure=0.03)

    # Tid for første loop
    last_read_time = time.time()

    print("Starter loop for å lese sensor og kjøre Kalman filter (Hykudoru)...")
    print("Trykk Ctrl+C for å avslutte.")

except Exception as e:
    print(f"Feil under initialisering: {e}")
    exit()

# --- Hovedloop ---
while True:
    try:
        # Les data fra sensor
        # Få rådata først for å beregne dt mer nøyaktig før Kalman update
        gyro_data = sensor.get_gyro_data()  # Får data i grader/sekund
        accel_data = sensor.get_accel_data(g=True) # Får data i g

        # Beregn vinkler KUN fra akselerometer (bruker metodene fra repoet)
        # ADVARSEL: Disse er ustabile nær 90 grader tilt!
        # Og merk at 'pitch' her er basert på y/z (vanligvis roll)
        # og 'roll' er basert på x/z (ikke standard pitch)
        accel_pitch_angle = sensor.get_accel_pitch(accel_data)
        accel_roll_angle = sensor.get_accel_roll(accel_data)

        # Oppdater Kalman filtrene
        # Send inn akselerometer-vinkel og gyro-rate for hver akse
        pitch = kalman_filter_pitch.update(accel_pitch_angle, gyro_data['x'])
        roll = kalman_filter_roll.update(accel_roll_angle, gyro_data['y'])
        # Merk: gyro_data['z'] brukes ikke i dette oppsettet

        # Skriv ut filtrerte vinkler
        # Bruk \r for å overskrive linjen for enklere lesing
        print(f"Pitch (Est): {pitch:6.1f} | Roll (Est): {roll:6.1f}     ", end='\r')

        # Pause for å kontrollere loop-hastighet (ca. 100 Hz)
        time.sleep(0.01)

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