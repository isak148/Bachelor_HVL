# -*- coding: utf-8 -*-
import time
import math
import numpy as np
import sys

# Importer klassene fra de andre filene
try:
    from mpu6050_1.mpu6050_2.mpu6050_interface import MPU6050_Interface
    from Mahony import MahonyAHRS
except ImportError as e:
    print(f"FEIL: Kunne ikke importere nødvendige klasser: {e}")
    print("Sørg for at 'mpu6050_interface.py' og 'mahony_ahrs.py' ligger i samme mappe som dette skriptet,")
    print("eller juster import-stiene.")
    sys.exit(1) # Avslutt hvis import feiler

# --- Hovedprogram ---
if __name__ == "__main__":
    try:
        # 1. Initialiser MPU6050_Interface
        #    Kalibreringen skjer automatisk i __init__
        print("-" * 50)
        mpu = MPU6050_Interface(0x68)
        print("-" * 50)

        # Hent kalibreringsoffset for referanse
        gyro_offset = mpu.gyro_offset
        accel_offset = mpu.accel_offset

        # 2. Initialiser MahonyAHRS
        SAMPLE_FREQ = 100.0  # Sett ønsket samplingsfrekvens (Hz)
        KP = 2.0            # Proportional gain (juster ved behov)
        KI = 0.1            # Integral gain (juster ved behov)
        mahony = MahonyAHRS(SAMPLE_FREQ, KP, KI)
        print("-" * 50)

        print("Starter Mahony AHRS loop... Trykk Ctrl+C for å avslutte.")
        actual_loop_start_time = time.time()
        loop_count = 0

        while True:
            loop_start = time.time()

            # 3. Hent RÅ sensor data
            try:
                # Bruk g=True for å få akselerometerdata direkte i g
                accel_data_raw = mpu.get_accel_data(g=True)
                gyro_data_raw = mpu.get_gyro_data()
            except IOError as e:
                print(f"Advarsel: IOError ved lesing fra MPU6050: {e}. Hopper over syklus.")
                time.sleep(0.1)
                continue

            # 4. ANVEND KALIBRERING (VIKTIG!)
            accel_cal = {
                'x': accel_data_raw['x'] - accel_offset['x'],
                'y': accel_data_raw['y'] - accel_offset['y'],
                'z': accel_data_raw['z'] - accel_offset['z']
            }
            gyro_cal = {
                'x': gyro_data_raw['x'] - gyro_offset['x'],
                'y': gyro_data_raw['y'] - gyro_offset['y'],
                'z': gyro_data_raw['z'] - gyro_offset['z']
            }

            # 5. Oppdater Mahony filteret med KALIBRERTE data
            mahony.update(gyro_cal['x'], gyro_cal['y'], gyro_cal['z'],
                          accel_cal['x'], accel_cal['y'], accel_cal['z'])

            # 6. Hent ut vinkler
            angles = mahony.get_angles()

            # 7. Skriv ut resultater (formater litt penere)
            # Merk: Pitch er begrenset +/- 90 i denne Euler-konverteringen
            # Bias er den integrerte feilen, som representerer estimert bias
            print(f"Roll: {angles['roll']:6.1f} | Pitch: {angles['pitch']:6.1f} | Yaw: {angles['yaw']:6.1f} "
                  f"| Bias(x,y,z): {mahony.integral_fb[0]:+6.2f}, {mahony.integral_fb[1]:+6.2f}, {mahony.integral_fb[2]:+6.2f}", end='\r')

            loop_count += 1
            loop_end = time.time()
            processing_time = loop_end - loop_start

            # 8. Sov for å regulere loop-hastigheten
            target_dt = 1.0 / SAMPLE_FREQ
            sleep_time = max(0, target_dt - processing_time)
            time.sleep(sleep_time)

            # Skriv ut gjennomsnittlig frekvens periodisk
            if loop_count % (int(SAMPLE_FREQ) * 5) == 0: # Hvert 5. sekund
                 total_time = time.time() - actual_loop_start_time
                 actual_fs = loop_count / total_time
                 print(f"\n--- Gj.snittlig loop frekvens etter {loop_count} samples: {actual_fs:.2f} Hz ---")

    except ImportError:
        # Feilmelding skrives ut i import-blokken øverst
        pass
    except IOError:
        print("\nFEIL: Mistet kommunikasjon med MPU6050 under kjøring.")
        print("Sjekk tilkoblinger.")
    except KeyboardInterrupt:
        print("\n\nAvslutter programmet.")
    except Exception as e:
        print(f"\nEn uventet feil oppstod: {e}")
        import traceback
        traceback.print_exc()