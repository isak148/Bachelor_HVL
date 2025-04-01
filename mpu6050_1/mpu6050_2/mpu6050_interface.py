# -*- coding: utf-8 -*-
import time
import math
import numpy as np
from time import sleep

# Importer den grunnleggende mpu6050-klassen fra din struktur
# Juster stien om nødvendig hvis denne filen ikke ligger i Bachelor_HVL roten
try:
    from mpu6050_1.mpu6050_2.mpu6050 import mpu6050
except ImportError as e:
    print("FEIL: Kunne ikke importere den grunnleggende mpu6050-klassen.")
    print("Sørg for at mpu6050_interface.py ligger i Bachelor_HVL-mappen,")
    print("og at mappen 'mpu6050_1/mpu6050_2/' med mpu6050.py finnes.")
    raise e

class MPU6050_Interface(mpu6050):
    """
    Utvider mpu6050-klassen med forbedret kalibrering og lagring av offset.
    """
    def __init__(self, address=0x68, bus=1):
        print("Initialiserer MPU6050...")
        try:
            super().__init__(address, bus)
            print(f"Koblet til MPU6050 på adresse {hex(address)}.")
        except Exception as e:
            print(f"FEIL: Kunne ikke initialisere MPU6050 baseklasse: {e}")
            raise

        # Standard sensorinnstillinger (juster ved behov)
        try:
            self.set_gyro_range(self.GYRO_RANGE_250DEG)
            self.set_accel_range(self.ACCEL_RANGE_2G)
            self.set_filter_range(self.FILTER_BW_188) # Intern digitalt filter
            print("MPU6050 sensorinnstillinger satt.")
        except Exception as e:
            print(f"FEIL: Kunne ikke sette MPU6050 sensorinnstillinger: {e}")
            # Fortsett uansett, men vær obs på at standardinnstillinger brukes

        # Kjør kalibrering og lagre offsets
        self.gyro_offset = self.calibrate_gyro(samples=150) # Økt samples
        self.accel_offset = self.calibrate_accel(samples=150) # Økt samples

    def calibrate_accel(self, samples=100):
        print("\nStarter kalibrering av akselerometer.")
        print("VIKTIG: Plasser sensoren HELT I RO og HORISONTALT (flatt)!")
        sleep(2.5) # Gi brukeren tid til å plassere sensoren

        offset = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        sum_sq = {'x': 0.0, 'y': 0.0, 'z': 0.0} # For standardavvik
        read_errors = 0

        print("Samler akselerometerdata...")
        for i in range(samples):
            try:
                accel_data = self.get_accel_data(g=True)
                offset['x'] += accel_data['x']
                offset['y'] += accel_data['y']
                offset['z'] += accel_data['z']
                sum_sq['x'] += accel_data['x']**2
                sum_sq['y'] += accel_data['y']**2
                sum_sq['z'] += accel_data['z']**2
                sleep(0.02) # Vent mellom målinger
                if (i + 1) % 25 == 0:
                    print(f"  ...sample {i+1}/{samples}")
            except IOError:
                 read_errors += 1
                 if read_errors > samples // 4: # Hvis for mange feil, avbryt
                     print("FEIL: For mange I/O-feil under aksel.kalibrering.")
                     raise IOError("Kommunikasjonsproblem med MPU6050 under aksel.kalibrering")
                 print(f"Advarsel: IOError under aksel.kalibrering sample {i+1}/{samples}. Fortsetter...")
                 sleep(0.1) # Vent litt ekstra ved feil

        if samples == 0: return {'x': 0, 'y': 0, 'z': 0} # Unngå deling på null

        offset['x'] /= samples
        offset['y'] /= samples
        offset['z'] /= samples

        # Beregn standardavvik for å sjekke stabilitet
        std_dev_x = math.sqrt(max(0, sum_sq['x'] / samples - offset['x']**2))
        std_dev_y = math.sqrt(max(0, sum_sq['y'] / samples - offset['y']**2))
        std_dev_z = math.sqrt(max(0, sum_sq['z'] / samples - offset['z']**2))

        print(f"Aksel. Kalibrering Std Dev: x={std_dev_x:.4f}, y={std_dev_y:.4f}, z={std_dev_z:.4f}")
        threshold_g = 0.05 # Terskel for stabilitet i g
        if max(std_dev_x, std_dev_y, std_dev_z) > threshold_g:
            print(f"ADVARSEL: Akselerometeret var ustabilt (StdDev > {threshold_g}g) under kalibrering! Resultatet kan være upålitelig.")

        # --- Forbedret Offset Beregning ---
        accel_offset_cal = {'x': offset['x'], 'y': offset['y']}
        g_magnitude_z = abs(offset['z'])
        if g_magnitude_z < 0.8 or g_magnitude_z > 1.2: # Bør være nær 1g hvis den ligger flatt
             print(f"ADVARSEL: Z-aksen ({offset['z']:.2f}g) er ikke nær +/-1g. Er sensoren plassert flatt?")
             # Bruk den målte verdien som offset hvis vi ikke er sikre
             accel_offset_cal['z'] = offset['z']
        else:
            expected_g_z = np.sign(offset['z']) # +1.0 eller -1.0
            accel_offset_cal['z'] = offset['z'] - expected_g_z

        print(f"Ferdig kalibrert akselerometer. Offset beregnet til: x={accel_offset_cal['x']:.4f}, y={accel_offset_cal['y']:.4f}, z={accel_offset_cal['z']:.4f}")
        return accel_offset_cal

    def calibrate_gyro(self, samples=100):
        print("\nStarter kalibrering av gyroskop.")
        print("VIKTIG: Plasser sensoren HELT I RO!")
        sleep(2.5) # Gi brukeren tid

        offset = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        sum_sq = {'x': 0.0, 'y': 0.0, 'z': 0.0} # For standardavvik
        read_errors = 0

        print("Samler gyroskopdata...")
        for i in range(samples):
            try:
                gyro_data = self.get_gyro_data()
                offset['x'] += gyro_data['x']
                offset['y'] += gyro_data['y']
                offset['z'] += gyro_data['z']
                sum_sq['x'] += gyro_data['x']**2
                sum_sq['y'] += gyro_data['y']**2
                sum_sq['z'] += gyro_data['z']**2
                sleep(0.02) # Vent mellom målinger
                if (i + 1) % 25 == 0:
                    print(f"  ...sample {i+1}/{samples}")
            except IOError:
                 read_errors += 1
                 if read_errors > samples // 4: # Hvis for mange feil, avbryt
                     print("FEIL: For mange I/O-feil under gyro.kalibrering.")
                     raise IOError("Kommunikasjonsproblem med MPU6050 under gyro.kalibrering")
                 print(f"Advarsel: IOError under gyro.kalibrering sample {i+1}/{samples}. Fortsetter...")
                 sleep(0.1)

        if samples == 0: return {'x': 0, 'y': 0, 'z': 0}

        offset['x'] /= samples
        offset['y'] /= samples
        offset['z'] /= samples

         # Beregn standardavvik
        std_dev_x = math.sqrt(max(0, sum_sq['x'] / samples - offset['x']**2))
        std_dev_y = math.sqrt(max(0, sum_sq['y'] / samples - offset['y']**2))
        std_dev_z = math.sqrt(max(0, sum_sq['z'] / samples - offset['z']**2))

        print(f"Gyro Kalibrering Std Dev: x={std_dev_x:.4f}, y={std_dev_y:.4f}, z={std_dev_z:.4f}")
        threshold_dps = 1.0 # Terskel for stabilitet i grader/sekund
        if max(std_dev_x, std_dev_y, std_dev_z) > threshold_dps:
            print(f"ADVARSEL: Gyroskopet var ustabilt (StdDev > {threshold_dps} dps) under kalibrering! Resultatet kan være upålitelig.")

        print(f"Ferdig kalibrert gyro. Offset beregnet til: x={offset['x']:.4f}, y={offset['y']:.4f}, z={offset['z']:.4f}")
        return offset

    # Vi antar at get_accel_data() og get_gyro_data() finnes i baseklassen
    # Hvis ikke, må de defineres her eller i baseklassen.

# Testdel (valgfritt, for å sjekke klassen isolert)
if __name__ == '__main__':
    print("Tester MPU6050_Interface...")
    try:
        mpu_interface = MPU6050_Interface(0x68)
        print("\nKalibreringsobjekt opprettet.")
        print("Tester datalesing (trykk Ctrl+C for å stoppe):")
        for _ in range(10):
            accel = mpu_interface.get_accel_data(g=True)
            gyro = mpu_interface.get_gyro_data()
            print(f"Accel Raw: x={accel['x']:.2f}, y={accel['y']:.2f}, z={accel['z']:.2f} | "
                  f"Gyro Raw: x={gyro['x']:.2f}, y={gyro['y']:.2f}, z={gyro['z']:.2f}")
            sleep(0.1)
    except Exception as e:
        print(f"En feil oppstod under testing: {e}")
        import traceback
        traceback.print_exc()