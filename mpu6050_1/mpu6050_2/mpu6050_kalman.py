# -*- coding: utf-8 -*-
from mpu6050_2.mpu6050 import mpu6050
from time import sleep
import math
import time

import os
import csv
from collections import deque
import numpy as np # Viktig for Kalmanfilteret
from scipy.signal import butter, lfilter

#import smbus2

class MPU6050_Orientation(mpu6050):
    def __init__(self, address, bus=1):
        super().__init__(address, bus)

        #Justere følsomhet skala(grader/s) for gyro
        self.set_gyro_range(self.GYRO_RANGE_250DEG)
        #Juster akselrometer følsomhet
        self.set_accel_range(self.ACCEL_RANGE_2G)
        #Velg filtrerng for mpu mer filtrering treigere respons
        self.set_filter_range(self.FILTER_BW_188) # Prøv ulike filterinnstillinger

        self.gyro_offset = self.calibrate_gyro(100) # Øk gjerne samples
        self.accel_offset = self.calibrate_accel(100) # Øk gjerne samples

        # --- Fjernet komplementærfilter-variabler ---
        # self.gyro_angle_x = 0.0
        # self.gyro_angle_y = 0.0
        # self.alpha = 0.95 # Komplementær filtervekt

        self.last_time = time.time()

        # --- Kalman Filter Initialisering ---
        # Tilstandsvektor [vinkel, bias]
        self.x_roll = np.array([[0.0], [0.0]]) # [roll_angle, roll_bias]
        self.x_pitch = np.array([[0.0], [0.0]]) # [pitch_angle, pitch_bias]

        # Kovariansmatrise P (usikkerhet i estimatet)
        self.P_roll = np.eye(2) * 0.1 # Start med litt usikkerhet
        self.P_pitch = np.eye(2) * 0.1

        # Prosess-støy Q (usikkerhet i modellen - hvor mye stoler vi på gyro-integrasjon?)
        # Disse er viktige tuning-parametre!
        self.Q_angle = 0.001 # Usikkerhet i vinkel pga. modell
        self.Q_gyro_bias = 0.003 # Usikkerhet i bias (hvor mye drifter bias?)

        # Målestøy R (usikkerhet i målingen - hvor mye stoler vi på akselerometeret?)
        # Dette er en viktig tuning-parameter!
        self.R_angle = 0.03 # Høyere verdi = mindre tillit til akselerometer

        # variabler for automatiskkalibrering av roll pitch
        self.pitch_offset = 0.0
        self.roll_offset = 0.0
        #variabler for å telle stabile målinger
        self.stable_count_x = 0
        self.stable_count_y = 0
        self.stable_count_z = 0
        self.required_stable_count = 20
        self.threshold = 0.05 # Terskel for 1G deteksjon

        # variabler for FFS
        self.sample_rate = 100  # Hz (Sjekk om dette stemmer med faktisk loop-hastighet)
        self.window_size = int(self.sample_rate * 5)  # 5 sekunder med data
        self.data_buffer = deque(maxlen=self.window_size)
        self.raw_data_buffer = deque(maxlen=self.window_size)
        self.last_periodicity_status = None

    def calibrate_accel(self, samples=100):
        print("starter kalibrering accelrometer")
        offset = {'x':0.0, 'y':0.0, 'z': 0.0}
        sum_sq = {'x':0.0, 'y':0.0, 'z': 0.0} # For å sjekke stabilitet
        for i in range(samples):
            accel_data = self.get_accel_data(g=True)
            offset['x'] += accel_data['x']
            offset['y'] += accel_data['y']
            offset['z'] += accel_data['z']
            sum_sq['x'] += accel_data['x']**2
            sum_sq['y'] += accel_data['y']**2
            sum_sq['z'] += accel_data['z']**2
            sleep(0.02) # Gi tid mellom målinger

        offset['x'] /= samples
        offset['y'] /= samples
        offset['z'] /= samples

        # Sjekk standardavvik for å se om sensoren var stabil
        std_dev_x = math.sqrt(sum_sq['x'] / samples - offset['x']**2)
        std_dev_y = math.sqrt(sum_sq['y'] / samples - offset['y']**2)
        std_dev_z = math.sqrt(sum_sq['z'] / samples - offset['z']**2)
        print(f"Accel Cal Std Dev: x={std_dev_x:.4f}, y={std_dev_y:.4f}, z={std_dev_z:.4f}")
        if max(std_dev_x, std_dev_y, std_dev_z) > 0.1: # Juster terskel etter behov
             print("ADVARSEL: Akselerometeret var ustabilt under kalibrering!")

        # Juster z for gravitasjon *etter* gjennomsnittet er tatt
        # Anta at z-aksen peker oppover (eller nedover) under kalibrering
        # Finn hvilken akse som er nærmest +/- 1G
        g_val = math.sqrt(offset['x']**2 + offset['y']**2 + offset['z']**2)
        print(f"Gjennomsnittlig G under accel kalibrering: {g_val:.3f}")
        # Vanligvis justeres z for 1G, men hvis den ligger flatt,
        # kan det være x eller y. Dette forutsetter at den ligger flatt.
        offset['z'] -= np.sign(offset['z']) # Trekk fra 1 eller -1 avhengig av retning

        print(f"Ferdig kalibrert accelrometer. Offset: x={offset['x']:.3f}, y={offset['y']:.3f}, z={offset['z']:.3f}")
        return offset

    def calibrate_gyro(self, samples=100):
        print("starter kalibrering gyro")
        offset = {'x':0.0, 'y':0.0, 'z': 0.0}
        sum_sq = {'x':0.0, 'y':0.0, 'z': 0.0} # For å sjekke stabilitet
        for _ in range(samples):
            gyro_data = self.get_gyro_data()
            offset['x'] += gyro_data['x']
            offset['y'] += gyro_data['y']
            offset['z'] += gyro_data['z']
            sum_sq['x'] += gyro_data['x']**2
            sum_sq['y'] += gyro_data['y']**2
            sum_sq['z'] += gyro_data['z']**2
            sleep(0.02) # Gi tid mellom målinger

        offset['x'] /= samples
        offset['y'] /= samples
        offset['z'] /= samples

        # Sjekk standardavvik
        std_dev_x = math.sqrt(sum_sq['x'] / samples - offset['x']**2)
        std_dev_y = math.sqrt(sum_sq['y'] / samples - offset['y']**2)
        std_dev_z = math.sqrt(sum_sq['z'] / samples - offset['z']**2)
        print(f"Gyro Cal Std Dev: x={std_dev_x:.4f}, y={std_dev_y:.4f}, z={std_dev_z:.4f}")
        if max(std_dev_x, std_dev_y, std_dev_z) > 0.5: # Juster terskel etter behov
             print("ADVARSEL: Gyroskopet var ustabilt under kalibrering!")

        print(f"Ferdig kalibrert gyro. Offset: x={offset['x']:.3f}, y={offset['y']:.3f}, z={offset['z']:.3f}")
        return offset

    def _kalman_update(self, x, P, rate, dt, accel_angle, offset_angle):
        """ Hjelpefunksjon for Kalman filter oppdatering for én akse """
        angle = x[0, 0]
        bias = x[1, 0]

        # --- Prediksjonssteg ---
        # Oppdater vinkelestimat med gyro (+ bias korreksjon)
        angle += dt * (rate - bias)

        # Oppdater kovariansmatrise P
        # P = F * P * F^T + Q
        F = np.array([[1, -dt], [0, 1]])
        Q = np.array([[self.Q_angle, 0], [0, self.Q_gyro_bias]]) * dt # Skaler Q med dt? Vanlig praksis
        P = F @ P @ F.T + Q

        # --- Oppdateringssteg ---
        # Sammenlign med akselerometer-vinkel (justert for offset)
        measurement = accel_angle - offset_angle
        y = measurement - angle # Måle-residual (innovasjon)

        # H = [1 0] (vi måler bare vinkelen direkte)
        # S = H * P * H^T + R
        H = np.array([[1, 0]])
        R_mat = np.array([[self.R_angle]])
        S = H @ P @ H.T + R_mat # Residual kovarians
        S_inv = 1.0 / S[0,0] # Inverter S (er 1x1 matrise)

        # Kalman Gain K
        # K = P * H^T * S^-1
        K = P @ H.T * S_inv # K blir en 2x1 vektor

        # Oppdater tilstandsestimat x
        # x = x + K * y
        x = np.array([[angle], [bias]]) + K * y

        # Oppdater kovariansmatrise P
        # P = (I - K * H) * P
        I = np.eye(2)
        P = (I - K @ H) @ P

        return x, P


    def get_orientation(self): # Fjerner dt argumentet, beregnes internt

        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= 0: # Unngå deling på null eller negativ tid
             dt = 1 / self.sample_rate # Bruk nominell sample rate hvis tiden står stille

        self.last_time = current_time

        # Hent rådata
        accel_data_raw = self.get_accel_data(g=True)
        gyro_data_raw = self.get_gyro_data()

        # Juster for kalibrert bias (offset)
        accel_data = {
            'x': accel_data_raw['x'] - self.accel_offset['x'],
            'y': accel_data_raw['y'] - self.accel_offset['y'],
            'z': accel_data_raw['z'] - self.accel_offset['z'] # Merk: z-offset inkluderer IKKE -1g justering
        }
        gyro_data = {
            'x': gyro_data_raw['x'] - self.gyro_offset['x'],
            'y': gyro_data_raw['y'] - self.gyro_offset['y'],
            'z': gyro_data_raw['z'] - self.gyro_offset['z']
        }

        # Beregn vinkel fra akselerometer i grader
        # Bruker atan2 for riktig kvadrant og unngår deling på null
        # Sørg for at argumentene til atan2 er i riktig rekkefølge for ønsket vinkel
        # Roll (rotasjon rundt X-aksen): Bruker Y og Z
        accel_angle_x = math.atan2(accel_data['y'],
                                   math.sqrt(accel_data['x']**2 + accel_data['z']**2)) * (180/math.pi)
        # Pitch (rotasjon rundt Y-aksen): Bruker X og Z
        accel_angle_y = math.atan2(-accel_data['x'],
                                   math.sqrt(accel_data['y']**2 + accel_data['z']**2)) * (180/math.pi)
        # Alternativ pitch-beregning (mer vanlig, men kan være ustabil nær +/- 90 grader):
        # accel_angle_y = math.atan2(-accel_data['x'], accel_data['z']) * (180/math.pi)


        # --- Automatisk nullstilling/offset justering (fra din originale kode) ---
        # Sjekker om en av aksene er stabil på ca +/- 1G
        # (Bruker rå, *ukalibrerte* data for å sjekke mot 1G, da kalibreringen fjerner gravitasjonskomponenten)
        g_total_raw = math.sqrt(accel_data_raw['x']**2 + accel_data_raw['y']**2 + accel_data_raw['z']**2)
        # Normaliser rådata for å sjekke mot 1G uavhengig av små variasjoner i total G
        if g_total_raw > 0.1: # Unngå deling på null
            norm_ax = accel_data_raw['x'] / g_total_raw
            norm_ay = accel_data_raw['y'] / g_total_raw
            norm_az = accel_data_raw['z'] / g_total_raw
        else:
            norm_ax, norm_ay, norm_az = 0, 0, 0

        # Øk tellere hvis en akse er nær +/- 1
        self.stable_count_x = self.stable_count_x + 1 if abs(norm_ax) > (1.0 - self.threshold) else 0
        self.stable_count_y = self.stable_count_y + 1 if abs(norm_ay) > (1.0 - self.threshold) else 0
        self.stable_count_z = self.stable_count_z + 1 if abs(norm_az) > (1.0 - self.threshold) else 0

        reset_occurred = False
        if self.stable_count_z >= self.required_stable_count:
            # Z-aksen er stabil (enheten ligger flatt)
            self.roll_offset = 0.0 # accel_angle_x er nå referansen for roll
            self.pitch_offset = 0.0 # accel_angle_y er nå referansen for pitch
            # Resett Kalman filterets vinkel til akselerometerets vinkel
            # La bias-estimatet være som det er
            self.x_roll[0,0] = accel_angle_x
            self.x_pitch[0,0] = accel_angle_y
            # Nullstill P-matrise vinkel-elementer for å vise høy tillit til ny start
            self.P_roll[0,0] = 0.01
            self.P_pitch[0,0] = 0.01
            self.stable_count_z = 0 # Resetter teller
            reset_occurred = True
            # print("RESET: Z stabil")

        elif self.stable_count_y >= self.required_stable_count:
             # Y-aksen er stabil (enheten står på siden)
             # Bestem roll/pitch offset basert på Y-verdi (+1g eller -1g)
            if norm_ay > 0: # +1g på Y -> Roll = +90 grader
                self.roll_offset = 90.0
                self.pitch_offset = 0.0 # Pitch er fortsatt referert fra Z
            else: # -1g på Y -> Roll = -90 grader
                self.roll_offset = -90.0
                self.pitch_offset = 0.0
            # Resett Kalman filterets vinkel
            self.x_roll[0,0] = accel_angle_x - self.roll_offset # Juster startvinkel med offset
            self.x_pitch[0,0] = accel_angle_y # Pitch referanse er uendret
            self.P_roll[0,0] = 0.01
            self.P_pitch[0,0] = 0.01
            self.stable_count_y = 0 # Resetter teller
            reset_occurred = True
            # print(f"RESET: Y stabil, Roll Offset={self.roll_offset}")

        elif self.stable_count_x >= self.required_stable_count:
            # X-aksen er stabil (enheten står på enden)
            # Bestem roll/pitch offset basert på X-verdi (+1g eller -1g)
            if norm_ax > 0: # +1g på X -> Pitch = +90 grader
                self.pitch_offset = 90.0
                self.roll_offset = 0.0 # Roll er fortsatt referert fra Z
            else: # -1g på X -> Pitch = -90 grader
                self.pitch_offset = -90.0
                self.roll_offset = 0.0
             # Resett Kalman filterets vinkel
            self.x_pitch[0,0] = accel_angle_y - self.pitch_offset # Juster startvinkel med offset
            self.x_roll[0,0] = accel_angle_x # Roll referanse er uendret
            self.P_pitch[0,0] = 0.01
            self.P_roll[0,0] = 0.01
            self.stable_count_x = 0 # Resetter teller
            reset_occurred = True
            # print(f"RESET: X stabil, Pitch Offset={self.pitch_offset}")

        # --- Kalman Filter - Kjør for Roll og Pitch ---
        # Bruker den interne hjelpefunksjonen
        # Rate = gyro rate for den aksen
        # accel_angle = vinkel beregnet fra akselerometer for den aksen
        # offset_angle = gjeldende offset bestemt av stabilitetslogikken over

        self.x_roll, self.P_roll = self._kalman_update(
            self.x_roll, self.P_roll, gyro_data['x'], dt, accel_angle_x, self.roll_offset
        )

        self.x_pitch, self.P_pitch = self._kalman_update(
            self.x_pitch, self.P_pitch, gyro_data['y'], dt, accel_angle_y, self.pitch_offset
        )

        # Returner de estimerte vinklene fra Kalmanfilteret
        return {'roll': self.x_roll[0, 0], 'pitch': self.x_pitch[0, 0]}


    # --- Resten av metodene dine (FFT, etc.) forblir de samme ---

    def butter_highpass(self, cutoff, fs, order=5): # butter høypass filter
        nyquist = 0.5 * fs
        normal_cutoff = cutoff / nyquist
        b, a = butter(order, normal_cutoff, btype='high', analog=False)
        return b, a

    def highpass_filter(self, data, cutoff, fs, order=5): # kaller butter_highpass filteret og gir 1 returverdi.
        b, a = self.butter_highpass(cutoff, fs, order=order)
        y = lfilter(b, a, data)
        return y

    def is_periodic(self, signal, threshold_ratio=0.1, min_significant_freqs=1):
        """Vurderer om et signal inneholder et periodisk mønster basert på FFT-analyse."""
        N = len(signal)
        if N < 2: # Trenger minst 2 punkter for FFT
             return False
        fft_values = np.fft.rfft(signal[1:]) # Hopp over første element hvis det kan være påvirket av filtertransient
        fft_magnitude = np.abs(fft_values) / (N-1) # Normaliser med N-1
        max_magnitude = np.max(fft_magnitude)
        if max_magnitude < 1e-6: # Unngå terskel på null hvis signalet er flatt
            return False
        threshold = threshold_ratio * max_magnitude
        # Teller frekvenser *over* DC-komponenten (indeks 0)
        significant_freqs = np.sum(fft_magnitude[1:] > threshold)
        return significant_freqs >= min_significant_freqs


    def gi_status_aks(self):
        # Hent rå akselerasjonsdata (før kalibrering) for G-kraft beregning
        accel_data = self.get_accel_data(g=True)
        tot_G = math.sqrt(accel_data['x']**2 + accel_data['y']**2 + accel_data['z']**2)

        # Legg til den nyeste *total G* målingen i en buffer for filtrering og periodisitetsanalyse
        self.raw_data_buffer.append(tot_G)

        current_periodicity_status = self.last_periodicity_status # Behold forrige status inntil nok data

        # Når bufferen har nok data, filtrer og vurder periodisiteten
        if len(self.raw_data_buffer) >= self.window_size:
            # Filtrer dataene
            # Konverter deque til liste eller numpy array for lfilter
            data_to_filter = list(self.raw_data_buffer)
            # Bruk try/except rundt filtrering, da det kan feile med ustabile data
            try:
                filtered_data = self.highpass_filter(data_to_filter, cutoff=0.5, fs=self.sample_rate, order=5)
                # Vurder periodisiteten basert på de filtrerte dataene
                current_periodicity_status = self.is_periodic(filtered_data)

            except ValueError as e:
                 print(f"Filter/FFT feil: {e}. Data: {data_to_filter[:10]}...") # Skriv ut litt data for feilsøking
                 current_periodicity_status = False # Anta ujevnt ved feil


            self.last_periodicity_status = current_periodicity_status
            # Tøm bufferen for neste vindu - Bruk clear() for deque
            self.raw_data_buffer.clear()
            # Vi trenger ikke data_buffer lenger ser det ut som?


        # Hent orienteringsdata (nå fra Kalmanfilteret)
        orientation = self.get_orientation()
        roll = orientation['roll']
        pitch = orientation['pitch']

        # Returner resultatene
        return {
            'total_G': tot_G,
            'roll': roll,
            'pitch': pitch,
            'is_periodic': current_periodicity_status # Returner siste status
        }

# --- Hoved-loop ---
if __name__ == "__main__":
    try:
        mpu = MPU6050_Orientation(0x68)
        print("MPU6050 initialisert.")
        print("Starter datainnsamling (Trykk Ctrl+C for å avslutte)...")

        sample_count = 0
        start_loop_time = time.time()

        while True:
            loop_start_time = time.time() # Tid for starten av denne loopen

            status = mpu.gi_status_aks()

            # Formater output for bedre lesbarhet
            g_str = f"Total G: {status['total_G']:.2f}"
            roll_str = f"Roll: {status['roll']:.2f}"
            pitch_str = f"Pitch: {status['pitch']:.2f}"
            # Vis bias også for debugging
            # roll_bias_str = f"Roll Bias: {mpu.x_roll[1, 0]:.3f}"
            # pitch_bias_str = f"Pitch Bias: {mpu.x_pitch[1, 0]:.3f}"

            # Sjekk om periodisitet er None (første vindu er ikke fullt)
            if status['is_periodic'] is None:
                period_str = "Periodisitet: Venter på data..."
            else:
                period_str = f"Periodisitet: {'Jevn' if status['is_periodic'] else 'Ujevn'}"

            print(f"{g_str:<18} | {roll_str:<18} | {pitch_str:<18} | {period_str}")
            # print(f"{roll_bias_str:<25} | {pitch_bias_str:<25}") # Fjern kommentar for å se bias

            sample_count += 1
            end_loop_time = time.time()
            loop_duration = end_loop_time - loop_start_time
            target_sleep = max(0, (1.0 / mpu.sample_rate) - loop_duration)
            sleep(target_sleep) # Prøv å holde sample rate

            # Beregn og skriv ut faktisk sample rate av og til
            if sample_count % 100 == 0:
                 total_time = time.time() - start_loop_time
                 actual_fs = sample_count / total_time
                 print(f"--- Faktisk sample rate: {actual_fs:.2f} Hz ---")


    except ImportError:
        print("Feil: Biblioteket 'mpu6050' eller 'numpy'/'scipy' er ikke installert.")
        print("Installer med: pip install mpu6050-raspberrypi numpy scipy")
    except IOError:
        print("Feil: Kunne ikke kommunisere med MPU6050 på adressen 0x68.")
        print("Sjekk tilkoblinger og I2C-konfigurasjon (sudo raspi-config).")
        print("Kjør 'i2cdetect -y 1' for å se tilkoblede enheter.")
    except KeyboardInterrupt:
        print("\nAvslutter programmet.")
    except Exception as e:
        print(f"\nEn uventet feil oppstod: {e}")
        import traceback
        traceback.print_exc()