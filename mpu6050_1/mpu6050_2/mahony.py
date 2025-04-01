from mpu6050 import mpu6050
from time import sleep
import math
import time

import os
# import csv # Ikke lenger brukt i hovedløkken
from collections import deque
import numpy as np
from scipy.signal import butter, lfilter

# import smbus2

class MPU6050_Orientation(mpu6050):
    def __init__(self, address, bus=1):
        super().__init__(address, bus)

        # Justere følsomhet skala(grader/s) for gyro
        self.set_gyro_range(self.GYRO_RANGE_250DEG)
        # Juster akselrometer følsomhet
        self.set_accel_range(self.ACCEL_RANGE_2G)
        # Velg filtrering for mpu mer filtrering treigere respons
        self.set_filter_range(self.FILTER_BW_188) # Lavere BW gir mer filtrering, men tregere respons

        print("Starter kalibrering...")
        self.gyro_offset = self.calibrate_gyro(100) # Økt antall samples for bedre kalibrering
        self.accel_offset = self.calibrate_accel(100) # Økt antall samples
        print("Kalibrering ferdig.")

        # Mahony filter variabler
        self.Kp = 2.0  # Proportional gain - hvor mye akselerometeret påvirker korreksjonen
        self.Ki = 0.005 # Integral gain - hvor mye biasen korrigeres over tid
        # Initial orientering som kvaternion (w, x, y, z) - starter i 'null'-orientering
        self.q0, self.q1, self.q2, self.q3 = 1.0, 0.0, 0.0, 0.0
        # Estimert integrert gyro bias
        self.integral_bias_x, self.integral_bias_y, self.integral_bias_z = 0.0, 0.0, 0.0

        self.last_time = time.time()

        # --- FFS variabler (uendret) ---
        self.sample_rate = 100  # Hz (Prøv å match dette med faktisk loop-hastighet)
        self.window_size = self.sample_rate * 5  # 5 sekunder med data
        self.data_buffer = deque(maxlen=self.window_size)
        self.raw_data_buffer = deque(maxlen=self.window_size)
        self.last_periodicity_status = None
        # --- Slutt FFS variabler ---


    # --- Kalibreringsfunksjoner (uendret, men økte samples i __init__) ---
    def calibrate_accel(self, samples=100):
        print("starter kalibrering accelrometer")
        offset = {'x':0.0, 'y':0.0, 'z': 0.0}
        sum_z = 0.0
        for _ in range(samples):
            try:
                accel_data = self.get_accel_data(g=False) # Bruk m/s^2 for potensielt bedre bias-fjerning
                offset['x'] += accel_data['x']
                offset['y'] += accel_data['y']
                sum_z += accel_data['z'] # Kalibrer z rundt forventet gravitasjon (ca. 9.81)
            except IOError:
                print("IOError under akselerometerkalibrering, prøver igjen...")
                sleep(0.05)
                continue # Hopp over denne iterasjonen
            sleep(0.02) # Kortere sleep hvis mulig

        offset['x'] /= samples
        offset['y'] /= samples
        # Forventer at z-aksen peker opp eller ned, mål avvik fra 1G (eller 9.81 m/s^2)
        # Antar at sensoren ligger flatt under kalibrering (z peker mot/fra G)
        # For å få G, må vi vite hvilken vei sensoren lå. La oss anta z peker opp (-G).
        # Hvis den lå motsatt vei, blir offset feil. Bedre å bare fjerne gjennomsnittet.
        # En mer robust kalibrering krever flere posisjoner.
        # Vi bruker get_accel_data(g=True) senere, så vi kalibrerer rundt 0 for x/y og 1 for z.
        # La oss gå tilbake til g=True for konsistens med resten av koden
        offset = {'x':0.0, 'y':0.0, 'z': 0.0}
        for _ in range(samples):
            try:
                 accel_data_g = self.get_accel_data(g=True)
                 offset['x'] += accel_data_g['x']
                 offset['y'] += accel_data_g['y']
                 offset['z'] += accel_data_g['z']
            except IOError:
                 print("IOError under akselerometerkalibrering, prøver igjen...")
                 sleep(0.05)
                 continue
            sleep(0.02)

        print("Ferdig kalibrert accelrometer")
        offset['x'] /= samples
        offset['y'] /= samples
        # Juster Z for å være relativt til 1G (antar Z peker opp eller ned)
        # Dette antar at Z-aksen er noenlunde vertikal under kalibrering.
        z_avg = offset['z'] / samples
        if z_avg > 0.5 : # Antar Z peker nedover (måler +1G)
            offset['z'] = z_avg - 1.0
        elif z_avg < -0.5: # Antar Z peker oppover (måler -1G)
             offset['z'] = z_avg + 1.0
        else: # Antar Z er horisontal (bør være nær 0G)
            offset['z'] = z_avg
        # Deling med samples er allerede gjort i z_avg
        offset['z'] = offset['z'] # Bare behold gjennomsnittet minus 1/-1

        # Korriger: Offset skal være det vi *trekker fra*, så det er gjennomsnittsverdien selv
        offset['x'] = offset['x'] / samples
        offset['y'] = offset['y'] / samples
        offset['z'] = z_avg # Lagre gjennomsnittlig Z-verdi målt
        print(f"Accel Calibrated Offsets: x={offset['x']:.3f}, y={offset['y']:.3f}, z={offset['z']:.3f} (avg Z reading)")
        # VIKTIG: Vi må justere Z-målingen senere basert på orientering hvis vi vil ha 0 offset.
        # Foreløpig fjerner vi bare gjennomsnitts-biasen for x og y, og lagrer gj.snitt Z.

        return offset

    def calibrate_gyro(self, samples=100):
        print("starter kalibrering gyro")
        offset = {'x':0.0, 'y':0.0, 'z': 0.0}
        for _ in range(samples):
           try:
               gyro_data = self.get_gyro_data() # Grader per sekund
               offset['x'] += gyro_data['x']
               offset['y'] += gyro_data['y']
               offset['z'] += gyro_data['z']
           except IOError:
               print("IOError under gyrokalibrering, prøver igjen...")
               sleep(0.05)
               continue # Hopp over denne iterasjonen
           sleep(0.02) # Kortere sleep hvis mulig

        print("Ferdig kalibrert gyro")
        offset['x'] /= samples
        offset['y'] /= samples
        offset['z'] /= samples
        print(f"Gyro Calibrated Offsets: x={offset['x']:.3f}, y={offset['y']:.3f}, z={offset['z']:.3f}")
        return offset

    # --- Mahony AHRS oppdateringsfunksjon ---
    def update_mahony(self, gx, gy, gz, ax, ay, az, dt):
        """
        Oppdaterer orienteringsestimatet (kvaternion) ved hjelp av Mahony-filteret.
        Args:
            gx, gy, gz: Gyroskopdata i *radianer* per sekund.
            ax, ay, az: Akselerometerdata (normalisert).
            dt: Tidsintervall siden siste oppdatering i sekunder.
        """
        if dt <= 0: return # Unngå divisjon med null eller tidsreise

        q0, q1, q2, q3 = self.q0, self.q1, self.q2, self.q3
        integral_bias_x, integral_bias_y, integral_bias_z = self.integral_bias_x, self.integral_bias_y, self.integral_bias_z

        # Normaliser akselerometer-målingen
        norm_acc = math.sqrt(ax * ax + ay * ay + az * az)
        if norm_acc == 0.0:
            # Kan ikke bruke akselerometerdata hvis magnituden er null (usannsynlig, men trygt å sjekke)
            print("Warning: Accelerometer magnitude is zero.")
            return
        ax /= norm_acc
        ay /= norm_acc
        az /= norm_acc

        # Estimer gravitasjonsretningen basert på nåværende kvaternion
        # v_x, v_y, v_z er komponentene til gravitasjonsvektoren [0,0,1] rotert av kvaternionen (q^-1 * [0,0,1] * q)
        vx = 2.0 * (q1 * q3 - q0 * q2)
        vy = 2.0 * (q0 * q1 + q2 * q3)
        vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3

        # Beregn feilen mellom målt akselerasjon (gravitasjon) og estimert gravitasjon
        # Feilen er kryssproduktet mellom målt og estimert retning
        error_x = (ay * vz - az * vy)
        error_y = (az * vx - ax * vz)
        error_z = (ax * vy - ay * vx)

        # Integrer feilen for å estimere gyro-bias (Ki-termen)
        if self.Ki > 0.0:
            integral_bias_x += error_x * self.Ki * dt
            integral_bias_y += error_y * self.Ki * dt
            integral_bias_z += error_z * self.Ki * dt
        else:
            integral_bias_x = 0.0  # Ingen bias-korreksjon hvis Ki = 0
            integral_bias_y = 0.0
            integral_bias_z = 0.0

        # Juster gyro-målingene med estimert bias og proporsjonal feil (Kp-termen)
        gx += integral_bias_x + self.Kp * error_x
        gy += integral_bias_y + self.Kp * error_y
        gz += integral_bias_z + self.Kp * error_z

        # Oppdater kvaternionen ved å integrere de justerte gyro-ratene
        # Halvér ratene fordi vi bruker dt/2 i integrasjonen? Nei, standard er rate*dt
        # Standard RK4-lignende oppdatering (førsteordens approksimasjon)
        delta_q0 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz) * dt
        delta_q1 = 0.5 * ( q0 * gx + q2 * gz - q3 * gy) * dt
        delta_q2 = 0.5 * ( q0 * gy - q1 * gz + q3 * gx) * dt
        delta_q3 = 0.5 * ( q0 * gz + q1 * gy - q2 * gx) * dt

        q0 += delta_q0
        q1 += delta_q1
        q2 += delta_q2
        q3 += delta_q3

        # Normaliser kvaternionen for å unngå drift pga. numeriske feil
        norm_q = math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
        if norm_q == 0.0:
           print("Error: Quaternion magnitude zero during normalization!")
           # Reset til identity quaternion for å unngå krasj
           q0, q1, q2, q3 = 1.0, 0.0, 0.0, 0.0
        else:
           q0 /= norm_q
           q1 /= norm_q
           q2 /= norm_q
           q3 /= norm_q

        # Lagre den oppdaterte tilstanden
        self.q0, self.q1, self.q2, self.q3 = q0, q1, q2, q3
        self.integral_bias_x, self.integral_bias_y, self.integral_bias_z = integral_bias_x, integral_bias_y, integral_bias_z


    def get_orientation_mahony(self):
        """Henter sensordata, kjører Mahony-filteret og returnerer roll/pitch."""
        try:
            current_time = time.time()
            dt = current_time - self.last_time
            self.last_time = current_time

            if dt <= 0: dt = 1.0 / self.sample_rate # Estimer dt hvis tiden står stille

            # Hent rådata
            accel_data = self.get_accel_data(g=True) # Bruker g for konsistens med filterets antakelser om gravitasjon
            gyro_data = self.get_gyro_data() # Grader per sekund

            # Juster for kalibrert statisk offset
            # NB: Vi korrigerer IKKE accel_z med 1.0 her, filteret bruker den rå (men biaskorrigerte) verdien.
            ax = accel_data['x'] - self.accel_offset['x']
            ay = accel_data['y'] - self.accel_offset['y']
            az = accel_data['z'] - (self.accel_offset['z'] - 1.0 if self.accel_offset['z'] > 0.5 else (self.accel_offset['z'] + 1.0 if self.accel_offset['z'] < -0.5 else self.accel_offset['z'])) # Korriger Z-biasen relativt til +1 eller -1 G
            # Enklere: Filteret normaliserer uansett, fjern bare gjennomsnittsverdien som målt
            az = accel_data['z'] - self.accel_offset['z']

            gx = gyro_data['x'] - self.gyro_offset['x']
            gy = gyro_data['y'] - self.gyro_offset['y']
            gz = gyro_data['z'] - self.gyro_offset['z']

            # Konverter gyro til radianer per sekund for Mahony-filteret
            gx_rad = gx * (math.pi / 180.0)
            gy_rad = gy * (math.pi / 180.0)
            gz_rad = gz * (math.pi / 180.0)

            # Kjør Mahony-oppdateringen
            self.update_mahony(gx_rad, gy_rad, gz_rad, ax, ay, az, dt)

            # Konverter den oppdaterte kvaternionen til Euler-vinkler (Roll, Pitch)
            # Roll (rotasjon rundt x-aksen)
            roll_rad = math.atan2(2.0 * (self.q0 * self.q1 + self.q2 * self.q3),
                                  1.0 - 2.0 * (self.q1 * self.q1 + self.q2 * self.q2))
            # Pitch (rotasjon rundt y-aksen)
            # Bruker - for q3*q1 for å matche standard definisjon
            pitch_arg = 2.0 * (self.q0 * self.q2 - self.q3 * self.q1)
            # Begrens argumentet til asin for å unngå mattefeil pga flyttall
            pitch_arg = max(-1.0, min(1.0, pitch_arg))
            pitch_rad = math.asin(pitch_arg)

            # Yaw (rotasjon rundt z-aksen) - beregnes men returneres ikke av denne funksjonen
            # yaw_rad = math.atan2(2.0 * (self.q0 * self.q3 + self.q1 * self.q2),
            #                     1.0 - 2.0 * (self.q2 * self.q2 + self.q3 * self.q3))

            # Konverter til grader
            roll_deg = roll_rad * (180.0 / math.pi)
            pitch_deg = pitch_rad * (180.0 / math.pi)
            # yaw_deg = yaw_rad * (180.0 / math.pi) # Kan legges til i returverdi om nødvendig

            return {'roll': roll_deg, 'pitch': pitch_deg}

        except IOError as e:
            print(f"IOError i get_orientation_mahony: {e}")
            # Returner siste kjente gyldige verdier eller None/standardverdier
            # For enkelhets skyld, returnerer 0 her, men bedre feilhåndtering kan vurderes
            return {'roll': 0.0, 'pitch': 0.0}
        except Exception as e:
            print(f"Uventet feil i get_orientation_mahony: {e}")
            return {'roll': 0.0, 'pitch': 0.0}


    # --- Funksjoner for FFT/periodisitet (uendret) ---
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
        if N < 10: return False # Trenger nok punkter for FFT
        fft_values = np.fft.rfft(signal)
        fft_magnitude = np.abs(fft_values) / N
        # Ignorer DC komponenten (første element) for terskelberegning? Kan være lurt.
        if len(fft_magnitude) > 1:
             max_mag = np.max(fft_magnitude[1:]) # Max uten DC
        else:
             max_mag = np.max(fft_magnitude)

        if max_mag == 0: return False # Ingen variasjon
        threshold = threshold_ratio * max_mag
        # Antall frekvenser *over* terskelen (ekskluder DC)
        significant_freqs = np.sum(fft_magnitude[1:] > threshold) if len(fft_magnitude) > 1 else 0
        return significant_freqs >= min_significant_freqs

    # --- Hovedfunksjon for status (oppdatert til å bruke Mahony) ---
    def gi_status_aks(self):
        try:
            # Hent akselerasjonsdata for total G og periodisitetsanalyse
            accel_data = self.get_accel_data(g=True) # Bruker g for enkel tolkning av tot_G
            # Korriger rådata for offset før beregning av total G? Kanskje ikke nødvendig her.
            tot_G = math.sqrt(accel_data['x']**2 + accel_data['y']**2 + accel_data['z']**2)

            # Legg til den nyeste målingen i en buffer for filtrering og FFT
            self.raw_data_buffer.append(tot_G)

            # Når bufferen har nok data, filtrer og vurder periodisiteten
            current_periodicity = self.last_periodicity_status # Behold forrige status til ny er klar
            if len(self.raw_data_buffer) == self.window_size:
                # Kopier data for å unngå å modifisere deque mens vi jobber
                data_list = list(self.raw_data_buffer)
                # Filtrer dataene
                filtered_data = self.highpass_filter(data_list, cutoff=0.5, fs=self.sample_rate, order=5)
                #self.data_buffer.extend(filtered_data) # Trenger vi denne bufferen?

                # Vurder periodisiteten basert på de filtrerte dataene
                # Bruk filtrerte data direkte
                is_periodic = self.is_periodic(filtered_data)
                current_periodicity = is_periodic
                self.last_periodicity_status = is_periodic # Oppdater status

                # Tøm bufferen for neste vindu? Nei, deque gjør dette automatisk.

            # Hent orienteringsdata ved hjelp av Mahony-filteret
            orientation = self.get_orientation_mahony()
            roll = orientation['roll']
            pitch = orientation['pitch']

            # Returner resultatene
            return {
                'total_G': tot_G,
                'roll': roll,
                'pitch': pitch,
                'is_periodic': current_periodicity # Returner siste gyldige status
            }

        except IOError as e:
             print(f"IOError i gi_status_aks: {e}")
             return {'total_G': 0.0, 'roll': 0.0, 'pitch': 0.0, 'is_periodic': None}
        except Exception as e:
            print(f"Uventet feil i gi_status_aks: {e}")
            return {'total_G': 0.0, 'roll': 0.0, 'pitch': 0.0, 'is_periodic': None}


# --- Hovedløkke (uendret, bruker nå Mahony via gi_status_aks) ---
if __name__ == "__main__":
    try:
        mpu = MPU6050_Orientation(0x68)
    except IOError as e:
        print(f"Kunne ikke initialisere MPU6050 på adresse 0x68: {e}")
        print("Sjekk tilkobling og I2C-buss.")
        exit()
    except Exception as e:
        print(f"En uventet feil oppstod under initialisering: {e}")
        exit()

    loop_count = 0
    start_time = time.time()

    while True:
        try:
            status = mpu.gi_status_aks()
            loop_count += 1

            # Skriv ut status med jevne mellomrom for lesbarhet
            if loop_count % 10 == 0: # Skriv ut ca. hver 10. gang (juster etter behov)
               print(f"Total G: {status['total_G']:.2f} | Roll: {status['roll']:.2f} | Pitch: {status['pitch']:.2f} | Periodisk: {status['is_periodic']}")

            # Oppretthold ønsket sample rate (ca.)
            # Dette er en enkel måte, mer nøyaktig timing kan kreve asyncio eller tråder
            current_run_time = time.time() - start_time
            expected_time = loop_count / mpu.sample_rate
            sleep_time = expected_time - current_run_time
            if sleep_time > 0.001: # Bare sov hvis det er mer enn 1 ms å vente
                 sleep(sleep_time)
            # Hvis koden kjører for sakte, vil sleep_time være negativ, og vi henger etter.

        except KeyboardInterrupt:
            print("Avslutter programmet.")
            break
        except IOError as e:
            print(f"IOError i hovedløkke: {e}. Fortsetter...")
            sleep(0.1) # Vent litt før neste forsøk
        except Exception as e:
            print(f"En uventet feil oppstod i hovedløkken: {e}")
            sleep(0.1)