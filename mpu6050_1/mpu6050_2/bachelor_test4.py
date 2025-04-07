from mpu6050 import mpu6050
from time import sleep
import math
import time

import os
import csv
from collections import deque
import numpy as np
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
        self.set_filter_range(self.FILTER_BW_188)

        # Kjør kalibrering
        self.gyro_offset = self.calibrate_gyro(100) # Økt antall samples for bedre kalibrering
        self.accel_offset = self.calibrate_accel(100) # Økt antall samples for bedre kalibrering

        self.gyro_angle_x = 0.0
        self.gyro_angle_y = 0.0
        self.alpha = 0.95 #Komplementær filtervekt
        self.last_time = time.time()

        # variabler for FFS / Stabilitetsvurdering
        self.sample_rate = 100  # Hz (Samples per sekund)
        # window_size bestemmer hvor mange samples som samles FØR vurder_stabilitet_G kalles
        self.window_size = self.sample_rate * 1 # Justert til 1 sekund (100 samples) for hyppigere oppdatering
                                                # Kommentaren i vurder_stabilitet_G nevner 0.5s (50 samples)
                                                # Endre til self.sample_rate * 0.5 hvis du vil matche kommentaren
                                                # Men koden kaller den per 'window_size'

        self.raw_data_buffer = deque(maxlen=self.window_size)
        # self.data_buffer er ikke i bruk slik koden står nå
        # self.data_buffer = deque(maxlen=self.window_size)

        # Nye variabler for den nye logikken i vurder_stabilitet_G
        self.stabilitet_historikk = [] # Liste for å lagre nivå (1, 2, 3)
        self.last_computed_stability_status = "Initialiserer..." # Holder siste beregnede status

    def calibrate_accel(self, samples=100):
        print(f"Starter kalibrering akselerometer ({samples} samples)...")
        offset = {'x':0.0, 'y':0.0, 'z': 0.0}
        for _ in range(samples):
            accel_data = self.get_accel_data(g=True)
            offset['x'] += accel_data['x']
            offset['y'] += accel_data['y']
            offset['z'] += accel_data['z']
            sleep(0.02) # Liten pause mellom målinger

        offset['x'] /= samples
        offset['y'] /= samples
        # Juster for gravitasjon på Z-aksen (forutsetter at MPU ligger flatt)
        offset['z'] = (offset['z'] / samples) - 1.0
        print("Ferdig kalibrert akselerometer.")
        print(f"Accel Offset: {offset}")
        return offset

    def calibrate_gyro(self, samples=100):
        print(f"Starter kalibrering gyroskop ({samples} samples)...")
        offset = {'x':0.0, 'y':0.0, 'z': 0.0}
        for _ in range(samples):
            gyro_data = self.get_gyro_data()
            offset['x'] += gyro_data['x']
            offset['y'] += gyro_data['y']
            offset['z'] += gyro_data['z']
            sleep(0.02) # Liten pause

        offset['x'] /= samples
        offset['y'] /= samples
        offset['z'] /= samples
        print("Ferdig kalibrert gyroskop.")
        print(f"Gyro Offset: {offset}")
        return offset

    def get_data(self): # Fjernet dt=0.01 da den ble overskrevet
        current_time = time.time()
        # dt = current_time - self.last_time # dt beregnes men brukes ikke her
        self.last_time = current_time

        #Hent rådata
        try:
            accel_data_raw = self.get_accel_data(g=True)
            gyro_data_raw = self.get_gyro_data()
        except Exception as e:
            print(f"Feil ved lesing fra MPU6050: {e}")
            # Returner dummy-data eller siste kjente verdi ved feil?
            return {'x': 0, 'y': 0, 'z': 1}, {'x': 0, 'y': 0, 'z': 0}


        # Kopier data for å unngå å endre originalen hvis den brukes andre steder
        accel_data = accel_data_raw.copy()
        gyro_data = gyro_data_raw.copy()

        #Juster for kalibrert bias
        accel_data['x'] -= self.accel_offset['x']
        accel_data['y'] -= self.accel_offset['y']
        accel_data['z'] -= self.accel_offset['z']

        gyro_data['x'] -= self.gyro_offset['x']
        gyro_data['y'] -= self.gyro_offset['y']
        gyro_data['z'] -= self.gyro_offset['z']

        return accel_data, gyro_data

    def butter_highpass(self, cutoff, fs, order=5): # butter høypass filter
        nyquist = 0.5 * fs
        normal_cutoff = cutoff / nyquist
        b, a = butter(order, normal_cutoff, btype='high', analog=False)
        return b, a

    def highpass_filter(self, data, cutoff, fs, order=5): # kaller butter_highpass filteret og gir 1 returverdi.
        b, a = self.butter_highpass(cutoff, fs, order=order)
        y = lfilter(b, a, data)
        return y

    def vurder_stabilitet_G(self, tot_G_values, ro_grense=1.0, normal_aktivitet_grense=1.1, høy_aktivitet_grense=1.1, toleranse=0.05):
        """
        Vurderer aktivitet basert på tot_G over tid. Følger logikk beskrevet i kommentarer.

        :param tot_G_values: Liste med tot_G verdier fra et tidsvindu.
        :param ro_grense: Grense for å anse som "i ro".
        :param normal_aktivitet_grense: Nedre grense for "middels" aktivitet. (Merk: Ikke brukt direkte i ny logikk, men definerer skillet).
        :param høy_aktivitet_grense: Nedre grense for "høy" aktivitet.
        :param toleranse: Toleranse rundt ro_grense for "Lav" klassifisering.
        :return: String "Lav", "Middels", "Høy", eller sist beregnede status.
        """
        if not tot_G_values: # Håndterer tilfellet med tom liste
            return self.last_computed_stability_status

        mean_tot_G = np.mean(tot_G_values)
        # print(f"Debug: Mean Tot G for window: {mean_tot_G:.3f}") # For feilsøking

        # --- Steg 1: Klassifiser gjennomsnittet av den nåværende datagruppen ---
        # Definer grensene tydeligere basert på input-parametere
        lav_øvre_grense = ro_grense + toleranse
        høy_nedre_grense = høy_aktivitet_grense + toleranse # Antar Høy er *over* denne grensen + toleranse

        nivå = 0 # Initialiser nivå
        if mean_tot_G <= lav_øvre_grense and mean_tot_G >= ro_grense - toleranse:
             # Hvis innenfor ro_grense +/- toleranse
            nivå = 1 # Lav aktivitet / I ro
        elif mean_tot_G > høy_nedre_grense:
            # Hvis over grensen for høy aktivitet
            nivå = 3 # Høy aktivitet
        else:
            # Alt annet (mellom Lav og Høy) blir Middels
            nivå = 2 # Middels aktivitet

        # --- Steg 2: Legg til klassifiseringen i historikken ---
        self.stabilitet_historikk.append(nivå)
        # print(f"Debug: Nivå: {nivå}, Historikk: {self.stabilitet_historikk}") # For feilsøking

        # --- Steg 3: Sjekk om historikken er full (10 verdier) ---
        if len(self.stabilitet_historikk) >= 10:
            # --- Steg 4: Beregn snitt, rund av ---
            snitt_nivå = np.mean(self.stabilitet_historikk)
            rundet_snitt = int(round(snitt_nivå)) # Rund av til nærmeste heltall
            # print(f"Debug: Historikk full. Snitt: {snitt_nivå:.2f}, Rundet: {rundet_snitt}") # For feilsøking

            # --- Steg 5: Map avrundet snitt til status-streng ---
            if rundet_snitt == 1:
                self.last_computed_stability_status = "Lav"
            elif rundet_snitt == 2:
                self.last_computed_stability_status = "Middels"
            elif rundet_snitt == 3:
                self.last_computed_stability_status = "Høy"
            else:
                # Skal i teorien ikke skje med nivåer 1, 2, 3
                self.last_computed_stability_status = f"Ukjent ({rundet_snitt})"

            # --- Steg 6: Tøm historikken for neste syklus ---
            self.stabilitet_historikk.clear()
            # print(f"Debug: Ny status: {self.last_computed_stability_status}. Historikk tømt.") # For feilsøking

        # --- Steg 7: Returner den sist BEREGNEDE statusen ---
        # Funksjonen returnerer alltid den sist lagrede statusen,
        # som kun oppdateres når historikken er full.
        return self.last_computed_stability_status


    # vurder_stabilitet_Gyro er uendret fra din kode
    def vurder_stabilitet_Gyro(self, gyro_values, stabil_grense=1, toleranse=0.1, høy_grense=1.2, stille_grense=0.2):
        """
        Funksjon som vurderer stabilitet basert på gjennomsnitt av gyro-magnitude.
        MERK: Input bør være gyro magnitude, ikke tot_G fra akselerometer. Navn kan være forvirrende.
        """
        # Beregn gjennomsnittet av de siste verdiene
        mean_tot_Gyro = np.mean(gyro_values) # Antar gyro_values er en liste med gyro magnitude

        # Vurder stabilitet basert på grensene
        if mean_tot_Gyro > høy_grense:
            return "Høy"
        elif abs(mean_tot_Gyro - stabil_grense) <= toleranse:
             # Dette sjekker om det er NÆRT stabil_grense (f.eks. 1 deg/s?). Sjelden for gyro?
            return "Stabil" # Kanskje "Moderat" er bedre?
        elif mean_tot_Gyro < stille_grense:
            return "Stille" # Veldig lav rotasjon
        else:
            # Mellom stille og høy, men ikke nær "stabil_grense"
            return "Ustabil" # Eller kanskje "Bevegelse"?


    def gi_status_aks(self):
        # Hent kalibrerte data
        accel_data, gyro_data = self.get_data()

        # Beregn total G (magnitude av akselerasjonsvektor)
        tot_G = math.sqrt(accel_data['x']**2 + accel_data['y']**2 + accel_data['z']**2)

        # Beregn total Gyro (magnitude av vinkelhastighetsvektor) - Hvis nødvendig for vurder_stabilitet_Gyro
        # tot_Gyro = math.sqrt(gyro_data['x']**2 + gyro_data['y']**2 + gyro_data['z']**2)

        # Legg til den nyeste tot_G målingen i bufferen
        self.raw_data_buffer.append(tot_G)

        # Behold den forrige statusen som standard returverdi
        status_fra_G = self.last_computed_stability_status

        # Når bufferen er full (har 'window_size' antall målinger)
        if len(self.raw_data_buffer) == self.window_size:
            # --- Filtrering (er kommentert ut) ---
            # filtered_data = self.highpass_filter(list(self.raw_data_buffer), cutoff=0.1, fs=self.sample_rate, order=5)
            # self.data_buffer.extend(filtered_data) # self.data_buffer brukes ikke videre

            # Send hele innholdet av bufferen til vurderingsfunksjonen
            # Konverter deque til liste før sending
            status_fra_G = self.vurder_stabilitet_G(list(self.raw_data_buffer))

            # Tøm bufferen for neste vindu (deque håndterer dette automatisk hvis maxlen nås,
            # men her vil vi tømme den helt etter behandling for å starte nytt vindu)
            # NB: .clear() tømmer deque'en. Siden vi sender dataene *før* clear(), er det ok.
            self.raw_data_buffer.clear()
            # self.data_buffer.clear() # Ikke nødvendig hvis ikke i bruk

        # Returner den øyeblikkelige G-verdien og den sist *beregnede* periodiske statusen
        return {
            'total_G': tot_G,
            'is_periodic': status_fra_G # Returnerer statusen fra vurder_stabilitet_G
        }


# --- Hovedprogram ---
if __name__ == "__main__":
    try:
        mpu = MPU6050_Orientation(0x68)
        print("MPU6050 initialisert. Starter datainnsamling...")

        output_interval = 1.0 # Sekunder mellom hver print til konsoll
        last_print_time = time.time()

        # CSV Fil setup
        file_path = "sensor_data_tot_G.csv"
        file_exists = os.path.exists(file_path)
        print(f"Logger data til: {file_path}")

        with open(file_path, "a", newline='') as file:
            writer = csv.writer(file)
            # Skriv header hvis filen er ny
            if not file_exists or os.path.getsize(file_path) == 0:
                writer.writerow(["Timestamp", "Total_G", "Beregnet_Status"]) # Lagt til Timestamp og Status

            while True:
                current_loop_time = time.time()
                status = mpu.gi_status_aks()

                # Skriv til CSV ved hver loop (eller sjeldnere om ønskelig)
                writer.writerow([current_loop_time, status['total_G'], status['is_periodic']])
                # Viktig for å sikre at data skrives med en gang (spesielt ved feil/stopp)
                file.flush()

                # Print status med jevne mellomrom
                if current_loop_time - last_print_time >= output_interval:
                    print(f"Tid: {current_loop_time:.2f} | Total G: {status['total_G']:.3f} | Status: {status['is_periodic']}")
                    last_print_time = current_loop_time

                # Sov for å unngå 100% CPU-bruk og for å nærme oss ønsket sample rate
                # MPU6050 har intern sampling, men lesehastigheten påvirker også.
                # sleep(0.01) gir teoretisk 100Hz, men I2C-kommunikasjon tar også tid.
                # Juster denne om nødvendig.
                sleep(0.005) # Litt kortere enn 1/100 for å kompensere for tid brukt i loopen

    except KeyboardInterrupt:
        print("\nAvslutter programmet.")
    except Exception as e:
        print(f"\nEn feil oppstod: {e}")